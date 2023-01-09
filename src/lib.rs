use anyhow::Result;
use apriltag::MatdRef;
use apriltag::{DetectorBuilder, Family, TagParams};
use mat2image::ToImage;
use opencv::core::Mat;
use opencv::prelude::MatTraitConstManual;
use opencv::prelude::VideoCaptureTrait;
use opencv::prelude::VideoCaptureTraitConst;
use opencv::{core::Scalar, highgui, imgproc, videoio};
use std::{thread, time::Duration, time::Instant};

// #[derive(Debug, Clone)]
// struct TagParamsArg {
//     pub tagsize: f64,
//     pub fx: f64,
//     pub fy: f64,
//     pub cx: f64,
//     pub cy: f64,
// }
//
// impl From<TagParamsArg> for TagParams {
//     fn from(arg: TagParamsArg) -> Self {
//         let TagParamsArg {
//             tagsize,
//             fx,
//             fy,
//             cx,
//             cy,
//         } = arg;
//
//         Self {
//             tagsize,
//             fx,
//             fy,
//             cx,
//             cy,
//         }
//     }
// }

// impl FromStr for TagParamsArg {
//     type Err = Error;
//
//     fn from_str(text: &str) -> Result<Self, Self::Err> {
//         let tokens: Vec<_> = text.split(',').collect();
//         ensure!(
//             tokens.len() == 5,
//             r#"tag parameters must be in format "tagsize,fx,fy,cx,cy""#
//         );
//
//         let values = tokens
//             .into_iter()
//             .map(|token| -> Result<_> {
//                 let value: f64 = token.parse().unwrap();
//                 Ok(value)
//             })
//             .collect::<Result<Vec<_>>>()
//             .with_context(|| format!("failed to parse tag parameters {}", text))?;
//
//         Ok(Self {
//             tagsize: values[0],
//             fx: values[1],
//             fy: values[2],
//             cx: values[3],
//             cy: values[4],
//         })
//     }
// }
//

fn fabs(x: f64) -> f64 {
    // On wasm32 we know that LLVM's intrinsic will compile to an optimized
    // `f64.abs` native instruction, so we can leverage this for both code size
    // and speed.
    f64::from_bits(x.to_bits() & (u64::MAX / 2))
}


const ATANHI: [f64; 4] = [
    4.63647609000806093515e-01, /* atan(0.5)hi 0x3FDDAC67, 0x0561BB4F */
    7.85398163397448278999e-01, /* atan(1.0)hi 0x3FE921FB, 0x54442D18 */
    9.82793723247329054082e-01, /* atan(1.5)hi 0x3FEF730B, 0xD281F69B */
    1.57079632679489655800e+00, /* atan(inf)hi 0x3FF921FB, 0x54442D18 */
];

const ATANLO: [f64; 4] = [
    2.26987774529616870924e-17, /* atan(0.5)lo 0x3C7A2B7F, 0x222F65E2 */
    3.06161699786838301793e-17, /* atan(1.0)lo 0x3C81A626, 0x33145C07 */
    1.39033110312309984516e-17, /* atan(1.5)lo 0x3C700788, 0x7AF0CBBD */
    6.12323399573676603587e-17, /* atan(inf)lo 0x3C91A626, 0x33145C07 */
];

const AT: [f64; 11] = [
    3.33333333333329318027e-01,  /* 0x3FD55555, 0x5555550D */
    -1.99999999998764832476e-01, /* 0xBFC99999, 0x9998EBC4 */
    1.42857142725034663711e-01,  /* 0x3FC24924, 0x920083FF */
    -1.11111104054623557880e-01, /* 0xBFBC71C6, 0xFE231671 */
    9.09088713343650656196e-02,  /* 0x3FB745CD, 0xC54C206E */
    -7.69187620504482999495e-02, /* 0xBFB3B0F2, 0xAF749A6D */
    6.66107313738753120669e-02,  /* 0x3FB10D66, 0xA0D03D51 */
    -5.83357013379057348645e-02, /* 0xBFADDE2D, 0x52DEFD9A */
    4.97687799461593236017e-02,  /* 0x3FA97B4B, 0x24760DEB */
    -3.65315727442169155270e-02, /* 0xBFA2B444, 0x2C6A6C2F */
    1.62858201153657823623e-02,  /* 0x3F90AD3A, 0xE322DA11 */
];

macro_rules! i {
    ($array:expr, $index:expr) => {
        unsafe { *$array.get_unchecked($index) }
    };
    ($array:expr, $index:expr, = , $rhs:expr) => {
        unsafe {
            *$array.get_unchecked_mut($index) = $rhs;
        }
    };
    ($array:expr, $index:expr, += , $rhs:expr) => {
        unsafe {
            *$array.get_unchecked_mut($index) += $rhs;
        }
    };
    ($array:expr, $index:expr, -= , $rhs:expr) => {
        unsafe {
            *$array.get_unchecked_mut($index) -= $rhs;
        }
    };
    ($array:expr, $index:expr, &= , $rhs:expr) => {
        unsafe {
            *$array.get_unchecked_mut($index) &= $rhs;
        }
    };
    ($array:expr, $index:expr, == , $rhs:expr) => {
        unsafe { *$array.get_unchecked_mut($index) == $rhs }
    };
}


fn atan(x: f64) -> f64 {
    let mut x = x;
    let mut ix = (x.to_bits() >> 32) as u32;
    let sign = ix >> 31;
    ix &= 0x7fff_ffff;
    if ix >= 0x4410_0000 {
        if x.is_nan() {
            return x;
        }

        let z = ATANHI[3] + f64::from_bits(0x0380_0000); // 0x1p-120f
        return if sign != 0 { -z } else { z };
    }

    let id = if ix < 0x3fdc_0000 {
        /* |x| < 0.4375 */
        if ix < 0x3e40_0000 {
            /* |x| < 2^-27 */
            if ix < 0x0010_0000 {
                /* raise underflow for subnormal x */
                unsafe {
                    core::ptr::read_volatile(&(x as f32));
                }
            }

            return x;
        }

        -1
    } else {
        x = f64::from_bits(x.to_bits() & (u64::MAX / 2));
        if ix < 0x3ff30000 {
            /* |x| < 1.1875 */
            if ix < 0x3fe60000 {
                /* 7/16 <= |x| < 11/16 */
                x = (2. * x - 1.) / (2. + x);
                0
            } else {
                /* 11/16 <= |x| < 19/16 */
                x = (x - 1.) / (x + 1.);
                1
            }
        } else if ix < 0x40038000 {
            /* |x| < 2.4375 */
            x = (x - 1.5) / (1. + 1.5 * x);
            2
        } else {
            /* 2.4375 <= |x| < 2^66 */
            x = -1. / x;
            3
        }
    };

    let z = x * x;
    let w = z * z;
    /* break sum from i=0 to 10 AT[i]z**(i+1) into odd and even poly */
    let s1 = z * (AT[0] + w * (AT[2] + w * (AT[4] + w * (AT[6] + w * (AT[8] + w * AT[10])))));
    let s2 = w * (AT[1] + w * (AT[3] + w * (AT[5] + w * (AT[7] + w * AT[9]))));

    if id < 0 {
        return x - x * (s1 + s2);
    }

    let z = i!(ATANHI, id as usize) - (x * (s1 + s2) - i!(ATANLO, id as usize) - x);

    if sign != 0 {
        -z
    } else {
        z
    }
}

fn sqrt(x: f64) -> f64 {
    #[cfg(target_feature = "sse2")]
    {
        #[cfg(target_arch = "x86")]
        use core::arch::x86::*;
        #[cfg(target_arch = "x86_64")]
        use core::arch::x86_64::*;
        unsafe {
            let m = _mm_set_sd(x);
            let m_sqrt = _mm_sqrt_pd(m);
            _mm_cvtsd_f64(m_sqrt)
        }
    }
    #[cfg(not(target_feature = "sse2"))]
    {
        use core::num::Wrapping;

        const TINY: f64 = 1.0e-300;

        let mut z: f64;
        let sign: Wrapping<u32> = Wrapping(0x80000000);
        let mut ix0: i32;
        let mut s0: i32;
        let mut q: i32;
        let mut m: i32;
        let mut t: i32;
        let mut i: i32;
        let mut r: Wrapping<u32>;
        let mut t1: Wrapping<u32>;
        let mut s1: Wrapping<u32>;
        let mut ix1: Wrapping<u32>;
        let mut q1: Wrapping<u32>;

        ix0 = (x.to_bits() >> 32) as i32;
        ix1 = Wrapping(x.to_bits() as u32);

        /* take care of Inf and NaN */
        if (ix0 & 0x7ff00000) == 0x7ff00000 {
            return x * x + x; /* sqrt(NaN)=NaN, sqrt(+inf)=+inf, sqrt(-inf)=sNaN */
        }
        /* take care of zero */
        if ix0 <= 0 {
            if ((ix0 & !(sign.0 as i32)) | ix1.0 as i32) == 0 {
                return x; /* sqrt(+-0) = +-0 */
            }
            if ix0 < 0 {
                return (x - x) / (x - x); /* sqrt(-ve) = sNaN */
            }
        }
        /* normalize x */
        m = ix0 >> 20;
        if m == 0 {
            /* subnormal x */
            while ix0 == 0 {
                m -= 21;
                ix0 |= (ix1 >> 11).0 as i32;
                ix1 <<= 21;
            }
            i = 0;
            while (ix0 & 0x00100000) == 0 {
                i += 1;
                ix0 <<= 1;
            }
            m -= i - 1;
            ix0 |= (ix1 >> (32 - i) as usize).0 as i32;
            ix1 = ix1 << i as usize;
        }
        m -= 1023; /* unbias exponent */
        ix0 = (ix0 & 0x000fffff) | 0x00100000;
        if (m & 1) == 1 {
            /* odd m, double x to make it even */
            ix0 += ix0 + ((ix1 & sign) >> 31).0 as i32;
            ix1 += ix1;
        }
        m >>= 1; /* m = [m/2] */

        /* generate sqrt(x) bit by bit */
        ix0 += ix0 + ((ix1 & sign) >> 31).0 as i32;
        ix1 += ix1;
        q = 0; /* [q,q1] = sqrt(x) */
        q1 = Wrapping(0);
        s0 = 0;
        s1 = Wrapping(0);
        r = Wrapping(0x00200000); /* r = moving bit from right to left */

        while r != Wrapping(0) {
            t = s0 + r.0 as i32;
            if t <= ix0 {
                s0 = t + r.0 as i32;
                ix0 -= t;
                q += r.0 as i32;
            }
            ix0 += ix0 + ((ix1 & sign) >> 31).0 as i32;
            ix1 += ix1;
            r >>= 1;
        }

        r = sign;
        while r != Wrapping(0) {
            t1 = s1 + r;
            t = s0;
            if t < ix0 || (t == ix0 && t1 <= ix1) {
                s1 = t1 + r;
                if (t1 & sign) == sign && (s1 & sign) == Wrapping(0) {
                    s0 += 1;
                }
                ix0 -= t;
                if ix1 < t1 {
                    ix0 -= 1;
                }
                ix1 -= t1;
                q1 += r;
            }
            ix0 += ix0 + ((ix1 & sign) >> 31).0 as i32;
            ix1 += ix1;
            r >>= 1;
        }

        /* use floating add to find out rounding direction */
        if (ix0 as u32 | ix1.0) != 0 {
            z = 1.0 - TINY; /* raise inexact flag */
            if z >= 1.0 {
                z = 1.0 + TINY;
                if q1.0 == 0xffffffff {
                    q1 = Wrapping(0);
                    q += 1;
                } else if z > 1.0 {
                    if q1.0 == 0xfffffffe {
                        q += 1;
                    }
                    q1 += Wrapping(2);
                } else {
                    q1 += q1 & Wrapping(1);
                }
            }
        }
        ix0 = (q >> 1) + 0x3fe00000;
        ix1 = q1 >> 1;
        if (q & 1) == 1 {
            ix1 |= sign;
        }
        ix0 += m << 20;
        f64::from_bits((ix0 as u64) << 32 | ix1.0 as u64)
    }
}

const PIO2_HI: f64 = 1.57079632679489655800e+00; /* 0x3FF921FB, 0x54442D18 */
const PIO2_LO: f64 = 6.12323399573676603587e-17; /* 0x3C91A626, 0x33145C07 */
/* coefficients for R(x^2) */
const P_S0: f64 = 1.66666666666666657415e-01; /* 0x3FC55555, 0x55555555 */
const P_S1: f64 = -3.25565818622400915405e-01; /* 0xBFD4D612, 0x03EB6F7D */
const P_S2: f64 = 2.01212532134862925881e-01; /* 0x3FC9C155, 0x0E884455 */
const P_S3: f64 = -4.00555345006794114027e-02; /* 0xBFA48228, 0xB5688F3B */
const P_S4: f64 = 7.91534994289814532176e-04; /* 0x3F49EFE0, 0x7501B288 */
const P_S5: f64 = 3.47933107596021167570e-05; /* 0x3F023DE1, 0x0DFDF709 */
const Q_S1: f64 = -2.40339491173441421878e+00; /* 0xC0033A27, 0x1C8A2D4B */
const Q_S2: f64 = 2.02094576023350569471e+00; /* 0x40002AE5, 0x9C598AC8 */
const Q_S3: f64 = -6.88283971605453293030e-01; /* 0xBFE6066C, 0x1B8D0159 */
const Q_S4: f64 = 7.70381505559019352791e-02; /* 0x3FB3B8C5, 0xB12E9282 */

fn comp_r(z: f64) -> f64 {
    let p = z * (P_S0 + z * (P_S1 + z * (P_S2 + z * (P_S3 + z * (P_S4 + z * P_S5)))));
    let q = 1.0 + z * (Q_S1 + z * (Q_S2 + z * (Q_S3 + z * Q_S4)));
    p / q
}

#[inline]
fn get_high_word(x: f64) -> u32 {
    (x.to_bits() >> 32) as u32
}

#[inline]
fn get_low_word(x: f64) -> u32 {
    x.to_bits() as u32
}

#[inline]
fn with_set_low_word(f: f64, lo: u32) -> f64 {
    let mut tmp = f.to_bits();
    tmp &= 0xffff_ffff_0000_0000;
    tmp |= lo as u64;
    f64::from_bits(tmp)
}

fn asin(mut x: f64) -> f64 {
    let z: f64;
    let r: f64;
    let s: f64;
    let hx: u32;
    let ix: u32;

    hx = get_high_word(x);
    ix = hx & 0x7fffffff;
    /* |x| >= 1 or nan */
    if ix >= 0x3ff00000 {
        let lx: u32;
        lx = get_low_word(x);
        if ((ix - 0x3ff00000) | lx) == 0 {
            /* asin(1) = +-pi/2 with inexact */
            return x * PIO2_HI + f64::from_bits(0x3870000000000000);
        } else {
            return 0.0 / (x - x);
        }
    }
    /* |x| < 0.5 */
    if ix < 0x3fe00000 {
        /* if 0x1p-1022 <= |x| < 0x1p-26, avoid raising underflow */
        if ix < 0x3e500000 && ix >= 0x00100000 {
            return x;
        } else {
            return x + x * comp_r(x * x);
        }
    }
    /* 1 > |x| >= 0.5 */
    z = (1.0 - fabs(x)) * 0.5;
    s = sqrt(z);
    r = comp_r(z);
    if ix >= 0x3fef3333 {
        /* if |x| > 0.975 */
        x = PIO2_HI - (2. * (s + s * r) - PIO2_LO);
    } else {
        let f: f64;
        let c: f64;
        /* f+c = sqrt(z) */
        f = with_set_low_word(s, 0);
        c = (z - f * f) / (s + f);
        x = 0.5 * PIO2_HI - (2.0 * s * r - (PIO2_LO - 2.0 * c) - (0.5 * PIO2_HI - 2.0 * f));
    }
    if hx >> 31 != 0 {
        -x
    } else {
        x
    }
}

const PI: f64 = 3.1415926535897931160E+00; /* 0x400921FB, 0x54442D18 */
const PI_LO: f64 = 1.2246467991473531772E-16; /* 0x3CA1A626, 0x33145C07 */

fn atan2(y: f64, x: f64) -> f64 {
    if x.is_nan() || y.is_nan() {
        return x + y;
    }
    let mut ix = (x.to_bits() >> 32) as u32;
    let lx = x.to_bits() as u32;
    let mut iy = (y.to_bits() >> 32) as u32;
    let ly = y.to_bits() as u32;
    if ((ix.wrapping_sub(0x3ff00000)) | lx) == 0 {
        /* x = 1.0 */
        return atan(y);
    }
    let m = ((iy >> 31) & 1) | ((ix >> 30) & 2); /* 2*sign(x)+sign(y) */
    ix &= 0x7fffffff;
    iy &= 0x7fffffff;

    /* when y = 0 */
    if (iy | ly) == 0 {
        return match m {
            0 | 1 => y, /* atan(+-0,+anything)=+-0 */
            2 => PI,    /* atan(+0,-anything) = PI */
            _ => -PI,   /* atan(-0,-anything) =-PI */
        };
    }
    /* when x = 0 */
    if (ix | lx) == 0 {
        return if m & 1 != 0 { -PI / 2.0 } else { PI / 2.0 };
    }
    /* when x is INF */
    if ix == 0x7ff00000 {
        if iy == 0x7ff00000 {
            return match m {
                0 => PI / 4.0,        /* atan(+INF,+INF) */
                1 => -PI / 4.0,       /* atan(-INF,+INF) */
                2 => 3.0 * PI / 4.0,  /* atan(+INF,-INF) */
                _ => -3.0 * PI / 4.0, /* atan(-INF,-INF) */
            };
        } else {
            return match m {
                0 => 0.0,  /* atan(+...,+INF) */
                1 => -0.0, /* atan(-...,+INF) */
                2 => PI,   /* atan(+...,-INF) */
                _ => -PI,  /* atan(-...,-INF) */
            };
        }
    }
    /* |y/x| > 0x1p64 */
    if ix.wrapping_add(64 << 20) < iy || iy == 0x7ff00000 {
        return if m & 1 != 0 { -PI / 2.0 } else { PI / 2.0 };
    }

    /* z = atan(|y/x|) without spurious underflow */
    let z = if (m & 2 != 0) && iy.wrapping_add(64 << 20) < ix {
        /* |y/x| < 0x1p-64, x<0 */
        0.0
    } else {
        atan(fabs(y / x))
    };
    match m {
        0 => z,                /* atan(+,+) */
        1 => -z,               /* atan(-,+) */
        2 => PI - (z - PI_LO), /* atan(+,-) */
        _ => (z - PI_LO) - PI, /* atan(-,-) */
    }
}

struct Rotation {
    pub heading: f64,
    pub roll: f64,
    pub pitch: f64,
}

fn matrix_to_heading(m: MatdRef) -> Rotation {
    let rc = m.nrows();
    let lc = m.ncols();
    let d = m.data();
    assert!(rc == 3 && lc == 3);
    // https://community.esri.com/t5/net-maps-sdk-questions/how-do-i-convert-from-a-rotation-matrix-to-heading/td-p/729775
    if !(d[0][2] == 1.0 || d[0][2] == - 1.0) {
        return Rotation {
            heading: (atan2(d[0][1], m[2][2]) * 180 / PI),
            roll: (atan2(d[1][2], d[2][2]) * 180 / PI),
            pitch: (asin(d[0][2])) as f32,
        }
    }
    unimplemented!();
}

pub fn detect_loop() -> Result<()> {
    let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;
    let res = (800.0, 440.0);
    highgui::named_window("seancv", 1)?;
    cam.set(3, res.0)?;
    cam.set(4, res.1)?;
    let _ = videoio::VideoCapture::is_opened(&cam)?;
    let family: Family = Family::tag_16h5();
    // let tag_params: Option<TagParams> = tag_params.map(|params| params.into());
    let mut detector = DetectorBuilder::new()
        .add_family_bits(family, 1)
        .build()
        .unwrap();

    let mut start = Instant::now();
    let mut frame_num = 0;
    let mut first = true;
    loop {
        let frame_time = Instant::now();
        let mut frame = Mat::default();
        if cam.read(&mut frame).is_err() {
            thread::sleep(Duration::from_millis(50));
            if let Ok(v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                cam = v;
            }
            let _ = highgui::named_window("seancv", 1);
            let _ = cam.set(3, res.0);
            let _ = cam.set(4, res.1);
            let _ = videoio::VideoCapture::is_opened(&cam);
            continue;
        }
        // if one frame took a long time we can ignore
        if frame_num % 10 == 0 {
            start = Instant::now();
            frame_num = 0;
        }
        frame_num += 1;
        let calc_time = Instant::now();
        if frame.size()?.width == 0 || frame.size()?.height == 0 {
            thread::sleep(Duration::from_millis(50));
        }

        // if input_files.is_empty() {
        //     eprintln!("no input files");
        //     return Ok(());
        // }

        let Ok(frame_img) = frame.to_image() else {
            thread::sleep(Duration::from_millis(50));
            if let Ok(v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                cam = v;
            }
            let _ = highgui::named_window("seancv", 1);
            let _ = cam.set(3, res.0);
            let _ = cam.set(4, res.1);
            let _ = videoio::VideoCapture::is_opened(&cam);
            continue;
        };
        // let DynamicImage::ImageLuma8(frame) = frame else {
        //     unreachable!();
        // };
        let detections = detector.detect(frame_img.to_luma8());
        // let detections = detector.detect(frame.to_image().unwrap().as_flat_samples_u8().unwrap());

        if first {
            start = Instant::now();
            first = false;
        }
        for det in detections {
            if det.decision_margin() < 50.0 {
                continue;
            }
            println!("fps {:?}", frame_num as f32 / start.elapsed().as_secs_f32());
            let pose = det.estimate_tag_pose(&TagParams {
                tagsize: 0.152,
                fx: 578.272,
                fy: 578.272,
                cx: 402.145,
                cy: 221.506,
            });
            // println!("  - pose {}: {:#?}", index, pose);
            if let Some(pose) = pose {
                println!("id {:?}", det.id());
                println!("translation {:?}", pose.translation());
                let t = pose.translation().data();
                let x = t[0];
                let y = t[1];
                let z = t[2];
                println!(" x {}", x / 0.0254);
                println!(" y {}", y / 0.0254);
                println!(" z {}", z / 0.0254);
                println!(" mag {}", (x * x + z * z).sqrt() / 0.0254);
            }
            let corners = det.corners();
            let center = det.center();
            let _ = imgproc::line(
                &mut frame,
                opencv::core::Point_::new(corners[0][0] as i32, corners[0][1] as i32),
                opencv::core::Point_::new(corners[1][0] as i32, corners[1][1] as i32),
                Scalar::new(255f64, 255f64, 0f64, 0f64), // color value
                4,
                2,
                0,
            );
            let _ = imgproc::line(
                &mut frame,
                opencv::core::Point_::new(corners[0][0] as i32, corners[0][1] as i32),
                opencv::core::Point_::new(corners[3][0] as i32, corners[3][1] as i32),
                Scalar::new(255f64, 255f64, 0f64, 0f64), // color value
                4,
                2,
                0,
            );
            let _ = imgproc::line(
                &mut frame,
                opencv::core::Point_::new(corners[3][0] as i32, corners[3][1] as i32),
                opencv::core::Point_::new(corners[2][0] as i32, corners[2][1] as i32),
                Scalar::new(255f64, 255f64, 0f64, 0f64), // color value
                4,
                2,
                0,
            );
            let _ = imgproc::line(
                &mut frame,
                opencv::core::Point_::new(corners[2][0] as i32, corners[2][1] as i32),
                opencv::core::Point_::new(corners[1][0] as i32, corners[1][1] as i32),
                Scalar::new(255f64, 255f64, 0f64, 0f64), // color value
                4,
                2,
                0,
            );
            let _ = imgproc::circle(
                &mut frame,
                opencv::core::Point_::new(center[0] as i32, center[1] as i32),
                5,
                Scalar::new(255f64, 255f64, 0f64, 0f64),
                2,
                2,
                0,
            );
        };
        println!("capture {:?}", (calc_time - frame_time).as_millis());
        println!("calc {:?}", calc_time.elapsed().as_millis());
        println!("total {:?}", frame_time.elapsed().as_millis());
        highgui::imshow("seancv", &frame)?;
        if highgui::wait_key(1)? > 0 {
            continue;
        }
    }
}
