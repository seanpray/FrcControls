use anyhow::Result;
use apriltag::MatdRef;
use apriltag::{DetectorBuilder, Family, TagParams};
use mat2image::ToImage;
use opencv::core::Mat;
use opencv::prelude::MatTraitConstManual;
use opencv::prelude::VideoCaptureTrait;
use opencv::prelude::VideoCaptureTraitConst;
use opencv::{core::Scalar, highgui, imgproc, videoio};
use serde::Deserialize;
use std::error::Error;
use std::f64::consts::FRAC_PI_2;
use std::f64::consts::PI;
use std::fs::read_to_string;
use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex, RwLock};
use std::time::SystemTime;
use std::{thread, time::Duration, time::Instant};

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct Quaternion {
    W: f64,
    X: f64,
    Y: f64,
    Z: f64,
}

#[derive(Deserialize)]
struct TagRotation {
    quaternion: Quaternion,
}

#[derive(Deserialize)]
struct Translation {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Deserialize)]
struct Pose {
    translation: Translation,
    rotation: TagRotation,
}

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct Tag {
    ID: u8,
    pose: Pose,
}

#[derive(Deserialize)]
struct FieldData {
    length: f64,
    width: f64,
}

#[derive(Deserialize)]
pub(crate) struct Field {
    tags: Vec<Tag>,
    field: FieldData,
}

impl Field {
    pub fn from_file<P: AsRef<Path>>(file: P) -> Result<Self, Box<dyn Error>> {
        let data = read_to_string(file)?;
        let data: Field = serde_json::from_str(&data)?;
        Ok(data)
    }

    pub fn reload_field<P: AsRef<Path>>(file: P) -> bool {
        match Self::from_file(file) {
            Ok(v) => {
                *FIELD_DATA.write().unwrap() = v;
                true
            }
            _ => false,
        }
    }
}

struct NetworkTableEntries {
    heading: f64,
    location_x: f64,
    location_z: f64,
    location_y: f64,
    tolerance: f64,
    rotational_tolerance: f64,
    invalid_tags: f64,
    velocity_x: f64,
    velocity_z: f64,
    velocity_y: f64,
    frame_time: u128,
}

impl NetworkTableEntries {
    #[allow(clippy::too_many_arguments)]
    pub fn generate(
        heading: f64,
        roll: f64,
        pitch: f64,
        location_x: f64,
        location_z: f64,
        location_y: f64,
        tolerance: f64,
        rotational_tolerance: f64,
        invalid_tags: f64,
        velocity_x: f64,
        velocity_z: f64,
        velocity_y: f64,
    ) -> Self {
        Self {
            heading,
            location_x,
            location_z,
            location_y,
            tolerance,
            rotational_tolerance,
            invalid_tags,
            velocity_x,
            velocity_z,
            velocity_y,
            frame_time: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis(),
        }
    }
    pub fn publish_changes(&self) {
        NetworkTables::connect(&(IP.lock().unwrap()), "vision");
    }
}

pub enum OP {
    ReloadField,
}

pub const FIELD_DATA_NAME: &str = "2023-chargedup.json";

lazy_static::lazy_static! {
//     pub (crate) static ref DISPLAY_CACHE: Arc<Mutex<Vec<ScreenData>>> = Arc::new(Mutex::new(vec![]));
    pub (crate) static ref FIELD_DATA: Arc<RwLock<Field>> = Arc::new(RwLock::new(Field::from_file(FIELD_DATA_NAME).unwrap()));
    pub (crate) static ref RUNTIME_CACHE: Arc<Mutex<Vec<OP>>> = Arc::new(Mutex::new(vec![]));
    pub (crate) static ref INSTRUCTION_UPDATE: AtomicBool = AtomicBool::from(false);
}

#[inline(always)]
fn fabs(x: f64) -> f64 {
    // On wasm32 we know that LLVM's intrinsic will compile to an optimized
    // `f64.abs` native instruction, so we can leverage this for both code size
    // and speed.
    f64::from_bits(x.to_bits() & (u64::MAX / 2))
}

const ATANHI: [f64; 4] = [
    4.636_476_090_008_061e-1, /* atan(0.5)hi 0x3FDDAC67, 0x0561BB4F */
    7.853_981_633_974_483e-1, /* atan(1.0)hi 0x3FE921FB, 0x54442D18 */
    9.827_937_232_473_29e-1,  /* atan(1.5)hi 0x3FEF730B, 0xD281F69B */
    FRAC_PI_2,                /* atan(inf)hi 0x3FF921FB, 0x54442D18 */
];

const ATANLO: [f64; 4] = [
    2.269_877_745_296_168_7e-17, /* atan(0.5)lo 0x3C7A2B7F, 0x222F65E2 */
    3.061_616_997_868_383e-17,   /* atan(1.0)lo 0x3C81A626, 0x33145C07 */
    1.390_331_103_123_099_8e-17, /* atan(1.5)lo 0x3C700788, 0x7AF0CBBD */
    6.123_233_995_736_766e-17,   /* atan(inf)lo 0x3C91A626, 0x33145C07 */
];

const AT: [f64; 11] = [
    3.333_333_333_333_293e-1,    /* 0x3FD55555, 0x5555550D */
    -1.999_999_999_987_648_3e-1, /* 0xBFC99999, 0x9998EBC4 */
    1.428_571_427_250_346_6e-1,  /* 0x3FC24924, 0x920083FF */
    -1.111_111_040_546_235_6e-1, /* 0xBFBC71C6, 0xFE231671 */
    9.090_887_133_436_507e-2,    /* 0x3FB745CD, 0xC54C206E */
    -7.691_876_205_044_83e-2,    /* 0xBFB3B0F2, 0xAF749A6D */
    6.661_073_137_387_531e-2,    /* 0x3FB10D66, 0xA0D03D51 */
    -5.833_570_133_790_573_5e-2, /* 0xBFADDE2D, 0x52DEFD9A */
    4.976_877_994_615_932_4e-2,  /* 0x3FA97B4B, 0x24760DEB */
    -3.653_157_274_421_691_6e-2, /* 0xBFA2B444, 0x2C6A6C2F */
    1.628_582_011_536_578_2e-2,  /* 0x3F90AD3A, 0xE322DA11 */
];

#[inline(always)]
pub fn calc_proportion(w: f64, h: f64) -> f64 {
    (1.0 / 8.0 * w * w) / h + 0.5 * h
}

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

#[inline(always)]
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

#[inline(always)]
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

const PIO2_HI: f64 = FRAC_PI_2; /* 0x3FF921FB, 0x54442D18 */
const PIO2_LO: f64 = 6.123_233_995_736_766e-17; /* 0x3C91A626, 0x33145C07 */
/* coefficients for R(x^2) */
const P_S0: f64 = 1.666_666_666_666_666_6e-1; /* 0x3FC55555, 0x55555555 */
const P_S1: f64 = -3.255_658_186_224_009e-1; /* 0xBFD4D612, 0x03EB6F7D */
const P_S2: f64 = 2.012_125_321_348_629_3e-1; /* 0x3FC9C155, 0x0E884455 */
const P_S3: f64 = -4.005_553_450_067_941e-2; /* 0xBFA48228, 0xB5688F3B */
const P_S4: f64 = 7.915_349_942_898_145e-4; /* 0x3F49EFE0, 0x7501B288 */
const P_S5: f64 = 3.479_331_075_960_212e-5; /* 0x3F023DE1, 0x0DFDF709 */
const Q_S1: f64 = -2.403_394_911_734_414; /* 0xC0033A27, 0x1C8A2D4B */
const Q_S2: f64 = 2.020_945_760_233_505_7; /* 0x40002AE5, 0x9C598AC8 */
const Q_S3: f64 = -6.882_839_716_054_533e-1; /* 0xBFE6066C, 0x1B8D0159 */
const Q_S4: f64 = 7.703_815_055_590_194e-2; /* 0x3FB3B8C5, 0xB12E9282 */

#[inline(always)]
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

#[inline(always)]
fn asin(mut x: f64) -> f64 {
    let hx = get_high_word(x);
    let ix = hx & 0x7fffffff;
    /* |x| >= 1 or nan */
    if ix >= 0x3ff00000 {
        let lx = get_low_word(x);
        if ((ix - 0x3ff00000) | lx) == 0 {
            /* asin(1) = +-pi/2 with inexact */
            return x * PIO2_HI + f64::from_bits(0x3870000000000000);
        }
        return 0.0;
    }
    /* |x| < 0.5 */
    if ix < 0x3fe00000 {
        /* if 0x1p-1022 <= |x| < 0x1p-26, avoid raising underflow */
        if (0x00100000..0x3e500000).contains(&ix) {
            return x;
        } else {
            return x + x * comp_r(x * x);
        }
    }
    /* 1 > |x| >= 0.5 */
    let z = (1.0 - fabs(x)) * 0.5;
    let s = sqrt(z);
    let r = comp_r(z);
    if ix >= 0x3fef3333 {
        /* if |x| > 0.975 */
        x = PIO2_HI - (2. * (s + s * r) - PIO2_LO);
    } else {
        /* f+c = sqrt(z) */
        let f = with_set_low_word(s, 0);
        let c = (z - f * f) / (s + f);
        x = 0.5 * PIO2_HI - (2.0 * s * r - (PIO2_LO - 2.0 * c) - (0.5 * PIO2_HI - 2.0 * f));
    }
    if hx >> 31 != 0 {
        -x
    } else {
        x
    }
}

const PI_LO: f64 = 1.224_646_799_147_353_2E-16; /* 0x3CA1A626, 0x33145C07 */

#[inline(always)]
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

impl ToString for Rotation {
    fn to_string(&self) -> String {
        format!(
            "heading: {}\nroll: {}\npitch: {}",
            self.heading, self.roll, self.pitch
        )
    }
}

#[allow(unused_assignments)]
#[inline(always)]
fn matrix_to_heading(m: MatdRef) -> Rotation {
    let mut psi = 0.0;
    let mut theta = 0.0;
    let mut phi = 0.0;
    let rc = m.nrows();
    let lc = m.ncols();
    let d = m.data();
    assert!(rc == 3 && lc == 3);
    // https://community.esri.com/t5/net-maps-sdk-questions/how-do-i-convert-from-a-rotation-matrix-to-heading/td-p/729775
    if !(d[2] == 1.0 || d[2] == -1.0) {
        phi = atan2(d[1], d[0]);
        psi = atan2(d[rc + 2], d[2 * rc + 2]);
        theta = asin(d[2]);
    } else if d[2] == -1.0 {
        theta = PI / 2.0;
        psi = phi + atan2(d[rc], d[rc * 2]);
    } else {
        theta = -PI / 2.0;
        psi = -phi + atan2(-d[rc], -d[rc * 2]);
    }
    Rotation {
        heading: theta * 180.0 / PI,
        roll: phi * 180.0 / PI,
        pitch: psi * 180.0 / PI,
    }
}

// take x and z position then apply a counter clockwise linear transformation to adjust the offset
#[inline(always)]
fn linear_transform(theta: f64, x: f64, z: f64) -> (f64, f64) {
    (
        x * theta.cos() - x * theta.sin(),
        x * theta.sin() + x * theta.cos(),
    )
}

macro_rules! compute {
    // take xz tag data, convert x and y based on id
    ($data:expr, $angle:expr, $tag:expr) => {{
        let d = &$data.tags[$tag].pose;
        let t = &d.translation;
        let mag = (t.x.powf(2.0) + t.z.powf(2.0)).sqrt();
        // let (x, y) = linear_transform($angle, t.x, t.z);
        (mag * $angle.cos(), mag * $angle.sin())
    }};
}

// transform based on camera id
// compute robot position from static tag position
#[inline(always)]
fn compute_offset(cam_id: usize, id: usize, x: f64, y: f64, z: f64) -> Option<(f64, f64, f64)> {
    let f_data = FIELD_DATA.read().unwrap();
    if id >= f_data.tags.len() {
        return None;
    }
    if cam_id > 2 {
        return None;
    }
    let (x_i, y_i) = compute!(f_data, cam_id as f64 * 30.0, id);
    Some((x_i - x, y_i - y, z))
}

// TODO: stick inside event loop, look up stuff from network table to add/remove instructions on
// different thread
fn compute_instruction() {
    if INSTRUCTION_UPDATE.load(std::sync::atomic::Ordering::Relaxed) {
        INSTRUCTION_UPDATE.swap(false, std::sync::atomic::Ordering::Relaxed);
        let Some(v) = RUNTIME_CACHE.lock().unwrap().pop() else {
            return;
        };
        match v {
            OP::ReloadField => {
                let _ = Field::reload_field(FIELD_DATA_NAME);
                // maybe log on failure?
            }
            _ => {}
        }
    }
}

fn quaternion_to_rotation(q: Quaternion) -> Rotation {
    Rotation {
        heading: atan2(
            2.0 * (q.Z * q.W + q.X * q.Y),
            -1.0 + 2.0 * (q.W * q.W + q.X * q.X),
        ),
        roll: atan2(
            2.0 * (q.Z * q.Y + q.W * q.X),
            1.0 - 2.0 * (q.X * q.X + q.Y * q.Y),
        ),
        pitch: atan(2.0 * (q.Y * q.W - q.Z * q.X)),
    }
}
/*

            let pose = det.estimate_tag_pose(&TagParams {
                tagsize: 0.152,
                fx: 578.272,
                fy: 578.272,
                cx: 402.145,
                cy: 221.506,
            });
 */

// NOTE: this calibration is for the C920 webcam at 800x448.
pub struct CameraMatrix {
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    x: f64,
    y: f64
}

impl Default for CameraMatrix {
    fn default() -> Self {
        Self {
            fx: 578.272,
            fy: 578.272,
            // origin of focal plane
            cx: 402.145,
            cy: 221.506,
            x: 800.0,
            y: 448.0,
        }
    }
}

// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

impl CameraMatrix {
    pub fn scaled(x: f64, y: f64) -> Self {
        let default = Self::default();
        Self {
            fx: default.fx * (x / default.x),
            fy: default.fy * (y / default.y),
            ..default
        }
    }
}

static G_ACCEL: f64 = 9.80665;

// TODO use macro for input
// kg, drag coef, time step, x, z, y, tolerance (ft)
// position 0 is robot, target is relative
#[allow(non_snake_case)]
fn eulers_method_drag(
    mass: f64,
    k: f64,
    dt: f64,
    v: (f64, f64, f64),
    a: (f64, f64, f64),
    target: (f64, f64, f64),
    epsilon: f64,
    max_iter: usize,
) {
    let mut t = dt;
    let mut x = 0.0;
    let mut z = 0.0;
    let mut y = 0.0;
    let mut Vx = v.0;
    let mut Vz = v.1;
    let mut Vy = v.2;
    let mut Ax = a.0;
    // need to measure realistic accel
    let mut Az = a.1;
    let mut Ay = a.2 - G_ACCEL;
    let mut iter = 0;
    while iter < max_iter
        && ((x - target.0).abs() < epsilon
            && (z - target.1).abs() < epsilon
            && (y - target.2).abs() < epsilon)
    {
        x += dt * Vx;
        z += dt * Vz;
        y += dt * Vy;
        Vx += Ax - k * Vx.powf(2.0) * dt;
        Vz += Az - k * Vz.powf(2.0) * dt;
        Vy += Ay - k * Vy.powf(2.0) * dt;
        t += dt;
        iter += 1;
    }
}

static RES: (f64, f64) = (640.0, 480.0);

pub fn detect_loop_multithreaded(cam_index: i32, threads: usize) -> Result<()> {
    let mut cam = videoio::VideoCapture::new(cam_index, videoio::CAP_ANY)?;
    cam.set(3, RES.0)?;
    cam.set(4, RES.1)?;
    let _ = videoio::VideoCapture::is_opened(&cam)?;
    // let tag_params: Option<TagParams> = tag_params.map(|params| params.into());
    thread::spawn(move || {
        let family: Family = Family::tag_16h5();
        let mut detector = DetectorBuilder::new()
            .add_family_bits(family, 1)
            .build()
            .unwrap();
        let mut start = Instant::now();
        let mut frame_num = 0;
        let mut first = true;
        let mut avg_frame_time = 0;
        loop {
            let frame_time = Instant::now();
            let mut frame = Mat::default();
            if cam.read(&mut frame).is_err() {
                thread::sleep(Duration::from_millis(50));
                if let Ok(mut v) = videoio::VideoCapture::new(0, videoio::CAP_DSHOW) {
                    // let _ = v.set(videoio::CAP_PROP_EXPOSURE, -10.0);
                    cam = v;
                }
                let _ = cam.set(3, RES.0);
                let _ = cam.set(4, RES.1);
                let _ = videoio::VideoCapture::is_opened(&cam);
                continue;
            }
            // if one frame took a long time we can ignore
            if frame_num % 30 == 0 {
                start = Instant::now();
                frame_num = 0;
            }
            frame_num += 1;
            let calc_time = Instant::now();
            if frame.size().unwrap_or_default().width == 0
                || frame.size().unwrap_or_default().height == 0
            {
                thread::sleep(Duration::from_millis(10));
            }

            // if input_files.is_empty() {
            //     eprintln!("no input files");
            //     return Ok(());
            // }

            let Ok(frame_img) = frame.to_image() else {
                thread::sleep(Duration::from_millis(1));
                if let Ok(v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                    cam = v;
                }
                let _ = cam.set(3, RES.0);
                let _ = cam.set(4, RES.1);
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
                if det.decision_margin() < 40.0 {
                    continue;
                }
                let c = CameraMatrix::scaled(RES.0, RES.1);
                let pose = det.estimate_tag_pose(&TagParams {
                    tagsize: 0.152,
                    fx: c.fx,
                    fy: c.fy,
                    cx: c.cx,
                    cy: c.cy,
                });
                // println!("  - pose {}: {:#?}", index, pose);
                if let Some(pose) = pose {
                    println!("id {:?}", det.id());
                    println!("translation {:?}", pose.translation());
                    println!("{}", matrix_to_heading(pose.rotation()).to_string());
                    let t = pose.translation().data();
                    let x = t[0];
                    let y = t[1];
                    let z = t[2];
                    println!(" x {}", x / 0.0254);
                    println!(" y {}", y / 0.0254);
                    println!(" z {}", z / 0.0254);
                    // 39.3700787402 is 1 / 0.0254
                    println!(" mag {}", (x * x + z * z).sqrt() * 39.3700787402);
                }
            }
            println!("capture {:?}", (calc_time - frame_time).as_millis());
            println!("calc {:?}", calc_time.elapsed().as_millis());
            println!("total {:?}", frame_time.elapsed().as_millis());
            println!("fps {:?}", frame_num as f32 / start.elapsed().as_secs_f32());
            // *(DISPLAY_CACHE.lock()).push(frame.clone());
        }
    });
    let mut worker_threads = Vec::with_capacity(threads);
    for _ in 0..threads {
        let t = thread::spawn(move || {
            let family: Family = Family::tag_16h5();
            let mut detector = DetectorBuilder::new()
                .add_family_bits(family, 1)
                .build()
                .unwrap();
            thread::park();
            // stuff
        });
        worker_threads.push(t);
    }
    Ok(())
}

pub fn detect_loop_single(cam_index: i32) -> Result<()> {
    let mut cam = videoio::VideoCapture::new(cam_index, videoio::CAP_ANY)?;
    cam.set(3, RES.0)?;
    cam.set(4, RES.1)?;
    let _ = videoio::VideoCapture::is_opened(&cam)?;
    let family: Family = Family::tag_16h5();
    let _ = cam.set(
        videoio::CAP_PROP_FOURCC,
        videoio::VideoWriter::fourcc('M', 'J', 'P', 'G')
            .unwrap()
            .into(),
    );
    let _ = cam.set(
        videoio::CAP_PROP_XI_EXPOSURE,
        25000.0,
    );
    // let _ = cam.set(videoio::CAP_OPENCV_MJPEG, 1.0);
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
            if let Ok(mut v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                // let _ = v.set(videoio::CAP_PROP_EXPOSURE, -10.0);
                let _ = v.set(videoio::CAP_PROP_FPS, 60.0);
                cam = v;
            }
            let _ = highgui::named_window(&format!("seancv{cam_index}"), 1);
            let _ = cam.set(3, RES.0);
            let _ = cam.set(4, RES.1);
            let _ = cam.set(
                videoio::CAP_PROP_XI_EXPOSURE,
                2500.0,
            );
            // let _ = cam.set(videoio::CAP_PROP_EXPOSURE, -10.0);
            let _ = cam.set(
                videoio::CAP_PROP_FOURCC,
                videoio::VideoWriter::fourcc('M', 'J', 'P', 'G')
                    .unwrap()
                    .into(),
            );
            let _ = cam.set(videoio::CAP_PROP_FPS, 60.0);
            // let _ = cam.set(videoio::CAP_OPENCV_MJPEG, 1.0);
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
            if let Ok(mut v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                // let _ = v.set(videoio::CAP_PROP_EXPOSURE, -10.0);
                let _ = v.set(videoio::CAP_PROP_FPS, 60.0);
                cam = v;
            }
            let _ = highgui::named_window(&format!("seancv{cam_index}"), 1);
            let _ = cam.set(3, RES.0);
            let _ = cam.set(
                videoio::CAP_PROP_XI_EXPOSURE,
                25000.0,
            );
            let _ = cam.set(
                videoio::CAP_PROP_FOURCC,
                videoio::VideoWriter::fourcc('M', 'J', 'P', 'G')
                    .unwrap()
                    .into(),
            );
            let _ = cam.set(4, RES.1);
            // let _ = cam.set(videoio::CAP_OPENCV_MJPEG, 1.0);
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
            if det.decision_margin() < 40.0 {
                continue;
            }
            let c = CameraMatrix::scaled(RES.0, RES.1);
            let pose = det.estimate_tag_pose(&TagParams {
                tagsize: 0.152,
                fx: c.fx,
                fy: c.fy,
                cx: c.cx,
                cy: c.cy,
            });

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
                println!("{}", matrix_to_heading(pose.rotation()).to_string());
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
        }
        println!("capture {:?}", (calc_time - frame_time).as_millis());
        println!("calc {:?}", calc_time.elapsed().as_millis());
        println!("total {:?}", frame_time.elapsed().as_millis());
        println!("fps {:?}", frame_num as f32 / start.elapsed().as_secs_f32());
        // *(DISPLAY_CACHE.lock()).push(frame.clone());
        highgui::imshow(&format!("seancv{cam_index}"), &frame)?;
        if highgui::wait_key(1)? > 0 {
            continue;
        }
    }
}

pub fn detect_loop_hybrid(cam_index: i32) -> Result<()> {
    let mut cam = videoio::VideoCapture::new(cam_index, videoio::CAP_ANY)?;
    cam.set(3, RES.0)?;
    cam.set(4, RES.1)?;
    let _ = cam.set(
        videoio::CAP_PROP_FOURCC,
        videoio::VideoWriter::fourcc('M', 'J', 'P', 'G')
            .unwrap()
            .into(),
    );
    let _ = cam.set(videoio::CAP_OPENCV_MJPEG, 1.0);
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
    let (tx, rx) = channel();
    thread::spawn(move || {
        let tx = tx.clone();
        loop {
            let frame_time = Instant::now();
            let mut frame = Mat::default();
            if cam.read(&mut frame).is_err() {
                thread::sleep(Duration::from_millis(10));
                if let Ok(v) = videoio::VideoCapture::new(0, videoio::CAP_ANY) {
                    cam = v;
                }
                let _ = cam.set(
                    videoio::CAP_PROP_FOURCC,
                    videoio::VideoWriter::fourcc('M', 'J', 'P', 'G')
                        .unwrap()
                        .into(),
                );
                let _ = cam.set(videoio::CAP_OPENCV_MJPEG, 1.0);
                let _ = highgui::named_window(&format!("seancv{cam_index}"), 1);
                let _ = cam.set(3, RES.0);
                let _ = cam.set(4, RES.1);
                let _ = videoio::VideoCapture::is_opened(&cam);
                continue;
            }
            // if one frame took a long time we can ignore
            if frame_num % 10 == 0 {
                start = Instant::now();
                frame_num = 0;
            }
            frame_num += 1;
            if frame.size().unwrap().width == 0 || frame.size().unwrap().height == 0 {
                thread::sleep(Duration::from_millis(1));
            }

            // TODO log here
            let _ = tx.send(frame);
            println!("capture {:?}", frame_time.elapsed().as_millis());
        }
    });
    while let Ok(frame) = rx.recv() {
        let Ok(frame_img) = frame.to_image() else {
            continue;
        };
        let calc_time = Instant::now();
        let detections = detector.detect(frame_img.to_luma8());
        frame_num += 1;

        if first {
            start = Instant::now();
            first = false;
        }
        for det in detections {
            if det.decision_margin() < 40.0 {
                continue;
            }
            let c = CameraMatrix::scaled(RES.0, RES.1);
            let pose = det.estimate_tag_pose(&TagParams {
                tagsize: 0.152,
                fx: c.fx,
                fy: c.fy,
                cx: c.cx,
                cy: c.cy,
            });
            // println!("  - pose {}: {:#?}", index, pose);
            if let Some(pose) = pose {
                println!("id {:?}", det.id());
                println!("translation {:?}", pose.translation());
                println!("{}", matrix_to_heading(pose.rotation()).to_string());
                let t = pose.translation().data();
                let x = t[0];
                let y = t[1];
                let z = t[2];
                println!(" x {}", x / 0.0254);
                println!(" y {}", y / 0.0254);
                println!(" z {}", z / 0.0254);
                println!(" mag {}", (x * x + z * z).sqrt() / 0.0254);
                // cam id
                compute_offset(0, det.id(), x, z, y);
            }
        }
        println!("calc {:?}", calc_time.elapsed().as_millis());
        println!("fps {:?}", frame_num as f32 / start.elapsed().as_secs_f32());
    }
    Ok(())
}
