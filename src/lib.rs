use anyhow::Result;
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

struct Rotation {
    heading: f32,
    roll: f32,
    pitch: f32,
}

fn matrix_to_heading() -> Rotation {
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
