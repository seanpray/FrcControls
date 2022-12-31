use anyhow::Result;



mod simple_detector {
    use apriltag::image_buf;
    use image::DynamicImage;
    use image::ImageBuffer;
    use mat2image::ToImage;
    use opencv::prelude::MatTraitConst;
    use opencv::prelude::VideoCaptureTrait;
    use opencv::prelude::VideoCaptureTraitConst;
    use opencv::prelude::MatTraitConstManual;
    use std::{thread, time::Duration};
    use opencv::{
        imgproc,
        highgui,
        videoio,
        core::Rect,
        core::Scalar,
    };
    use opencv::core::Mat;
    use anyhow::{ensure, Context, Error, Result};
    use apriltag::{DetectorBuilder, Family, TagParams};
    use std::{path::PathBuf, str::FromStr};
    use structopt::StructOpt;

    #[derive(Debug, Clone, StructOpt)]
    /// Simple AprilTag detector.
    struct Opts {
        #[structopt(long, default_value = "tag16h5")]
        /// family name.
        pub family: String,
        #[structopt(long)]
        /// optional tag parameters in format "tagsize,fx,fy,cx,cy".
        pub tag_params: Option<TagParamsArg>,
        /// input files.
        pub input_files: Vec<PathBuf>,
    }

    #[derive(Debug, Clone)]
    struct TagParamsArg {
        pub tagsize: f64,
        pub fx: f64,
        pub fy: f64,
        pub cx: f64,
        pub cy: f64,
    }

    impl From<TagParamsArg> for TagParams {
        fn from(arg: TagParamsArg) -> Self {
            let TagParamsArg {
                tagsize,
                fx,
                fy,
                cx,
                cy,
            } = arg;

            Self {
                tagsize,
                fx,
                fy,
                cx,
                cy,
            }
        }
    }

    impl FromStr for TagParamsArg {
        type Err = Error;

        fn from_str(text: &str) -> Result<Self, Self::Err> {
            let tokens: Vec<_> = text.split(',').collect();
            ensure!(
                tokens.len() == 5,
                r#"tag parameters must be in format "tagsize,fx,fy,cx,cy""#
            );

            let values = tokens
                .into_iter()
                .map(|token| -> Result<_> {
                    let value: f64 = token.parse().unwrap();
                    Ok(value)
                })
                .collect::<Result<Vec<_>>>()
                .with_context(|| format!("failed to parse tag parameters {}", text))?;

            Ok(Self {
                tagsize: values[0],
                fx: values[1],
                fy: values[2],
                cx: values[3],
                cy: values[4],
            })
        }
    }

    pub fn _main() -> Result<()> {
        let Opts {
            family: family_name,
            tag_params,
            input_files,
        } = Opts::from_args();
        let mut cam = videoio::VideoCapture::new(0, videoio::CAP_ANY)?;

        highgui::named_window("seancv", 1)?;
        cam.set(3, 1920.0)?;
        cam.set(4, 1080.0)?;
        let _ = videoio::VideoCapture::is_opened(&cam)?;
        let family: Family = Family::tag_16h5();
        let tag_params: Option<TagParams> = tag_params.map(|params| params.into());
            let mut detector = DetectorBuilder::new()
                .add_family_bits(family, 1)
                .build()
                .unwrap();

        loop {
            let mut frame = Mat::default();
            cam.read(&mut frame)?;
            if frame.size()?.width == 0 || frame.size()?.height == 0 {
                thread::sleep(Duration::from_millis(50));
            }


            // if input_files.is_empty() {
            //     eprintln!("no input files");
            //     return Ok(());
            // }

            let frame_img = frame.to_image().unwrap();
            // let DynamicImage::ImageLuma8(frame) = frame else {
            //     unreachable!();
            // };
            let detections = detector.detect(frame_img.to_luma8());
            // let detections = detector.detect(frame.to_image().unwrap().as_flat_samples_u8().unwrap());

            detections.into_iter().enumerate().for_each(|(index, det)| {
                println!("  - detection {}: {:#?}", index, det);
                if let Some(tag_params) = &tag_params {
                    let pose = det.estimate_tag_pose(tag_params);
                    println!("  - pose {}: {:#?}", index, pose);
                }
                let corners = det.corners();
                let center = det.center();
                let (w, h) = ((corners[2][0] - corners[3][0]) as i32, (corners[0][1] - corners[3][1]) as i32);
                let r#box = Rect {
                    x: center[0] as i32 - w / 2,
                    y: center[1] as i32 - h / 2,
                    width: w,
                    height: h,
                };
                imgproc::rectangle(
                    &mut frame,
                    r#box,
                    Scalar::new(255f64, 0f64, 0f64, 25f64), // color value
                    1, // border width of drawn rectangle
                    8,
                    0,
                ).unwrap();
            });
            highgui::imshow("seancv", &frame)?;
            if highgui::wait_key(1)? > 0 {
                continue;
            }
        }
        Ok(())
    }
}

fn main() -> Result<()> {
    simple_detector::_main()
}
