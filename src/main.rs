use std::thread;
use opencv::highgui;
use opencv::core::Mat;

mod spline;
use spline::*;
// use::UnsizedVec;

use anyhow::Result;


mod lib;
use lib::*;

struct ScreenData {
    m: Mat,
    screen: i32,
}

// TODO
// async capture + sync
// multicamera preview
// multicamera calculation (linear transformation)
// spline calcution
// spline following + correction
// robot balancing (maybe just do in java?)
// robot avoidance?
// fancy intake stuff

fn main() -> Result<()> {
    // let _ = detect_loop(0);
    // let i = 0;
    //     let screen1 = highgui::named_window(&format!("seancv{i}"), i)?;
        // let _ = detect_loop_single(0);
    // loop {}
    // for i in 0..4 {
    //     let screen1 = highgui::named_window(&format!("seancv{i}"), i)?;
    //     thread::spawn(move || {
    //         let _ = detect_loop_single(i);
    //     });
    // }
    for i in 0..4 {
        let _ = detect_loop_hybrid(i);
    }
    loop {
        // let d = DISPLAY_CACHE.lock().unwrap();
        // if let Some(v) = d {
        //     highgui::imshow(&format!("seancv{}", m.screen), &v.m).unwrap();
        // }

    }
    Ok(())
}
