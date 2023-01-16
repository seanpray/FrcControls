use std::thread;
use opencv::highgui;
use std::sync::{Arc, Mutex};
use opencv::core::Mat;
// use::UnsizedVec;

use anyhow::Result;

mod lib;
use lib::*;

struct ScreenData {
    m: Mat,
    screen: i32,
}

// lazy_static::lazy_static! {
//     pub (crate) static ref DISPLAY_CACHE: Arc<Mutex<Vec<ScreenData>>> = Arc::new(Mutex::new(vec![]));
// }

fn main() -> Result<()> {
    // let _ = detect_loop(0);
    let i = 0;
        let screen1 = highgui::named_window(&format!("seancv{i}"), i)?;
        let _ = detect_loop(i);
    loop {}
    for i in 0..4 {
        let screen1 = highgui::named_window(&format!("seancv{i}"), i)?;
        thread::spawn(move || {
            let _ = detect_loop(i);
        });
    }
    loop {
        // let d = DISPLAY_CACHE.lock().unwrap();
        // if let Some(v) = d {
        //     highgui::imshow(&format!("seancv{}", m.screen), &v.m).unwrap();
        // }

    }
    Ok(())
}
