use std::{thread, sync::{RwLock, atomic::AtomicBool}};
use opencv::highgui;
use std::sync::{Arc, Mutex};
use opencv::core::Mat;

mod spline;
use spline::*;
// use::UnsizedVec;

use anyhow::Result;

mod field;
use field::*;

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
enum OP {
    ReloadField
}

const FIELD_DATA_NAME: &str = "2023-chargedup.json";

lazy_static::lazy_static! {
//     pub (crate) static ref DISPLAY_CACHE: Arc<Mutex<Vec<ScreenData>>> = Arc::new(Mutex::new(vec![]));
    pub (crate) static ref FIELD_DATA: Arc<RwLock<Field>> = Arc::new(RwLock::new(Field::from_file("2023-chargedup.json").unwrap()));
    pub (crate) static ref RUNTIME_CACHE: Arc<Mutex<Vec<OP>>> = Arc::new(Mutex::new(vec![]));
    pub (crate) static ref INSTRUCTION_UPDATE: AtomicBool = AtomicBool::from(false);
}

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
