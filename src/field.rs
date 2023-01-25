use std::path::Path;
use serde::Deserialize;
use std::fs::read_to_string;
use std::error::Error;

use crate::FIELD_DATA;

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct Quaternion {
    W: f64,
    X: f64,
    Y: f64,
    Z: f64,
}

#[derive(Deserialize)]
struct Rotation {
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
pub (crate) struct Field {
    tags: Vec<Tag>,
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
            },
            _ => false
        }
    }
}
