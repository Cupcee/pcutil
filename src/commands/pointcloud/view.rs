use anyhow::{bail, Context, Result};
use raylib::prelude::*;
use std::path::PathBuf;

use crate::commands::pointcloud::pointcloud_utils::{
    is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::PointcloudViewArgs;

const WINDOW_WIDTH: i32 = 640;
const WINDOW_HEIGHT: i32 = 480;

/// Main entry point
pub fn execute(args: PointcloudViewArgs) -> Result<()> {
    let path = args.input;
    test_ext(&path)?;
    let path_str = path.to_string_lossy().to_string();
    let pc = read_pointcloud_file_to_buffer(&path_str, args.factor, args.dynamic_fields).context(
        format!("Was unable to read pointcloud in path {}", &path_str),
    )?;
    let (mut rl, thread) = raylib::init()
        .size(WINDOW_WIDTH, WINDOW_HEIGHT)
        .title("pcutil view")
        .build();
    let camera = Camera3D::perspective(
        Vector3::new(4.0, 2.0, 4.0),
        Vector3::new(0.0, 1.8, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        60.0,
    );
    rl.set_target_fps(60);
    while !rl.window_should_close() {
        rl.draw(&thread, |mut d| {
            d.clear_background(Color::WHITE);
            d.draw_mode3D(camera, |mut d3, _camera| {
                d3.draw_cube(Vector3::new(-16.0, 2.5, 0.0), 1.0, 5.0, 32.0, Color::BLUE);
            });
        });
    }
    Ok(())
}

fn test_ext(path: &PathBuf) -> Result<()> {
    match path.extension() {
        Some(ext) => {
            let ext = ext.to_string_lossy().to_string();
            if !is_supported_extension(&ext) {
                bail!("Unsupported format {}", &ext);
            }
            return Ok(());
        }
        None => bail!("Path {} has no extension!", path.to_string_lossy()),
    }
}
