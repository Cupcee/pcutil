use anyhow::{bail, Context, Result};
use bytemuck;
use pasture_core::{
    containers::BorrowedBuffer, containers::BorrowedBufferExt, containers::BorrowedMutBuffer,
    containers::OwningBuffer, containers::VectorBuffer,
};

use pasture_core::layout::{
    attributes::{COLOR_RGB, POSITION_3D},
    PointLayout,
};

use pasture_io::base::read_all;
use pcd_rs::DynReader;
use pcd_rs::PcdDeserialize;
use pcd_rs::Reader as PcdReader;
use std::path::Path;

/// PCD point struct for reading from .pcd files
#[derive(PcdDeserialize, Debug)]
pub struct PcdPoint {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub rgb: f32,
}

/// Decode a float `rgb` value from PCD into (r, g, b) as (u8, u8, u8).
pub fn decode_rgb(rgb: f32) -> (u8, u8, u8) {
    let rgb_u32 = rgb.to_bits();
    let r = ((rgb_u32 >> 16) & 0xFF) as u8;
    let g = ((rgb_u32 >> 8) & 0xFF) as u8;
    let b = (rgb_u32 & 0xFF) as u8;
    (r, g, b)
}

/// Determine file extension in lowercase
pub fn extension(path: &str) -> String {
    Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase()
}

pub fn is_supported_extension(ext: &str) -> bool {
    matches!(ext, "las" | "laz" | "pcd")
}

/// Read a single pointcloud file (.las/.laz or .pcd) into a VectorBuffer
pub fn read_pointcloud_file_to_buffer(path: &str, strict_pcd_schema: bool) -> Result<VectorBuffer> {
    let ext = Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("");
    match ext.to_lowercase().as_str() {
        "las" | "laz" => {
            let buffer = read_all::<VectorBuffer, _>(path)?;
            Ok(buffer)
        }
        "pcd" => {
            if strict_pcd_schema {
                read_pcd_file(path)
            } else {
                read_dyn_pcd_file(path)
            }
        }
        _ => bail!("Unsupported format: {}", path),
    }
}

/// Read a .pcd file without schema into a VectorBuffer with POSITION_3D
/// Bails if unable to read the file.
pub fn read_dyn_pcd_file(path: &str) -> Result<VectorBuffer> {
    let reader = DynReader::open(path)?;
    let points = reader.collect::<Result<Vec<_>, _>>()?;
    if points.is_empty() {
        bail!("No points found in the PCD file {}", path);
    }

    let mut layout = PointLayout::default();
    layout.add_attribute(POSITION_3D, pasture_core::layout::FieldAlignment::Default);

    let num_points = points.len();
    let mut buffer = VectorBuffer::with_capacity(num_points, layout);
    buffer.resize(num_points); // Allocate space for all points

    let pos_attr = buffer
        .point_layout()
        .get_attribute(&POSITION_3D)
        .unwrap()
        .attribute_definition()
        .clone();

    for (i, p) in points.into_iter().enumerate() {
        let xyz: [f32; 3] = p.to_xyz().context("Unable to find xyz in PCD schema.")?;
        let pos_data = [xyz[0] as f64, xyz[1] as f64, xyz[2] as f64];

        unsafe {
            buffer.set_attribute(&pos_attr, i, bytemuck::cast_slice(&pos_data));
        }
    }

    Ok(buffer)
}

/// Read a .pcd file into a VectorBuffer with POSITION_3D and COLOR_RGB
/// Panics if unable to read the file.
pub fn read_pcd_file(path: &str) -> Result<VectorBuffer> {
    let reader = PcdReader::open(path)?;
    let points: Vec<PcdPoint> = reader.collect::<Result<Vec<_>, _>>()?;
    if points.is_empty() {
        bail!("No points found in the PCD file {}", path);
    }

    let mut layout = PointLayout::default();
    layout.add_attribute(POSITION_3D, pasture_core::layout::FieldAlignment::Default);
    layout.add_attribute(COLOR_RGB, pasture_core::layout::FieldAlignment::Default);

    let num_points = points.len();
    let mut buffer = VectorBuffer::with_capacity(num_points, layout);
    buffer.resize(num_points); // Allocate space for all points

    let pos_attr = buffer
        .point_layout()
        .get_attribute(&POSITION_3D)
        .unwrap()
        .attribute_definition()
        .clone();
    let color_attr = buffer
        .point_layout()
        .get_attribute(&COLOR_RGB)
        .unwrap()
        .attribute_definition()
        .clone();

    for (i, p) in points.into_iter().enumerate() {
        let (r, g, b) = decode_rgb(p.rgb);
        let pos_data = [p.x as f64, p.y as f64, p.z as f64];
        let color_data = [r as u16, g as u16, b as u16];

        unsafe {
            buffer.set_attribute(&pos_attr, i, bytemuck::cast_slice(&pos_data));
            buffer.set_attribute(&color_attr, i, bytemuck::cast_slice(&color_data));
        }
    }

    Ok(buffer)
}

/// Compute bounding box for a buffer
pub fn compute_bounds(buffer: &VectorBuffer) -> (f64, f64, f64, f64, f64, f64) {
    use pasture_core::{layout::attributes::POSITION_3D, nalgebra::Vector3};

    let mut min_x = f64::MAX;
    let mut max_x = f64::MIN;
    let mut min_y = f64::MAX;
    let mut max_y = f64::MIN;
    let mut min_z = f64::MAX;
    let mut max_z = f64::MIN;

    if buffer.point_layout().has_attribute(&POSITION_3D) {
        let view = buffer.view_attribute::<Vector3<f64>>(&POSITION_3D);
        for p in view.into_iter() {
            if p.x < min_x {
                min_x = p.x;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.y > max_y {
                max_y = p.y;
            }
            if p.z < min_z {
                min_z = p.z;
            }
            if p.z > max_z {
                max_z = p.z;
            }
        }
    }

    (min_x, max_x, min_y, max_y, min_z, max_z)
}
