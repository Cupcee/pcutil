use anyhow::{Context, Result};
use npyz::WriterBuilder;
use pasture_core::containers::{BorrowedMutBuffer, OwningBuffer, VectorBuffer};
use pasture_core::layout::attributes;
use pcutil::commands::pointcloud::pointcloud_utils::write_pcd_file;
use std::fs::File;
use std::io::BufWriter;

pub fn create_mock_pcd(path: &str, points: &[[f64; 3]], intensities: Option<&[u16]>) -> Result<()> {
    let mut layout = pasture_core::layout::PointLayout::default();
    layout.add_attribute(
        attributes::POSITION_3D,
        pasture_core::layout::FieldAlignment::Default,
    );
    if intensities.is_some() {
        layout.add_attribute(
            attributes::INTENSITY,
            pasture_core::layout::FieldAlignment::Default,
        );
    }

    let mut buffer = VectorBuffer::with_capacity(points.len(), layout);
    buffer.resize(points.len());

    for (i, p) in points.iter().enumerate() {
        unsafe {
            buffer.set_attribute(&attributes::POSITION_3D, i, bytemuck::cast_slice(p));
            if let Some(ints) = intensities {
                buffer.set_attribute(&attributes::INTENSITY, i, bytemuck::cast_slice(&[ints[i]]));
            }
        }
    }

    write_pcd_file(&buffer, path)?;
    Ok(())
}

pub fn create_mock_poses(path: &str, num_poses: usize) -> Result<()> {
    let file = File::create(path).context("Failed to create poses file")?;
    let mut writer = npyz::WriteOptions::new()
        .default_dtype()
        .shape(&[num_poses as u64, 4, 4])
        .writer(BufWriter::new(file))
        .begin_nd()?;

    let identity: [f64; 16] = [
        1.0, 0.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 0.0, 
        0.0, 0.0, 1.0, 0.0, 
        0.0, 0.0, 0.0, 1.0,
    ];

    for _ in 0..num_poses {
        for &val in &identity {
            writer.push(&val)?;
        }
    }

    writer.finish()?;
    Ok(())
}
