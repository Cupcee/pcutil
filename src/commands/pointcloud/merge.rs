use anyhow::{bail, Context, Result};
use indicatif::{ParallelProgressIterator, ProgressIterator};
use npyz::NpyFile;
use pasture_core::containers::{
    BorrowedBuffer, BorrowedBufferExt, BorrowedMutBuffer, OwningBuffer, VectorBuffer,
};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::{Matrix4, Vector3, Vector4};
use pasture_io::base::write_all;
use rayon::prelude::*;
use std::fs::File;
use std::io::BufReader;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, gather_pointcloud_paths, read_pointcloud_file_to_buffer,
};
use crate::commands::pointcloud::voxelize::voxelize_buffer;
use crate::shared::progressbar::get_progress_bar;
use crate::PointcloudMergeArgs;

pub fn execute(args: PointcloudMergeArgs) -> Result<()> {
    let mut paths = gather_pointcloud_paths(&args.input, args.recursive)?;
    paths.sort();

    if paths.is_empty() {
        bail!("No pointcloud files found in directory: {}", args.input);
    }

    let poses = if let Some(ref poses_path) = args.poses {
        Some(load_poses(poses_path)?)
    } else {
        None
    };

    if let Some(ref p) = poses {
        if p.len() < paths.len() {
            bail!(
                "Number of poses ({}) is less than the number of frames ({}).",
                p.len(),
                paths.len()
            );
        }
    }

    println!("Merging {} files...", paths.len());

    // 1. Read files in parallel with a progress bar
    let results: Vec<Result<VectorBuffer>> = paths
        .par_iter()
        .progress_with_style(get_progress_bar("Reading pointclouds"))
        .map(|path| read_pointcloud_file_to_buffer(path, args.factor, args.dynamic_fields.clone()))
        .collect();

    let mut buffers = Vec::with_capacity(paths.len());
    let mut total_points = 0;
    for res in results {
        let buffer = res?;
        total_points += buffer.len();
        buffers.push(buffer);
    }

    if buffers.is_empty() {
        bail!("No points to merge.");
    }

    // 2. Prepare the super_buffer
    let layout = buffers[0].point_layout().clone();
    let mut super_buffer = VectorBuffer::with_capacity(total_points, layout.clone());
    super_buffer.resize(total_points);

    // 3. Merge buffers sequentially with a progress bar
    let mut current_offset = 0;
    for (i, buffer) in buffers
        .into_iter()
        .enumerate()
        .progress_with_style(get_progress_bar("Merging into supercloud"))
    {
        let num_points = buffer.len();
        let pose = poses.as_ref().map(|p| p[i]);

        // Copy all attributes that exist in both layouts
        for attr in layout.attributes() {
            if buffer
                .point_layout()
                .has_attribute(attr.attribute_definition())
            {
                if *attr.attribute_definition() == attributes::POSITION_3D && pose.is_some() {
                    let transform = pose.unwrap();
                    let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);

                    for (j, pos) in pos_view.into_iter().enumerate() {
                        let p_homogeneous = Vector4::new(pos.x, pos.y, pos.z, 1.0);
                        let p_transformed = transform * p_homogeneous;
                        let new_pos =
                            Vector3::new(p_transformed.x, p_transformed.y, p_transformed.z);

                        unsafe {
                            super_buffer.set_attribute(
                                &attributes::POSITION_3D,
                                current_offset + j,
                                bytemuck::cast_slice(&[new_pos.x, new_pos.y, new_pos.z]),
                            );
                        }
                    }
                } else {
                    // Generic copy for other attributes
                    let attr_size = attr.attribute_definition().size() as usize;
                    let mut data = vec![0u8; attr_size];
                    for j in 0..num_points {
                        buffer.get_attribute(attr.attribute_definition(), j, &mut data);
                        unsafe {
                            super_buffer.set_attribute(
                                attr.attribute_definition(),
                                current_offset + j,
                                &data,
                            );
                        }
                    }
                }
            }
        }
        current_offset += num_points;
    }

    let final_buffer = if let Some(voxel_size) = args.voxel_size {
        println!("Voxelizing supercloud with size {}...", voxel_size);
        voxelize_buffer(&super_buffer, voxel_size)?
    } else {
        super_buffer
    };

    let output_ext = extension(&args.output);
    if output_ext == "las" || output_ext == "laz" {
        write_all(&final_buffer, &args.output).context("Failure while writing las/laz.")?;
    } else if output_ext == "ply" {
        crate::commands::pointcloud::pointcloud_utils::write_ply_file(&final_buffer, &args.output)
            .context("Failure while writing ply.")?;
    } else if output_ext == "pcd" {
        crate::commands::pointcloud::pointcloud_utils::write_pcd_file(&final_buffer, &args.output)
            .context("Failure while writing pcd.")?;
    } else {
        bail!(
            "Unsupported output format: {}. Use las, laz, ply, or pcd.",
            output_ext
        );
    }

    println!(
        "Successfully merged into '{}' (total points: {})",
        args.output,
        final_buffer.len()
    );
    Ok(())
}

fn load_poses(path: &str) -> Result<Vec<Matrix4<f64>>> {
    let reader = BufReader::new(File::open(path).context("Failed to open poses file")?);
    let npy = NpyFile::new(reader).context("Failed to read npy file")?;
    
    // We expect f64 values. The array should be (N, 4, 4)
    // npyz reads them in a flat iterator.
    let data: Vec<f64> = npy.data::<f64>().context("Failed to read npy data as f64")?.collect::<Result<Vec<_>, _>>()?;
    
    if data.len() % 16 != 0 {
        bail!("Npy data length is not a multiple of 16 (each pose must be 4x4). Found {} elements.", data.len());
    }
    
    let num_poses = data.len() / 16;
    let mut matrices = Vec::with_capacity(num_poses);
    
    for i in 0..num_poses {
        let chunk = &data[i * 16..(i + 1) * 16];
        // numpy arrays are usually row-major. nalgebra Matrix4::from_row_slice
        let mat = Matrix4::from_row_slice(chunk);
        matrices.push(mat);
    }
    
    Ok(matrices)
}
