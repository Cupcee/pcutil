use anyhow::{bail, Context, Result};
use indicatif::ProgressIterator;
use pasture_core::containers::{
    BorrowedBuffer, BorrowedBufferExt, BorrowedMutBuffer, OwningBuffer, VectorBuffer,
};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::Vector3;
use pasture_io::base::write_all;
use std::collections::HashSet;
use std::fs;
use std::path::Path;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, gather_pointcloud_paths, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::shared::progressbar::get_progress_bar;
use crate::PointcloudVoxelizeArgs;

pub fn execute(args: PointcloudVoxelizeArgs) -> Result<()> {
    let input_path = Path::new(&args.input);
    
    if input_path.is_dir() {
        voxelize_directory(args)
    } else {
        voxelize_single_file(args)
    }
}

pub fn voxelize_buffer(buffer: &VectorBuffer, voxel_size: f64) -> Result<VectorBuffer> {
    if voxel_size <= 0.0 {
        return Ok(buffer.clone());
    }

    let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
    
    let mut indices_to_keep = Vec::new();
    let mut seen_voxels = HashSet::new();

    for (i, p) in pos_view.into_iter().enumerate() {
        let key = [
            (p.x / voxel_size).floor() as i64,
            (p.y / voxel_size).floor() as i64,
            (p.z / voxel_size).floor() as i64,
        ];
        if seen_voxels.insert(key) {
            indices_to_keep.push(i);
        }
    }

    let mut new_buffer = VectorBuffer::with_capacity(indices_to_keep.len(), buffer.point_layout().clone());
    new_buffer.resize(indices_to_keep.len());

    let layout = buffer.point_layout();
    for attr in layout.attributes() {
        let attr_def = attr.attribute_definition();
        let attr_size = attr_def.size() as usize;
        let mut data = vec![0u8; attr_size];
        
        for (new_idx, &old_idx) in indices_to_keep.iter().enumerate() {
            buffer.get_attribute(attr_def, old_idx, &mut data);
            unsafe {
                new_buffer.set_attribute(attr_def, new_idx, &data);
            }
        }
    }

    Ok(new_buffer)
}

fn voxelize_single_file(args: PointcloudVoxelizeArgs) -> Result<()> {
    let input_ext = extension(&args.input);
    let output_ext = extension(&args.output);

    if !is_supported_extension(&input_ext) {
        bail!("Unsupported input format: {}", input_ext);
    }
    if !is_supported_extension(&output_ext) {
        bail!("Unsupported output format: {}", output_ext);
    }

    let buffer = read_pointcloud_file_to_buffer(&args.input, args.factor, args.dynamic_fields)?;
    let voxelized = voxelize_buffer(&buffer, args.voxel_size)?;

    write_to_file(&voxelized, &args.output)?;
    println!("Voxelized '{}' to '{}' ({} -> {} points)", 
        args.input, args.output, buffer.len(), voxelized.len());

    Ok(())
}

fn voxelize_directory(args: PointcloudVoxelizeArgs) -> Result<()> {
    let target_ext = if let Some(ref f) = args.format {
        f.clone()
    } else {
        let ext = extension(&args.output);
        if is_supported_extension(&ext) {
            ext
        } else {
            String::new()
        }
    };

    if target_ext.is_empty() {
        bail!("Target format must be specified via --format when voxelizing a directory, or output must have a supported extension.");
    }

    let paths = gather_pointcloud_paths(&args.input, args.recursive)?;
    if paths.is_empty() {
        println!("No supported pointcloud files found in '{}'", args.input);
        return Ok(());
    }

    let input_base_path = Path::new(&args.input);
    let output_base_dir = Path::new(&args.output);

    for path in paths
        .into_iter()
        .progress_with_style(get_progress_bar("Voxelizing files"))
    {
        let input_p = Path::new(&path);
        let relative_p = input_p.strip_prefix(input_base_path).unwrap_or(input_p);
        let file_stem = relative_p.file_stem().and_then(|s| s.to_str()).unwrap_or("output");
        let parent = relative_p.parent().unwrap_or(Path::new(""));
        
        let output_subdir = output_base_dir.join(parent);
        if !output_subdir.exists() {
            fs::create_dir_all(&output_subdir)?;
        }
        
        let output_filename = format!("{}.{}", file_stem, target_ext);
        let output_path = output_subdir.join(output_filename);
        let output_path_str = output_path.to_string_lossy().to_string();

        let buffer = read_pointcloud_file_to_buffer(&path, args.factor, args.dynamic_fields.clone())?;
        let voxelized = voxelize_buffer(&buffer, args.voxel_size)?;

        write_to_file(&voxelized, &output_path_str)?;
        // println!("Voxelized '{}' to '{}' ({} -> {} points)", 
        //     path, output_path_str, buffer.len(), voxelized.len());
    }

    Ok(())
}

fn write_to_file(buffer: &VectorBuffer, path: &str) -> Result<()> {
    let ext = extension(path);
    if ext == "las" || ext == "laz" {
        write_all(buffer, path).context("Failure while writing las/laz.")?;
    } else if ext == "ply" {
        crate::commands::pointcloud::pointcloud_utils::write_ply_file(buffer, path)
            .context("Failure while writing ply.")?;
    } else if ext == "pcd" {
        crate::commands::pointcloud::pointcloud_utils::write_pcd_file(buffer, path)
            .context("Failure while writing pcd.")?;
    } else {
        bail!("Unsupported output format: {}", ext);
    }
    Ok(())
}
