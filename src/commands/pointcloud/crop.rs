use anyhow::{bail, Context, Result};
use indicatif::ProgressIterator;
use pasture_core::containers::{
    BorrowedBuffer, BorrowedBufferExt, BorrowedMutBuffer, OwningBuffer, VectorBuffer,
};
use pasture_core::layout::attributes;
use pasture_core::nalgebra::Vector3;
use pasture_io::base::write_all;
use std::fs;
use std::path::Path;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, gather_pointcloud_paths, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::shared::progressbar::get_progress_bar;
use crate::PointcloudCropArgs;

pub fn execute(args: PointcloudCropArgs) -> Result<()> {
    let input_path = Path::new(&args.input);

    if input_path.is_dir() {
        crop_directory(args)
    } else {
        crop_single_file(args)
    }
}

pub fn crop_buffer(buffer: &VectorBuffer, bounds: &[f64]) -> Result<VectorBuffer> {
    if bounds.len() != 6 {
        bail!("AABB bounds must have exactly 6 values: x_min y_min z_min x_max y_max z_max");
    }

    let x_min = bounds[0];
    let y_min = bounds[1];
    let z_min = bounds[2];
    let x_max = bounds[3];
    let y_max = bounds[4];
    let z_max = bounds[5];

    let pos_view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);

    let mut indices_to_keep = Vec::new();
    for (i, p) in pos_view.into_iter().enumerate() {
        if p.x >= x_min && p.x <= x_max && p.y >= y_min && p.y <= y_max && p.z >= z_min && p.z <= z_max
        {
            indices_to_keep.push(i);
        }
    }

    let mut new_buffer =
        VectorBuffer::with_capacity(indices_to_keep.len(), buffer.point_layout().clone());
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

fn crop_single_file(args: PointcloudCropArgs) -> Result<()> {
    let input_ext = extension(&args.input);
    let output_ext = extension(&args.output);

    if !is_supported_extension(&input_ext) {
        bail!("Unsupported input format: {}", input_ext);
    }
    if !is_supported_extension(&output_ext) {
        bail!("Unsupported output format: {}", output_ext);
    }

    let buffer = read_pointcloud_file_to_buffer(&args.input, args.factor, args.dynamic_fields)?;
    let cropped = crop_buffer(&buffer, &args.bounds)?;

    write_to_file(&cropped, &args.output)?;
    println!(
        "Cropped '{}' to '{}' ({} -> {} points)",
        args.input,
        args.output,
        buffer.len(),
        cropped.len()
    );

    Ok(())
}

fn crop_directory(args: PointcloudCropArgs) -> Result<()> {
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
        bail!("Target format must be specified via --format when cropping a directory, or output must have a supported extension.");
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
        .progress_with_style(get_progress_bar("Cropping files"))
    {
        let input_p = Path::new(&path);
        let relative_p = input_p.strip_prefix(input_base_path).unwrap_or(input_p);
        let file_stem = relative_p
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("output");
        let parent = relative_p.parent().unwrap_or(Path::new(""));

        let output_subdir = output_base_dir.join(parent);
        if !output_subdir.exists() {
            fs::create_dir_all(&output_subdir)?;
        }

        let output_filename = format!("{}.{}", file_stem, target_ext);
        let output_path = output_subdir.join(output_filename);
        let output_path_str = output_path.to_string_lossy().to_string();

        let buffer =
            read_pointcloud_file_to_buffer(&path, args.factor, args.dynamic_fields.clone())?;
        let cropped = crop_buffer(&buffer, &args.bounds)?;

        write_to_file(&cropped, &output_path_str)?;
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
