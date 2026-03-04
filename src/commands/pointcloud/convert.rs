use anyhow::{bail, Context, Result};
use pasture_io::base::write_all;
use std::path::Path;
use std::fs;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, gather_pointcloud_paths, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::PointcloudConvertArgs;

pub fn execute(args: PointcloudConvertArgs) -> Result<()> {
    let input_path = Path::new(&args.input);
    
    if input_path.is_dir() {
        convert_directory(args)
    } else {
        convert_single_file(args)
    }
}

fn convert_single_file(args: PointcloudConvertArgs) -> Result<()> {
    let input_ext = extension(&args.input);
    let output_ext = extension(&args.output);

    if !is_supported_extension(&input_ext) {
        bail!("Unsupported input format: {}", input_ext);
    }
    if !is_supported_extension(&output_ext) {
        bail!("Unsupported output format: {}", output_ext);
    }

    let buffer = read_pointcloud_file_to_buffer(&args.input, args.factor, args.dynamic_fields)?;

    // Supports writing LAS/LAZ, PLY or PCD output
    if output_ext == "las" || output_ext == "laz" {
        write_all(&buffer, &args.output).context("Failure while writing las/laz.")?;
        println!("Converted '{}' to '{}'", args.input, args.output);
    } else if output_ext == "ply" {
        crate::commands::pointcloud::pointcloud_utils::write_ply_file(&buffer, &args.output)
            .context("Failure while writing ply.")?;
        println!("Converted '{}' to '{}'", args.input, args.output);
    } else if output_ext == "pcd" {
        crate::commands::pointcloud::pointcloud_utils::write_pcd_file(&buffer, &args.output)
            .context("Failure while writing pcd.")?;
        println!("Converted '{}' to '{}'", args.input, args.output);
    } else {
        bail!("Currently only supports writing to LAS/LAZ, PLY or PCD output")
    }

    Ok(())
}

fn convert_directory(args: PointcloudConvertArgs) -> Result<()> {
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
        bail!("Target format must be specified via --format when converting a directory, or output must have a supported extension.");
    }

    if !is_supported_extension(&target_ext) {
        bail!("Unsupported target format: {}", target_ext);
    }

    let paths = gather_pointcloud_paths(&args.input, args.recursive)?;
    if paths.is_empty() {
        println!("No supported pointcloud files found in '{}'", args.input);
        return Ok(());
    }

    let input_base_path = Path::new(&args.input);
    let output_base_dir = Path::new(&args.output);

    for path in paths {
        let input_p = Path::new(&path);
        
        // Calculate relative path to preserve directory structure
        let relative_p = input_p.strip_prefix(input_base_path).unwrap_or(input_p);
        let file_stem = relative_p.file_stem().and_then(|s| s.to_str()).unwrap_or("output");
        let parent = relative_p.parent().unwrap_or(Path::new(""));
        
        let output_subdir = output_base_dir.join(parent);
        if !output_subdir.exists() {
            fs::create_dir_all(&output_subdir).context(format!("Failed to create output subdirectory '{:?}'", output_subdir))?;
        }
        
        let output_filename = format!("{}.{}", file_stem, target_ext);
        let output_path = output_subdir.join(output_filename);
        let output_path_str = output_path.to_string_lossy().to_string();

        let buffer = read_pointcloud_file_to_buffer(&path, args.factor, args.dynamic_fields.clone())?;

        if target_ext == "las" || target_ext == "laz" {
            write_all(&buffer, &output_path_str).context("Failure while writing las/laz.")?;
        } else if target_ext == "ply" {
            crate::commands::pointcloud::pointcloud_utils::write_ply_file(&buffer, &output_path_str)
                .context("Failure while writing ply.")?;
        } else if target_ext == "pcd" {
            crate::commands::pointcloud::pointcloud_utils::write_pcd_file(&buffer, &output_path_str)
                .context("Failure while writing pcd.")?;
        }
        println!("Converted '{}' to '{}'", path, output_path_str);
    }

    Ok(())
}
