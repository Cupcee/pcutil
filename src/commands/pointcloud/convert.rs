use anyhow::{bail, Context, Result};

use pasture_io::base::write_all;

use crate::commands::pointcloud::pointcloud_utils::{
    extension, is_supported_extension, read_pointcloud_file_to_buffer,
};
use crate::PointcloudConvertArgs;

pub fn execute(args: PointcloudConvertArgs) -> Result<()> {
    let input_ext = extension(&args.input);
    let output_ext = extension(&args.output);

    if !is_supported_extension(&input_ext) {
        bail!("Unsupported input format: {}", input_ext);
    }
    if !is_supported_extension(&output_ext) {
        bail!("Unsupported output format: {}", output_ext);
    }

    let buffer = read_pointcloud_file_to_buffer(&args.input, args.strict_pcd_schema)?;

    // Currently only supports writing LAS/LAZ output
    if output_ext == "las" || output_ext == "laz" {
        write_all(&buffer, &args.output).context("Failure while writing las/laz.")?;
        println!("Converted '{}' to '{}'", args.input, args.output);
    } else {
        bail!("Currently only supports writing to LAS/LAZ output")
    }

    Ok(())
}
