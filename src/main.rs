use clap::{Parser, Subcommand};
use pcutil::{
    PointcloudConvertArgs, PointcloudCropArgs, PointcloudMergeArgs, PointcloudSummaryArgs,
    PointcloudVisualizationArgs, PointcloudVoxelizeArgs,
};

/// Rust implementation of bash commands
#[derive(Debug, Parser)]
#[clap(name = "pcutil", version, author, about, infer_subcommands = true)]
pub struct RootArgs {
    #[clap(subcommand)]
    command: PointcloudCommand,
}

#[derive(Debug, Subcommand)]
enum PointcloudCommand {
    /// Summarize files in a given path.
    Summary(PointcloudSummaryArgs),
    /// Convert pointcloud file from one format to another.
    Convert(PointcloudConvertArgs),
    /// Visualize point cloud using Rerun.
    Visualize(PointcloudVisualizationArgs),
    /// Merge multiple pointcloud files into one supercloud.
    Merge(PointcloudMergeArgs),
    /// Voxelize (downsample) pointcloud file(s).
    Voxelize(PointcloudVoxelizeArgs),
    /// Crop pointcloud to specified AABB bounds.
    Crop(PointcloudCropArgs),
}

fn main() {
    let args = RootArgs::parse();

    let result = match args.command {
        PointcloudCommand::Summary(args) => pcutil::commands::pointcloud::summary::execute(args),
        PointcloudCommand::Convert(args) => pcutil::commands::pointcloud::convert::execute(args),
        PointcloudCommand::Visualize(args) => {
            pcutil::commands::pointcloud::visualize::execute(args)
        }
        PointcloudCommand::Merge(args) => pcutil::commands::pointcloud::merge::execute(args),
        PointcloudCommand::Voxelize(args) => pcutil::commands::pointcloud::voxelize::execute(args),
        PointcloudCommand::Crop(args) => pcutil::commands::pointcloud::crop::execute(args),
    };

    if let Err(e) = result {
        pcutil::handle_error(e);
    }
}
