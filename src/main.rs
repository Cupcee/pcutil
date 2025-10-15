use clap::{Parser, Subcommand};
use pcutil::{PointcloudConvertArgs, PointcloudSummaryArgs};

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
}

fn main() {
    let args = RootArgs::parse();

    let result = match args.command {
        PointcloudCommand::Summary(args) => pcutil::commands::pointcloud::summary::execute(args),
        PointcloudCommand::Convert(args) => pcutil::commands::pointcloud::convert::execute(args),
    };

    if let Err(e) = result {
        pcutil::handle_error(e);
    }
}
