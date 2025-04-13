use clap::Args;

pub mod commands;

#[derive(Debug, Args)]
pub struct PointcloudSummaryArgs {
    /// Input file or directory
    ///
    /// Supported pointcloud formats: [LAS, LAZ, PCD]
    #[arg(required = true)]
    pub input: String,

    /// If provided, recursively process directories
    #[clap(short, long)]
    pub recursive: bool,

    /// Use strict schema for PCD files. If provided, the schema is expected to be precisely x, y, z,
    /// rgb where all are F32. Otherwise, we only try dynamically parsing xyz information.
    #[clap(long)]
    pub strict_pcd_schema: bool,
}

#[derive(Debug, Args)]
pub struct PointcloudConvertArgs {
    /// Input pointcloud file
    ///
    /// Supported pointcloud formats: [PCD]
    #[arg(required = true)]
    pub input: String,
    /// Output pointcloud file
    ///
    /// Supported pointcloud formats: [LAS, LAZ]
    #[arg(required = true)]
    pub output: String,

    /// Strict PCD schema. If provided, the schema is expected to be precisely x, y, z,
    /// rgb where all are F32. Otherwise, we only try dynamically parsing xyz information.
    #[clap(long)]
    pub strict_pcd_schema: bool,
}

// Error handling utility that can be used by both lib and binary
pub fn handle_error(e: anyhow::Error) {
    eprintln!("Error!");
    for (i, cause) in e.chain().enumerate() {
        eprintln!("  Cause {}: {}", i, cause);
    }
    std::process::exit(1);
}
