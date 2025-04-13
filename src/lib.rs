use clap::Args;

pub mod commands;

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum DynFieldType {
    Classification,
    Intensity,
}

#[derive(Debug, Args)]
pub struct PointcloudSummaryArgs {
    /// Input file or directory
    ///
    /// Supported pointcloud formats: [LAS, LAZ, PCD]
    #[arg(required = true)]
    pub input: String,

    #[clap(long, value_enum, num_args = 0..)]
    pub pcd_dyn_fields: Vec<DynFieldType>,

    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// If provided, recursively process directories
    #[clap(short, long)]
    pub recursive: bool,
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

    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    #[clap(long, value_enum, num_args = 0..)]
    pub pcd_dyn_fields: Vec<DynFieldType>,
}

// Error handling utility that can be used by both lib and binary
pub fn handle_error(e: anyhow::Error) {
    eprintln!("Error!");
    for (i, cause) in e.chain().enumerate() {
        eprintln!("  Cause {}: {}", i, cause);
    }
    std::process::exit(1);
}
