use clap::Args;

pub mod commands;
pub mod shared;

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum DynFieldType {
    #[value(alias("cls"), hide = false)]
    #[value(alias("class"), hide = false)]
    #[value(alias("label"), hide = false)]
    Classification,
    Skip,
}

#[derive(Debug, Args)]
pub struct PointcloudSummaryArgs {
    /// Supported pointcloud formats: [LAS, LAZ, PCD]
    #[arg(required = true, value_delimiter = ' ', num_args = 1..)]
    pub input: Vec<String>,

    /// These should be passed in same order as the fields
    /// have in the input file, and we expect that first dynamic
    /// field comes after fields XYZ (4th field in data).
    /// E.g. if we have PCD file with fields x, y, z, class
    /// you may pass `-d label`.
    ///
    /// - `classification`: Classification field on the input data.
    /// Allows extracting additional information related to classes.
    /// If the input data for this field is not U8, reading the
    /// file fails (but program does not panic).
    /// Has aliases cls, class, label, classification.
    ///
    /// - `skip`: Skips reading the dynamic field at its position. Can be used to read columns that are
    /// unsupported by the tool.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// If provided, recursively process directories
    #[clap(short, long)]
    pub recursive: bool,
}

#[derive(Debug, Args)]
pub struct PointcloudVisualizationArgs {
    #[arg(required = true)]
    pub input: String,
    #[clap(short, long, default_value_t = 1.0)]
    pub factor: f64,
    #[clap(short, long)]
    pub dynamic_fields: Vec<DynFieldType>,
}

#[derive(Debug, Args)]
pub struct PointcloudConvertArgs {
    /// Supported pointcloud formats: [PCD]
    #[arg(required = true)]
    pub input: String,
    /// Supported pointcloud formats: [LAS, LAZ]
    #[arg(required = true)]
    pub output: String,

    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// These should be passed in same order as the fields
    /// have in the input file, and we expect that first dynamic
    /// field comes after fields XYZ (4th field in data).
    /// E.g. if we have PCD file with fields x, y, z, class
    /// you may pass `-d class`.
    ///
    /// - `classification`: Classification field on the input data.
    /// Allows extracting additional information related to classes.
    /// If the input data for this field is not U8, reading the
    /// file fails (but program does not panic).
    /// Has aliases cls, class, label, classification.
    ///
    /// - `skip`: Skips reading the dynamic field at its position. Useful for unsupported fields.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,
}

// Error handling utility that can be used by both lib and binary
pub fn handle_error(e: anyhow::Error) {
    eprintln!("Error!");
    for (i, cause) in e.chain().enumerate() {
        eprintln!("  Cause {}: {}", i, cause);
    }
    std::process::exit(1);
}
