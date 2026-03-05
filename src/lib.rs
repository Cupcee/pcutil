use clap::Args;

pub mod commands;
pub mod shared;

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum DynFieldType {
    #[value(alias("c"), hide = false)]
    #[value(alias("cls"), hide = false)]
    #[value(alias("class"), hide = false)]
    #[value(alias("label"), hide = false)]
    Classification,
    #[value(alias("sid"), hide = false)]
    #[value(alias("sensor_id"), hide = false)]
    #[value(alias("source_id"), hide = false)]
    SourceID,
    #[value(alias("i"), hide = false)]
    #[value(alias("int"), hide = false)]
    Intensity,
    #[value(alias("rgb"), hide = false)]
    #[value(alias("color"), hide = false)]
    Color,
    #[value(alias("t"), hide = false)]
    #[value(alias("time"), hide = false)]
    #[value(alias("gps"), hide = false)]
    GpsTime,
    #[value(alias("u"), hide = false)]
    #[value(alias("user"), hide = false)]
    UserData,
    #[value(alias("rn"), hide = false)]
    ReturnNumber,
    #[value(alias("nor"), hide = false)]
    NumberOfReturns,
    #[value(alias("s"), hide = false)]
    Skip,
}

#[derive(clap::ValueEnum, Clone, Debug)]
pub enum LabelField {
    #[value(alias("c"), hide = false)]
    #[value(alias("cls"), hide = false)]
    #[value(alias("class"), hide = false)]
    #[value(alias("label"), hide = false)]
    Classification,
    #[value(alias("sid"), hide = false)]
    #[value(alias("sensor_id"), hide = false)]
    #[value(alias("source_id"), hide = false)]
    SourceID,
}

#[derive(Debug, Args)]
pub struct PointcloudSummaryArgs {
    /// Supported pointcloud formats: [LAS, LAZ, PCD, PLY]
    #[arg(required = true, value_delimiter = ' ', num_args = 1..)]
    pub input: Vec<String>,

    /// These should be passed in same order as the fields
    /// have in the input file, and we expect that first dynamic
    /// field comes after fields XYZ (4th field in data).
    /// E.g. if we have PCD file with fields `x, y, z, class`
    /// you may pass `-d label`.
    ///
    /// - `classification`: Classification field on the input data.
    /// Allows extracting additional information related to classes.
    /// Expects the classification field to be uint8, i.e. defined between
    /// 0-255.
    /// If the input data for this field is not U8, the field is attempted
    /// to be cast to U8.
    /// Has aliases cls, class, label, classification.
    ///
    /// - `skip`: Skips reading the dynamic field at its position. Can be used to read columns that are
    /// unsupported by the tool.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// If provided, recursively process directories
    #[clap(short, long)]
    pub recursive: bool,
}

#[derive(Debug, Args)]
pub struct PointcloudVisualizationArgs {
    #[arg(required = true)]
    /// Path to input file, either [PCD, LAS, LAZ, PLY].
    pub input: String,
    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value_t = 1.0)]
    pub factor: f64,
    /// Visualized point radius (in pointcloud units). Defaults to 0.03 (at default voxel size 0.1).
    /// If voxel size is changed, this radius is scaled linearly.
    /// If provided on command line, this value is used as the base for scaling.
    #[clap(short, long, default_value_t = 0.03)]
    pub radii: f32,
    /// Downsampling voxel size, in data coordinate units. Defaults to 0.1.
    /// Radius of visualized points is scaled linearly according to this.
    #[clap(short, long, default_value_t = 0.1)]
    pub voxel_size: f32,
    /// These should be passed in same order as the fields
    /// have in the input file, and we expect that first dynamic
    /// field comes after fields XYZ (4th field in data).
    /// E.g. if we have PCD file with fields `x, y, z, class`
    /// you may pass `-d class`.
    ///
    /// - `classification`: Classification field on the input data.
    /// Allows extracting additional information related to classes.
    /// Expects the classification field to be uint8, i.e. defined between
    /// 0-255.
    /// If the input data for this field is not U8, the field is attempted
    /// to be cast to U8.
    /// Has aliases cls, class, label, classification.
    ///
    /// - `skip`: Skips reading the dynamic field at its position. Useful for unsupported fields.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    /// This field indicates which of the captured (not skipped) `dynamic_fields`
    /// is used as the `class_ids` field of the visualized pointcloud on Rerun.
    /// It defaults to the dynamic field of type `classification`.
    #[clap(short, long, value_enum, default_value = "class")]
    pub label_field: LabelField,
}

#[derive(Debug, Args)]
pub struct PointcloudConvertArgs {
    /// Supported pointcloud formats: [PCD, LAS, LAZ, PLY]
    #[arg(required = true)]
    pub input: String,
    /// Supported pointcloud formats: [PCD, LAS, LAZ, PLY]
    /// If input is a directory, this should also be a directory.
    #[arg(required = true)]
    pub output: String,

    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// These should be passed in same order as the fields
    /// have in the input file, and we expect that first dynamic
    /// field comes after fields XYZ (4th field in data).
    /// E.g. if we have PCD file with fields `x, y, z, class`
    /// you may pass `-d label`.
    ///
    /// - `classification`: Classification field on the input data.
    /// Allows extracting additional information related to classes.
    /// Expects the classification field to be uint8, i.e. defined between
    /// 0-255.
    /// If the input data for this field is not U8, the field is attempted
    /// to be cast to U8.
    /// Has aliases cls, class, label, classification.
    ///
    /// - `skip`: Skips reading the dynamic field at its position. Can be used to read columns that are
    /// unsupported by the tool.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    /// If provided, recursively process directories.
    #[clap(short, long)]
    pub recursive: bool,

    /// Target format for directory conversion (e.g., "ply", "pcd", "las", "laz").
    /// Required if input is a directory.
    #[clap(short, long)]
    pub format: Option<String>,
}

#[derive(Debug, Args)]
pub struct PointcloudMergeArgs {
    /// Directory containing pointcloud files.
    #[arg(required = true)]
    pub input: String,
    /// Output pointcloud file.
    #[arg(required = true)]
    pub output: String,

    /// Path to a .npy file containing an n_frames x 4 x 4 numpy array of transforms.
    #[clap(short, long)]
    pub poses: Option<String>,

    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// Optional voxel size for downsampling the merged supercloud.
    #[clap(short, long)]
    pub voxel_size: Option<f64>,

    /// Dynamic fields mapping, same as in other commands.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    /// If provided, recursively process directories.
    #[clap(short, long)]
    pub recursive: bool,
}

#[derive(Debug, Args)]
pub struct PointcloudVoxelizeArgs {
    /// Input pointcloud file or directory.
    #[arg(required = true)]
    pub input: String,
    /// Output pointcloud file or directory.
    #[arg(required = true)]
    pub output: String,

    /// Voxel size for downsampling.
    #[arg(required = true)]
    pub voxel_size: f64,

    /// Scales XYZ coordinates on load by this factor (factor x XYZ).
    #[clap(short, long, default_value = "1.0")]
    pub factor: f64,

    /// Dynamic fields mapping, same as in other commands.
    #[clap(short, long, value_enum, num_args = 0..)]
    pub dynamic_fields: Vec<DynFieldType>,

    /// If provided, recursively process directories.
    #[clap(short, long)]
    pub recursive: bool,

    /// Target format for directory conversion (e.g., "ply", "pcd", "las", "laz").
    /// Required if input is a directory.
    #[clap(short, long)]
    pub format: Option<String>,
}

// Error handling utility that can be used by both lib and binary
pub fn handle_error(e: anyhow::Error) {
    eprintln!("Error!");
    for (i, cause) in e.chain().enumerate() {
        eprintln!("  Cause {}: {}", i, cause);
    }
    std::process::exit(1);
}
