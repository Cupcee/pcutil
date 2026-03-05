# pcutil

Various operations for transforming and exploring pointcloud data.

## Caveats

This tool has been tested to work with MacOS and Ubuntu 24.04 LTS. Windows
support is untested.

## Prerequisites

- `rustc` (tested with version `1.90.0`)
- `Cargo` (tested with version `1.90.0`)
- Rerun-CLI (for `visualize`, see below for how to install this with Cargo)
- (Optional: `cargo-binstall`, for installing Rerun)

## Installation

Installation to path can be done with Cargo:

```bash
cargo install --path .
```

This installs the binary to path to alias `pcutil`.

To use `visualize`, install also the Rerun-cli. Either install
`rerun-cli` with some method of your own, or follow below method:

First, install `cargo-binstall` if it is missing:

```sh
# linux / macOS:
curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash
# windows:
Set-ExecutionPolicy Unrestricted -Scope Process; iex (iwr "https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.ps1").Content
```

Then install `rerun-cli`:

```sh
cargo binstall --force rerun-cli@0.24.1
```

Then, place a YAML file like:

```yaml
# Pointcloud Utility Configuration
# This file maps classification IDs to [Label, [R, G, B]]
classmap:
  0: ["Ground", [120, 40, 120]]
  1: ["Person", [200, 160, 29]]
  2: ["Box", [185, 131, 44]]
  # ... more
sensormap:
  0: ["Front", [255, 0, 0]]
  1: ["Left", [0, 255, 0]]
  # ... more

```

to your home directory as `$HOME/.pcutil.yaml`. This config
assigns class labels and colors to each point by its class index.

Sensormap alternatively maps the points according to sensor ID,
if this data is available.

## Usage

Currently tools are: `summary`, `visualize`, `convert`, `merge`, `voxelize`.

### PCD Field Auto-detection

For PCD files, `pcutil` attempts to automatically detect and map fields following the first three (XYZ) to standard point cloud attributes based on their names in the PCD header (case-insensitive). This removes the need to manually specify `-d` or `--dynamic-fields` for standard files.

The following mappings are supported:

| PCD Field Name | Mapped Attribute |
| :--- | :--- |
| `intensity` | Intensity |
| `rgb`, `rgba` | Color (supports packed U32/F32) |
| `label`, `classification` | Classification |
| `gps_time`, `timestamp` | GPS Time |
| `user_data` | User Data |
| `point_source_id` | Point Source ID |
| `return_number` | Return Number |
| `number_of_returns` | Number of Returns |

If a field name is not recognized, it is automatically skipped. You can still use the `-d` flag to manually define the mapping or to skip specific fields by position, which will override the auto-detection logic.

### Summary

Summary gives aggregated information from a single or a directory of pointcloud
files. Useful e.g. for summarizing pointcloud datasets.

```sh
Summarize files in a given path

Usage: pcutil summary [OPTIONS] <INPUT>...

Arguments:
  <INPUT>...
          Supported pointcloud formats: [LAS, LAZ, PCD, PLY]

Options:
  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file. For PCD files, standard fields (intensity, rgb, label, etc.) are automatically detected if this flag is omitted.

          - `classification`: Classification field. (aliases: c, cls, class, label)
          - `source_id`: Sensor/Source ID field. (aliases: sid, sensor_id, source_id)
          - `intensity`: Intensity field. (aliases: i, int)
          - `color`: RGB color field. (aliases: rgb, color)
          - `gps_time`: GPS timestamp field. (aliases: t, time, gps)
          - `user_data`: User data field. (aliases: u, user)
          - `return_number`: Return number field. (alias: rn)
          - `number_of_returns`: Total number of returns. (alias: nor)
          - `skip`: Skips reading the dynamic field at its position. (alias: s)

  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1.0]

  -r, --recursive
          If provided, recursively process directories

  -h, --help
          Print help (see a summary with '-h')
```

Examples:

```sh
# PCD files auto-detect standard fields, so usually you don't need -d:
pcutil summary path/to/file.pcd

# If you have non-standard fields or want to override auto-detection:
pcutil summary path/to/file.pcd -d class

# Commonly, a pointcloud format file has also other header data that is unimportant.
# For this, we have the `skip` positional argument.
# E.g. If we have a PCD file with fields `x, y, z, field1, field2, class_idx`,
# we can do:
pcutil summary path/to/file.pcd -d skip skip class # <-- here first two skips skip field1, field2

# Directory also works, in this case it processes all files in the directory
pcutil summary path/to/dir/ -d class

# Or, recursively...
pcutil summary -r path/to/root/ -d class
```

### Visualize

Visualize visualizes the input pointcloud file through the Rerun-cli
(make sure it's installed).

```sh
Visualize point cloud using Rerun

Usage: pcutil visualize [OPTIONS] <INPUT>

Arguments:
  <INPUT>
          Path to input file, either [PCD, LAS, LAZ, PLY]

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1]

  -r, --radii <RADII>
          Visualized point radius (in pointcloud units). Defaults to 0.03.
          This value is scaled linearly by the voxel-size (referenced to default voxel size 0.1).
          If provided on command line, this value is used as the base for scaling.

  -v, --voxel-size <VOXEL_SIZE>
          Downsampling voxel size, in data coordinate units. Defaults to 0.1.
          Radius of visualized points is scaled linearly according to this.

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file. For PCD files, standard fields (intensity, rgb, label, etc.) are automatically detected if this flag is omitted.

          - `classification`: Classification field. (aliases: c, cls, class, label)
          - `source_id`: Sensor/Source ID field. (aliases: sid, sensor_id, source_id)
          - `intensity`: Intensity field. (aliases: i, int)
          - `color`: RGB color field. (aliases: rgb, color)
          - `gps_time`: GPS timestamp field. (aliases: t, time, gps)
          - `user_data`: User data field. (aliases: u, user)
          - `return_number`: Return number field. (alias: rn)
          - `number_of_returns`: Total number of returns. (alias: nor)
          - `skip`: Skips reading the dynamic field at its position. (alias: s)

  -l, --label-field <LABEL_FIELD>
          Indicates which of the captured dynamic fields is used as the `class_ids` for the visualized pointcloud in Rerun. Defaults to `classification`.

  -h, --help
          Print help (see a summary with '-h')
```

### Convert

Convert pointcloud from one format to another. Supported formats include `pcd`, `las`, `laz`, and `ply`.

```sh
Convert pointcloud file from one format to another

Usage: pcutil convert [OPTIONS] <INPUT> <OUTPUT>

Arguments:
  <INPUT>
          Supported pointcloud formats: [PCD, LAS, LAZ, PLY]

  <OUTPUT>
          Supported pointcloud formats: [PCD, LAS, LAZ, PLY]

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1.0]

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file. For PCD files, standard fields (intensity, rgb, label, etc.) are automatically detected if this flag is omitted.

          - `classification`: Classification field. (aliases: c, cls, class, label)
          - `source_id`: Sensor/Source ID field. (aliases: sid, sensor_id, source_id)
          - `intensity`: Intensity field. (aliases: i, int)
          - `color`: RGB color field. (aliases: rgb, color)
          - `gps_time`: GPS timestamp field. (aliases: t, time, gps)
          - `user_data`: User data field. (aliases: u, user)
          - `return_number`: Return number field. (alias: rn)
          - `number_of_returns`: Total number of returns. (alias: nor)
          - `skip`: Skips reading the dynamic field at its position. (alias: s)

  -h, --help
          Print help (see a summary with '-h')
```

Examples:

```sh
# assuming file.pcd has headers x, y, z, class_idx_field
# Standard fields like classification are auto-detected from PCD.
pcutil convert path/to/file.pcd path/to/file.laz

# Convert PCD to PLY, auto-detecting all fields
pcutil convert path/to/file.pcd path/to/file.ply

# Convert PLY to PCD (standard attributes like intensity, rgb are preserved)
pcutil convert path/to/file.ply path/to/file.pcd
```

### Merge

Merge multiple pointcloud files from a directory into a single composite
supercloud. Optionally, an external `.npy` file can be provided containing
transformation matrices (4x4) for each frame.

You may obtain pose files with the accepted format e.g. with the `KISS-ICP` LiDAR
Odometry pipeline: [link](https://github.com/PRBonn/kiss-icp).

```sh
Merge multiple pointcloud files into one supercloud

Usage: pcutil merge [OPTIONS] <INPUT> <OUTPUT>

Arguments:
  <INPUT>
          Directory containing pointcloud files

  <OUTPUT>
          Output pointcloud file (supports .las, .laz, .pcd, .ply)

Options:
  -p, --poses <POSES>
          Path to a .npy file containing an n_frames x 4 x 4 numpy array of transforms (f64).
          Frames are assigned transforms in lexicographical order.

  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1.0]

  -v, --voxel-size <VOXEL_SIZE>
          Optional voxel size for downsampling the merged supercloud.

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          Dynamic fields mapping, same as in other commands.

  -r, --recursive
          If provided, recursively process directories

  -h, --help
          Print help (see a summary with '-h')
```

Example:

```sh
# Merge all PCD files in a directory into one LAS file
pcutil merge path/to/frames/ supercloud.las

# Merge with transformation matrices and voxelize the output
pcutil merge path/to/frames/ supercloud.pcd --poses poses.npy --voxel-size 0.05
```

### Voxelize

Perform voxelization (downsampling) on one or more pointcloud files. This keeps only one point per voxel of the specified size, significantly reducing the dataset size while preserving the overall structure.

```sh
Voxelize (downsample) pointcloud file(s)

Usage: pcutil voxelize [OPTIONS] <INPUT> <OUTPUT> <VOXEL_SIZE>

Arguments:
  <INPUT>
          Input pointcloud file or directory

  <OUTPUT>
          Output pointcloud file or directory

  <VOXEL_SIZE>
          Voxel size for downsampling (in data coordinate units)

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1.0]

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          Dynamic fields mapping, same as in other commands.

  -r, --recursive
          If provided, recursively process directories

  -t, --format <FORMAT>
          Target format for directory conversion (e.g., "ply", "pcd", "las", "laz").
          Required if input is a directory.

  -h, --help
          Print help (see a summary with '-h')
```

Example:

```sh
# Voxelize a single file with a 10cm voxel size
pcutil voxelize cloud.las voxelized.las 0.1

# Voxelize an entire directory of files
pcutil voxelize input_dir/ output_dir/ 0.05 --format pcd
```

