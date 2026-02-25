# pcutil

Various operations for transforming and exploring pointcloud data.

## Prerequisites

- `rustc` (tested with version `1.90.0`)
- `Cargo` (tested with version `1.90.0`)
- Rerun-CLI (for `visualize`, see below for how to install this with Cargo)

## Installation

Installation to path can be done with Cargo:

```bash
cargo install --path .
```

This installs the binary to path to alias `pcutil`.

To use `visualize`, install also the Rerun-cli:

```sh
cargo binstall --force rerun-cli@0.24.1`.
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

Currently tools are: `summary`, `visualize`, `convert`.

### Summary

Summary gives aggregated information from a single or a directory of pointcloud
files. Useful e.g. for summarizing pointcloud datasets.

```sh
Summarize files in a given path

Usage: pcutil summary [OPTIONS] <INPUT>...

Arguments:
  <INPUT>...
          Supported pointcloud formats: [LAS, LAZ, PCD]

Options:
  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file, and we expect that first dynamic field comes after fields XYZ (4th field in data). E.g. if we have PCD file with fields x, y, z, class you may pass `-d label`.

          - `classification`: Classification field on the input data. Allows extracting additional information related to classes. Expects the classification field to be uint8, i.e. defined between 0-255. If the input data for this field is not U8, the field is attempted to be cast to U8. Has aliases cls, class, label, classification.

          - `skip`: Skips reading the dynamic field at its position. Can be used to read columns that are unsupported by the tool.

          [possible values: classification, skip]

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
# If we have a PCD file with fields `x, y, z, class_idx`, we can do:
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
          Path to input file, either [PCD, LAS, LAZ]

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1]

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file, and we expect that first dynamic field comes after fields XYZ (4th field in data). E.g. if we have PCD file with fields x, y, z, class you may pass `-d class`.

          - `classification`: Classification field on the input data. Allows extracting additional information related to classes. Expects the classification field to be uint8, i.e. defined between 0-255. If the input data for this field is not U8, the field is attempted to be cast to U8. Has aliases cls, class, label, classification.

          - `sensor_id`: Sensor ID field in the input data. This allows visualizing which point belongs to which sensor, in a multi-sensor configuration. See sensormap in the configuration file for coloring options. Has aliases sid, sensor_id, source_id.

          - `skip`: Skips reading the dynamic field at its position. Useful for unsupported fields.

          [possible values: classification, sensor_id, skip]

  -h, --help
          Print help (see a summary with '-h')
```

### Convert

Convert pointcloud from one format to another. Currently, only `pcd` -> `las` / `laz`
is supported.

```sh
Convert pointcloud file from one format to another

Usage: pcutil convert [OPTIONS] <INPUT> <OUTPUT>

Arguments:
  <INPUT>
          Supported pointcloud formats: [PCD]

  <OUTPUT>
          Supported pointcloud formats: [LAS, LAZ]

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ)

          [default: 1.0]

  -d, --dynamic-fields [<DYNAMIC_FIELDS>...]
          These should be passed in same order as the fields have in the input file, and we expect that first dynamic field comes after fields XYZ (4th field in data). E.g. if we have PCD file with fields x, y, z, class you may pass `-d label`.

          - `classification`: Classification field on the input data. Allows extracting additional information related to classes. Expects the classification field to be uint8, i.e. defined between 0-255. If the input data for this field is not U8, the field is attempted to be cast to U8. Has aliases cls, class, label, classification.

          - `skip`: Skips reading the dynamic field at its position. Can be used to read columns that are unsupported by the tool.

          [possible values: classification, skip]

  -h, --help
          Print help (see a summary with '-h')
```

Examples:

```sh
# assuming file.pcd has headers x, y, z, class_idx_field
pcutil convert path/to/file.pcd path/to/file.laz -d class
```
