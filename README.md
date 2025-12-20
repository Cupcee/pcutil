# pcutil

Various operations for transforming and exploring pointcloud data.

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
  0: ["Ground", [44, 57, 54]]
  1: ["Cable", [225, 111, 6]]
  2: ["Box", [185, 131, 44]]
  3: ["Rock", [231, 205, 232]]
  4: ["Pile", [151, 243, 194]]
  5: ["Barricade", [244, 199, 13]]
  6: ["Person", [255, 91, 219]]
  7: ["Vehicle", [38, 23, 180]]
  8: ["Pole", [208, 138, 11]]
  9: ["Hole", [182, 37, 231]]
  10: ["Sign", [88, 237, 6]]
  11: ["Building", [55, 151, 131]]
  12: ["Container", [130, 28, 79]]
  13: ["TanksAndSilos", [206, 191, 98]]
  14: ["Tree", [39, 150, 54]]
  15: ["Pipe", [200, 160, 29]]
  16: ["Fence", [243, 136, 95]]
  17: ["Other", [237, 126, 132]]
  18: ["Cliff", [170, 25, 34]]
```

to your home directory as `$HOME/.pcutil.yaml`. This config
assigns class labels and colors to each point by its class index.

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

          - `classification`: Classification field on the input data. Allows extracting additional information related to classes. If the input data for this field is not U8, reading the file fails (but program does not panic). Has aliases cls, class, label, classification.

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

```sh
# If we have a PCD file with fields `x, y, z, class_idx`, we can do:
pcutil summary path/to/file.pcd -d class

# If we have a PCD file with fields `x, y, z, field1, field2, class_idx`, we can do:
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

Options:
  -f, --factor <FACTOR>
          Scales XYZ coordinates on load by this factor (factor x XYZ) [default: 1]
  -d, --dynamic-fields <DYNAMIC_FIELDS>
          [possible values: classification, skip]
  -h, --help
          Print help
```

### Convert

Convert pointcloud from one format to another. Currently, `pcd` <-> `las` / `laz`
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
          These should be passed in same order as the fields have in the input file, and we expect that first dynamic field comes after fields XYZ (4th field in data). E.g. if we have PCD file with fields x, y, z, class you may pass `-d class`.

          - `classification`: Classification field on the input data. Allows extracting additional information related to classes. If the input data for this field is not U8, reading the file fails (but program does not panic). Has aliases cls, class, label, classification.

          - `skip`: Skips reading the dynamic field at its position. Useful for unsupported fields.

          [possible values: classification, skip]

  -h, --help
          Print help (see a summary with '-h')
```
