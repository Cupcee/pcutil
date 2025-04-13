# pcutil

Various operations for transforming and exploring pointcloud data.

## Installation

```bash
cargo install --path .
```

## Commands

### `pointcloud summary`

Summarize files in a given path.

__Supported Extensions__ `.pcd`, `.las`, `.laz`
__Input__ Can be a single file or directory (non-recursive, only immediate children)
__Args__
    - `-d, --strict-pcd-schema`: If provided, the schema is expected to be precisely
    `x`, `y`, `z`, `rgb` where all are F32. Otherwise, we only try dynamically
    parsing `x`, `y`, `z` information.

```bash
pcutil summary [--strict-pcd-schema] <target>
```

Output:

```txt
Total number of files: 5
Failed to read: 0 files
Unique filetypes: 'las', 'laz', 'pcd'
Total number of points: 639
Mean number of points per file: 213.00
Median number of points per file: 213.00
Bounding box:
  X: [-0.9552599787712097, 0.9918500185012817]
  Y: [-0.3154599964618683, 0.3564099967479706]
  Z: [0, 0]
```

### `pointcloud convert`

Convert pointcloud file from one format to another.

__Supported inputs__ `.pcd` (single file)
__Supported outputs__ `.laz`, `.las` (single file)
__Args__
    - `-d, --strict-pcd-schema`: If provided, the schema is expected to be precisely
    `x`, `y`, `z`, `rgb` where all are F32. Otherwise, we only try dynamically
    parsing `x`, `y`, `z` information.

```bash
pcutil convert [--strict-pcd-schema] <input-file-path> <output-file-path>

