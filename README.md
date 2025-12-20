# pcutil

Various operations for transforming and exploring pointcloud data.

## Installation

```bash
cargo install --path .
```

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
