# gear [![Build Status](https://travis-ci.org/OTL/gear.svg?branch=master)](https://travis-ci.org/OTL/gear) [![crates.io](https://img.shields.io/crates/v/gear.svg)](https://crates.io/crates/gear)

Collision Avoidance Path Planning for robotics in Rust-lang

## Run example with GUI

```bash
cargo run --release --example reach
```

then,

* Up/Down/Left/Right/`f`/`b` to move IK target
* type `g` to move the end of the arm to the target

* type `i` to just solve inverse kinematics for the target
* type `r` to set random pose
* type `c` to check collision


The example can handle any urdf files (sample.urdf is used as default)

```bash
cargo run --release --example reach YOUR_URDF_FILE_PATH
```

[Video](https://www.youtube.com/watch?v=fYfZR1f2HW0)
