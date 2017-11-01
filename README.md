# gear [![Build Status](https://travis-ci.org/OTL/gear.svg?branch=master)](https://travis-ci.org/OTL/gear) [![crates.io](https://img.shields.io/crates/v/gear.svg)](https://crates.io/crates/gear)

Collision Avoidance Path Planning for robotics in Rust-lang

```
$ cargo run --release --example reach
```

then,

* Up/Down/Left/Right/`f`/`b` to move IK target
* type `m` to use current pose as init pose
* type `i` to reach the target
* type `p` to plan

* type `r` to set random pose
* type `c` to check collision


The example can handle any urdf files (sample.urdf is used as default)

```
$ cargo run --release --example reach YOUR_URDF_FILE_PATH
```

[Video](https://www.youtube.com/watch?v=fYfZR1f2HW0)
