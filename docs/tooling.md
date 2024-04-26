# Tooling

This is a non exhaustive list of install instructions for the different tooling.

## Rust

The easiest way of installing this is using the [rustup](https://rustup.rs/) tool, it installs most of what is needed.

### Probe-rs

This is only needed if you want to build the embedded code

```bash
cargo binstall probe-rs
```

This might need to be replaced in the future if we need to do that I will add documentation for that.

### Cortex-m4 target

This is easy to install

```bash
rustup target add thumbv7em-none-eabihf
```

Now you should be able to build the embedded code.

## [Open-cv](https://opencv.org/)

This is a common computer vision library with [rust bindings](https://github.com/twistedfall/opencv-rust) which it self provides [install instructions](https://github.com/twistedfall/opencv-rust/blob/master/INSTALL.md). It also provides python bindings but we should probably use c++ or rust for speed.

Guide for installing OpenCV for C++: https://www.opencv-srf.com/p/introduction.html.