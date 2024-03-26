# E7012E_software
This repo contains the software for our self-driving model car implemented for the course E7012E at LTU.

## Structure

This repo is split into 3 crates, [hosted](./hosted) [embedded_rtic](./embedded_rtic) [shared](./shared)
where shared and embedded_rtic are [no_std](https://docs.rust-embedded.org/book/intro/no-std.html) which allows them to run on both a hosted
and an embedded system.
The [hosted](./hosted) defines the main binary that will run on the [SBC](https://se.rs-online.com/web/p/rock-sbc-boards/2209536),
this code will likely be responsible for path planning using computer vision.
