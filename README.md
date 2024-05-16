# E7012E_software
This repo contains the software for our self-driving model car implemented for the course E7012E at LTU.

## Structure

This repo is split into 3 crates, [hosted](./hosted) [embedded_rtic](./embedded_rtic) [shared](./shared)
where shared and embedded_rtic are [no_std](https://docs.rust-embedded.org/book/intro/no-std.html) which allows them to run on both a hosted
and an embedded system.
The [hosted](./hosted) defines the main binary that will run on the [SBC](https://se.rs-online.com/web/p/rock-sbc-boards/2209536),
this code will likely be responsible for path planning using computer vision.

There is also a distinct computer_vision_draft directory that contians all computer vision related software.


## Labs

### LAB2

This is covered by the [`race`](embedded_rtic/src/bin/race.rs) binary which uses the encoder and the servo/esc.

### LAB3

The controllers are implemented in the [`shared`](./shared) library. There are a few, fixed sampling rate, dynamic and a gain scheduling one. The parameters for the PIDs can be found in  [`the car bsp`](embedded_rtic/src/car.rs). The SPI communication is done in the [`race`](embedded_rtic/src/bin/race.rs) binary and the [`hosted`](hosted) software. The protocol for SPI communication is defined in the [`shared`](shared) library.


### LAB4

This is just the [`race`](embedded_rtic/src/bin/race.rs) binary.
The start/stop detection is done in the [`computer_vision_draft`](computer_vision_draft) project in cpp.
