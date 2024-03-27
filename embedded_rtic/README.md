<h1 align="center">
  <a href="https://github.com/rtic-rs/rtic">RTIC</a> + <a href="https://github.com/knurling-rs/defmt">defmt</a> embedded rust application.
</h1>

This is our base embedded application it is based on the [rtic-rs](https://github.com/rtic-rs/defmt-app-template) example.

## Prerequisite

To be able to run this we `might` need to use a fork of probe-rs that side-steps the flashing rules by remapping the ram addresses to dcode addresses while flashing
<https://github.com/ivario123/probe-rs>.

Installation:

```bash
echo "creating a temporary directory."
mkdir -p tmp
cd tmp
echo "retrieving source."
git clone https://github.com/ivario123/probe-rs > /dev/null

echo "installing fulhack."
cd probe-rs
cargo install --path ./probe-rs > /dev/null

cd ../..
echo "cleaning up."
rm -rf tmp
```

This should only be needed if we run in to issues where `probe` fails due to no contigous memory regions containing those addresses.



## Initial optimizations

All of the code runs from RAM, the linker also places the `.data` region in RAM to ensure the shortest possible load times.

## Sensors

These are the peripherals we will be using to get information about our environment.

- [sonar](https://se.rs-online.com/web/p/hall-effect-sensors/7659325)
- [hall-effect](https://se.rs-online.com/web/p/hall-effect-sensors/7659325)

## Outputs

We will likely have 1-4 outputs for the motors, these should be individually controlled using PID controllers tuned to follow a reference in velocity and those four should in turn be controlled by a PID controller
for position.
Thus it would be ideal to offload all heavy computing i.e. [CV](https://github.com/alishobeiri/Monocular-Video-Odometery) to the [SBC](https://se.rs-online.com/web/p/rock-sbc-boards/2209536) allowing the dev-kit to only run control logic and some simple other logic, maybe some logging etc.

## Inter-board communications

We should use the [DMA](https://github.com/nrf-rs/nrf-hal/tree/master/examples/spi-demo) to allow transactions without affecting the control
logic. Likely it is better to run SPI rather than i2c as i2c is somewhat flaky in RTIC.
Likely this communication is one way aside from some handshaking that might need doing.
