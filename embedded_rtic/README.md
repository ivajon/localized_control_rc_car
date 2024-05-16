<h1 align="center">
  <a href="https://github.com/rtic-rs/rtic">RTIC</a> + <a href="https://github.com/knurling-rs/defmt">defmt</a> embedded rust application.
</h1>

This is our base embedded application it is based on the [rtic-rs](https://github.com/rtic-rs/defmt-app-template) example.

## Initial optimizations

All of the code runs from RAM, the linker also places the `.data` region in RAM to ensure the shortest possible load times.

## Sensors

These are the peripherals we will be using to get information about our environment.

- [sonar](https://se.rs-online.com/web/p/hall-effect-sensors/7659325)
- [hall-effect](https://se.rs-online.com/web/p/hall-effect-sensors/7659325)

The car should have `3` sonars mounted, one forward and two side-firing. Which, the code then smooths and removes outliers to ensure that the data is predictable and has low variance.

## Outputs

The system's `outputs` are simply the pwm signal to the motor and the servo. These are controlled using PID controllers provided by [`the shared library`](../shared/). The Control parameters for these are defined in [`the car bsp`](./src/car.rs) that defines various constants.

## Inter-board communications

We should use the [DMA](https://github.com/nrf-rs/nrf-hal/tree/master/examples/spi-demo) to allow transactions without affecting the control
logic. Likely it is better to run SPI rather than i2c as i2c is somewhat flaky in RTIC.
Likely this communication is one way aside from some handshaking that might need doing.
