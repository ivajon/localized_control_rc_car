//! Provides default pinmappings for the car.

use nrf52840_hal::{
    gpio::{self, p0, p1, Floating, Input, Output, Pin, PullDown, PullUp, PushPull},
    gpiote::Gpiote,
    ppi::{self, ConfigurablePpi, Ppi},
    spis,
};

use super::{
    event::EventManager,
    wrappers::{EscPwm, MotorController, ServoController, ServoPwm, SpiInstance},
};
use crate::{esc::Esc, servo::Servo};

/// Wraps the pins needed for a sonar.
pub struct SonarPins {
    /// Triggers a new sonar measurement.
    pub trigger: Pin<Output<PushPull>>,
    /// Goes high for a period that indicates the distance to the closest
    /// object.
    pub echo: Pin<Input<PullDown>>,
}

/// Wraps the pins needed for the SPI.
pub struct SpiPins {
    /// Incomming packets from the sbc.
    pub mosi: Pin<Input<Floating>>,
    /// Outgoing packets to the sbc.
    pub miso: Pin<Input<Floating>>,
    /// Clock driven from the sbc.
    pub sck: Pin<Input<Floating>>,
    /// Says that the packets are directed to us.
    pub cs: Pin<Input<Floating>>,
}

/// The pin mapping used in the car.
pub struct PinMapping<
    const SPI_USED: bool,
    const SERVO_USED: bool,
    const ESC_USED: bool,
    const EVENTS_CONFIGURED: bool,
> {
    /// The pins used for the forward sonar.
    sonar_forward: SonarPins,
    /// The pins used for the left sonar.
    sonar_left: SonarPins,
    /// The pins used for the right sonar.
    sonar_right: SonarPins,
    /// The pin used for the [`Servo`](crate::servo::Servo).
    servo_output: Option<Pin<Output<PushPull>>>,
    /// The pin used for the [`Esc`](crate::esc::Esc).
    esc_output: Option<Pin<Output<PushPull>>>,
    /// The pin used to trigger interrupts for the encoder.
    encoder: Pin<Input<PullUp>>,
    /// The pins connected to the spi.
    spi: Option<SpiPins>,
}

impl PinMapping<false, false, false, false> {
    /// Instantiates a new pin mapping.
    pub fn new(p0: p0::Parts, p1: p1::Parts) -> Self {
        // Outputs
        let esc_output = p0.p0_05.into_push_pull_output(gpio::Level::High).degrade();
        let servo_output = p0.p0_06.into_push_pull_output(gpio::Level::High).degrade();

        // Inputs
        let encoder = p1.p1_02.into_pullup_input().degrade();

        // Sonars

        let sonar_forward = SonarPins {
            trigger: p0.p0_12.into_push_pull_output(gpio::Level::Low).degrade(),
            echo: p0.p0_11.into_pulldown_input().degrade(),
        };

        let sonar_left = SonarPins {
            trigger: p1.p1_06.into_push_pull_output(gpio::Level::Low).degrade(),
            echo: p1.p1_05.into_pulldown_input().degrade(),
        };

        let sonar_right = SonarPins {
            trigger: p1.p1_03.into_push_pull_output(gpio::Level::Low).degrade(),
            echo: p1.p1_04.into_pulldown_input().degrade(),
        };

        // Communications.

        let mosi = p1.p1_12.into_floating_input().degrade();
        let miso = p1.p1_13.into_floating_input().degrade();
        let sck = p1.p1_14.into_floating_input().degrade();
        let cs = p1.p1_11.into_floating_input().degrade();

        let spi_pins = SpiPins {
            mosi,
            miso,
            sck,
            cs,
        };

        PinMapping {
            sonar_left,
            sonar_right,
            sonar_forward,
            servo_output: Some(servo_output),
            esc_output: Some(esc_output),
            encoder,
            spi: Some(spi_pins),
        }
    }
}

impl<const SERVO_USED: bool, const ESC_USED: bool, const EVENTS_CONFIGURED: bool>
    PinMapping<false, SERVO_USED, ESC_USED, EVENTS_CONFIGURED>
{
    /// Creates a new spis device which triggers an event on transfer end.
    pub fn spi(
        mut self,
        device: SpiInstance,
        buffer: &'static mut [u8],
    ) -> (
        PinMapping<true, SERVO_USED, ESC_USED, EVENTS_CONFIGURED>,
        spis::Transfer<SpiInstance, &'static mut [u8]>,
    ) {
        // This is checked by the type system.
        let spis_pins = unsafe { self.spi.take().unwrap_unchecked() }.into();
        let spi = spis::Spis::new(device, spis_pins);
        spi.enable_interrupt(spis::SpisEvent::End);
        let spis = spi.transfer(buffer).unwrap_or_else(|_| panic!());
        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: self.servo_output,
            esc_output: self.esc_output,
            encoder: self.encoder,
            spi: None,
        };
        (new_self, spis)
    }
}

impl<const SPI_USED: bool, const SERVO_USED: bool, const EVENTS_CONFIGURED: bool>
    PinMapping<SPI_USED, SERVO_USED, false, EVENTS_CONFIGURED>
{
    /// Creates a [`MotorController`] for the car.
    pub fn esc_controller(
        mut self,
        device: EscPwm,
    ) -> (
        PinMapping<SPI_USED, SERVO_USED, true, EVENTS_CONFIGURED>,
        MotorController<EscPwm>,
    ) {
        let esc = unsafe { self.esc_output.take().unwrap_unchecked() };
        let esc = Esc::new(device, esc);
        let controller = MotorController::new(esc);

        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: self.servo_output,
            esc_output: None,
            encoder: self.encoder,
            spi: self.spi,
        };
        (new_self, controller)
    }

    /// Creates a [`Esc`] for the car.
    pub fn esc_raw(
        mut self,
        device: EscPwm,
    ) -> (
        PinMapping<SPI_USED, SERVO_USED, true, EVENTS_CONFIGURED>,
        Esc<EscPwm>,
    ) {
        let esc = unsafe { self.esc_output.take().unwrap_unchecked() };
        let esc = Esc::new(device, esc);

        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: self.servo_output,
            esc_output: None,
            encoder: self.encoder,
            spi: self.spi,
        };
        (new_self, esc)
    }
}

impl<const SPI_USED: bool, const ESC_USED: bool, const EVENTS_CONFIGURED: bool>
    PinMapping<SPI_USED, false, ESC_USED, EVENTS_CONFIGURED>
{
    /// Creates a [`ServoController`] for the car.
    pub fn servo_controller(
        mut self,
        device: ServoPwm,
    ) -> (
        PinMapping<SPI_USED, true, ESC_USED, EVENTS_CONFIGURED>,
        ServoController<ServoPwm>,
    ) {
        let esc = unsafe { self.servo_output.take().unwrap_unchecked() };
        let esc = Servo::new(device, esc);
        let controller = ServoController::new(esc);

        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: None,
            esc_output: self.esc_output,
            encoder: self.encoder,
            spi: self.spi,
        };
        (new_self, controller)
    }

    /// Creates a [`Servo`] for the car.
    pub fn servo_raw(
        mut self,
        device: ServoPwm,
    ) -> (
        PinMapping<SPI_USED, true, ESC_USED, EVENTS_CONFIGURED>,
        Servo<ServoPwm>,
    ) {
        let servo = unsafe { self.esc_output.take().unwrap_unchecked() };
        let servo = Servo::new(device, servo);

        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: None,
            esc_output: self.esc_output,
            encoder: self.encoder,
            spi: self.spi,
        };
        (new_self, servo)
    }
}

impl<const SPI_USED: bool, const SERVO_USED: bool, const ESC_USED: bool>
    PinMapping<SPI_USED, SERVO_USED, ESC_USED, true>
{
    /// Consumes the type returning the raw pins.
    pub fn consume(self) -> (SonarPins, SonarPins, SonarPins, Pin<Input<PullUp>>) {
        (
            self.sonar_forward,
            self.sonar_left,
            self.sonar_right,
            self.encoder,
        )
    }
}

impl<const SPI_USED: bool, const SERVO_USED: bool, const ESC_USED: bool>
    PinMapping<SPI_USED, SERVO_USED, ESC_USED, false>
{
    /// Sets up the events for the inputs.
    pub fn configure_events(
        self,
        gpiote: Gpiote,
        ppi: ppi::Parts,
    ) -> (
        PinMapping<SPI_USED, SERVO_USED, ESC_USED, true>,
        EventManager,
    ) {
        gpiote
            .channel0()
            .input_pin(&self.encoder)
            .toggle()
            .enable_interrupt();
        gpiote
            .channel1()
            .input_pin(&self.sonar_forward.echo)
            .toggle()
            .enable_interrupt();
        gpiote
            .channel2()
            .input_pin(&self.sonar_left.echo)
            .toggle()
            .enable_interrupt();
        gpiote
            .channel3()
            .input_pin(&self.sonar_right.echo)
            .toggle()
            .toggle()
            .enable_interrupt();

        gpiote.port().input_pin(&self.encoder).high();
        gpiote.port().input_pin(&self.sonar_forward.echo).high();
        gpiote.port().input_pin(&self.sonar_left.echo).high();
        gpiote.port().input_pin(&self.sonar_right.echo).high();

        gpiote.port().input_pin(&self.sonar_forward.echo).low();
        gpiote.port().input_pin(&self.sonar_left.echo).low();
        gpiote.port().input_pin(&self.sonar_right.echo).low();
        let mut ppi0 = ppi.ppi0;
        ppi0.set_event_endpoint(gpiote.channel0().event());
        ppi0.set_task_endpoint(gpiote.channel0().task_out());
        ppi0.enable();

        let mut ppi1 = ppi.ppi1;
        ppi1.set_event_endpoint(gpiote.channel1().event());
        ppi1.set_task_endpoint(gpiote.channel1().task_out());
        ppi1.enable();

        let mut ppi2 = ppi.ppi2;
        ppi2.set_event_endpoint(gpiote.channel2().event());
        ppi2.set_task_endpoint(gpiote.channel2().task_out());
        ppi2.enable();

        let mut ppi3 = ppi.ppi3;
        ppi3.set_event_endpoint(gpiote.channel3().event());
        ppi3.set_task_endpoint(gpiote.channel3().task_out());
        ppi3.enable();

        gpiote.port().enable_interrupt();
        let new_self = PinMapping {
            sonar_left: self.sonar_left,
            sonar_right: self.sonar_right,
            sonar_forward: self.sonar_forward,
            servo_output: self.servo_output,
            esc_output: self.esc_output,
            encoder: self.encoder,
            spi: self.spi,
        };
        (new_self, EventManager::new(gpiote))
    }
}

impl SonarPins {
    /// Splits the sonar pins in to (trigger and echo) pins respectivly.
    pub fn split(self) -> (Pin<Output<PushPull>>, Pin<Input<PullDown>>) {
        (self.trigger, self.echo)
    }
}

impl From<SpiPins> for spis::Pins {
    fn from(value: SpiPins) -> spis::Pins {
        nrf52840_hal::spis::Pins {
            sck: value.sck,
            cs: value.cs,
            copi: Some(value.mosi),
            cipo: Some(value.miso),
        }
    }
}
