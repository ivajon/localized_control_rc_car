//! Provides a neat way to manage interrupts.
use nrf52840_hal::gpiote::Gpiote;

use super::constants::Sonar;

/// The events that can trigger gpiote events.
#[derive(Clone)]
pub enum GpioEvents {
    /// The encoder sent a pulse.
    Encoder,
    /// One of the sonars sent a pulse.
    Sonar(Sonar),
}

/// Manages events and provides access to the [`EventIter`]
pub struct EventManager {
    pub(crate) gpiote: Gpiote,
}

/// The order of the snars.
const SONAR_MAPPING: [Sonar; 3] = [Sonar::Forward, Sonar::Left, Sonar::Right];

/// Itterates over the generated events, and clears the previous one
/// automatically after each iteration
pub struct EventIter<'a> {
    idx: usize,
    manager: &'a mut EventManager,
}

impl EventManager {
    /// Constructs a new event manager.
    pub(crate) fn new(gpiote: Gpiote) -> Self {
        Self { gpiote }
    }
}

impl EventManager {
    /// Returns an iterator of the events that have been triggered.
    pub fn events(&mut self) -> EventIter<'_> {
        EventIter {
            idx: 0,
            manager: self,
        }
    }

    /// Clears all channel events
    pub fn clear(&mut self) {
        self.gpiote.reset_events();
        self.gpiote.channel0().clear();
        self.gpiote.channel1().clear();
        self.gpiote.channel2().clear();
        self.gpiote.channel3().clear();
    }
}

impl<'a> Iterator for EventIter<'a> {
    type Item = GpioEvents;

    fn next(&mut self) -> Option<Self::Item> {
        self.idx += 1;

        let gpiote = &mut self.manager.gpiote;
        match self.idx {
            1 => {
                if gpiote.channel0().is_event_triggered() {
                    Some(GpioEvents::Encoder)
                } else {
                    self.next()
                }
            }
            2 => {
                gpiote.channel0().clear();
                if gpiote.channel1().is_event_triggered() {
                    Some(GpioEvents::Sonar(SONAR_MAPPING[0]))
                } else {
                    self.next()
                }
            }
            3 => {
                gpiote.channel1().clear();
                if gpiote.channel2().is_event_triggered() {
                    Some(GpioEvents::Sonar(SONAR_MAPPING[1]))
                } else {
                    self.next()
                }
            }
            4 => {
                gpiote.channel2().clear();
                if gpiote.channel3().is_event_triggered() {
                    Some(GpioEvents::Sonar(SONAR_MAPPING[2]))
                } else {
                    self.next()
                }
            }
            5 => {
                gpiote.channel3().clear();
                gpiote.reset_events();
                // gpiote.port().reset_events();
                None
            }
            _ => {
                None
            }
        }
    }
}
