//! Simple single-color LED driver (GPIO on/off).
//!
//! For boards with a plain LED (not addressable RGB). Any non-zero
//! RGB value turns the LED on; all zeros turns it off.

use esp_hal::gpio::Output;

use crate::board::RgbLed;

pub struct SimpleLed {
    pin: Output<'static>,
}

impl SimpleLed {
    pub fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }
}

impl RgbLed for SimpleLed {
    async fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        if r > 0 || g > 0 || b > 0 {
            self.pin.set_high();
        } else {
            self.pin.set_low();
        }
    }
}
