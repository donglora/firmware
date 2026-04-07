//! Simple single-color LED driver (GPIO on/off).
//!
//! Generic over any `OutputPin`. Any non-zero RGB value turns the LED on;
//! all zeros turns it off.

use embedded_hal::digital::OutputPin;

use crate::board::RgbLed;

pub struct SimpleLed<P>(pub P);

impl<P: OutputPin> RgbLed for SimpleLed<P> {
    async fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        if r > 0 || g > 0 || b > 0 {
            let _ = self.0.set_high();
        } else {
            let _ = self.0.set_low();
        }
    }
}
