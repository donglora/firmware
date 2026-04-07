//! WS2812B single-LED driver using the ESP32-S3 RMT peripheral.
//!
//! Encodes RGB values as GRB PulseCodes and transmits via RMT.
//! At 80 MHz with divider=1, each tick is 12.5 ns.

use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel, PulseCode, Tx};

use crate::board::RgbLed;

/// WS2812 0-bit: High 400ns (32 ticks), Low 850ns (68 ticks).
const BIT_0: PulseCode = PulseCode::new(Level::High, 32, Level::Low, 68);

/// WS2812 1-bit: High 800ns (64 ticks), Low 450ns (36 ticks).
const BIT_1: PulseCode = PulseCode::new(Level::High, 64, Level::Low, 36);

/// WS2812B single-LED driver over RMT.
pub struct Ws2812 {
    channel: Channel<'static, esp_hal::Async, Tx>,
}

impl Ws2812 {
    pub fn new(channel: Channel<'static, esp_hal::Async, Tx>) -> Self {
        Self { channel }
    }

    /// Encode 3 bytes (GRB order) + end marker into PulseCodes.
    fn encode(r: u8, g: u8, b: u8) -> [PulseCode; 25] {
        let mut data = [PulseCode::end_marker(); 25];
        let bytes = [g, r, b]; // WS2812 expects GRB order
        let mut idx = 0;
        for byte in bytes {
            for bit in (0..8).rev() {
                data[idx] = if byte & (1 << bit) != 0 { BIT_1 } else { BIT_0 };
                idx += 1;
            }
        }
        // data[24] is already end_marker
        data
    }
}

impl RgbLed for Ws2812 {
    async fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        let data = Self::encode(r, g, b);
        let _ = self.channel.transmit(&data).await;
    }
}
