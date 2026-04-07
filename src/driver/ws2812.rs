//! WS2812B single-LED driver using the ESP32-S3 RMT peripheral.
//!
//! Encodes RGB values as GRB PulseCodes and transmits via RMT.
//! At 80 MHz with divider=1, each tick is 12.5 ns.
//! Uses blocking transmit (~31µs per LED update).

use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel, PulseCode, Tx};

use crate::board::RgbLed;

/// WS2812 0-bit: High 400ns (32 ticks), Low 850ns (68 ticks).
const BIT_0: PulseCode = PulseCode::new(Level::High, 32, Level::Low, 68);

/// WS2812 1-bit: High 800ns (64 ticks), Low 450ns (36 ticks).
const BIT_1: PulseCode = PulseCode::new(Level::High, 64, Level::Low, 36);

/// WS2812B single-LED driver over RMT (blocking).
pub struct Ws2812 {
    channel: Option<Channel<'static, esp_hal::Blocking, Tx>>,
}

impl Ws2812 {
    pub fn new(channel: Channel<'static, esp_hal::Blocking, Tx>) -> Self {
        Self {
            channel: Some(channel),
        }
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
        data
    }
}

impl RgbLed for Ws2812 {
    async fn set_rgb(&mut self, r: u8, g: u8, b: u8) {
        let data = Self::encode(r, g, b);
        if let Some(ch) = self.channel.take() {
            match ch.transmit(&data) {
                Ok(tx) => match tx.wait() {
                    Ok(ch) => self.channel = Some(ch),
                    Err((e, ch)) => {
                        defmt::warn!("WS2812 wait error: {}", e);
                        self.channel = Some(ch);
                    }
                },
                Err(e) => {
                    defmt::warn!("WS2812 transmit error: {}", e);
                    // Channel consumed on transmit error — LED is lost
                }
            }
        }
    }
}
