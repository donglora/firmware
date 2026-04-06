//! Minimal async SH1106 I2C OLED driver (128x64, page addressing).
//!
//! The SH1106 is very similar to the SSD1306 but only supports page-mode
//! addressing (no horizontal/vertical auto-advance). This driver sends
//! framebuffer data page-by-page with explicit page/column commands.

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::{OriginDimensions, Size};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::Pixel;
use embedded_hal_async::i2c::I2c;

const WIDTH: usize = 128;
const HEIGHT: usize = 64;
const PAGES: usize = HEIGHT / 8;
const BUF_SIZE: usize = WIDTH * PAGES; // 1024

/// SH1106 has a 132-column RAM; visible 128 columns are centered with a
/// 2-column offset on each side.
const COL_OFFSET: u8 = 2;

pub struct Sh1106<I> {
    i2c: I,
    addr: u8,
    buffer: [u8; BUF_SIZE],
}

impl<I: I2c> Sh1106<I> {
    pub fn new(i2c: I, addr: u8) -> Self {
        Self {
            i2c,
            addr,
            buffer: [0u8; BUF_SIZE],
        }
    }

    /// Send a sequence of commands (each prefixed with the I2C command byte 0x00).
    async fn cmd(&mut self, commands: &[u8]) -> Result<(), I::Error> {
        for &c in commands {
            self.i2c.write(self.addr, &[0x00, c]).await?;
        }
        Ok(())
    }

    /// Initialize the display with sensible defaults.
    pub async fn init(&mut self) -> Result<(), I::Error> {
        self.cmd(&[
            0xAE, // display off
            0xD5, 0x80, // clock div: default
            0xA8, 0x3F, // multiplex ratio: 64
            0xD3, 0x00, // display offset: 0
            0x40, // start line: 0
            0x8D, 0x14, // charge pump: enable
            0xA1, // segment remap: column 127 mapped to SEG0
            0xC8, // COM scan direction: remapped
            0xDA, 0x12, // COM pin config: alternative, no L/R remap
            0x81, 0xCF, // contrast: 207
            0xD9, 0xF1, // pre-charge: phase1=1, phase2=15
            0xDB, 0x40, // VCOMH deselect: ~0.77×Vcc
            0xA4, // entire display ON (follow RAM)
            0xA6, // normal display (not inverted)
            0xAF, // display on
        ])
        .await
    }

    /// Flush the entire framebuffer to the display, page by page.
    pub async fn flush(&mut self) -> Result<(), I::Error> {
        // Each I2C data write is prefixed with 0x40 (data continuation byte).
        let mut page_buf = [0u8; 1 + WIDTH]; // 0x40 + 128 data bytes
        page_buf[0] = 0x40;

        for page in 0..PAGES {
            // Set page address and column start (with SH1106 2-column offset).
            self.cmd(&[
                0xB0 | page as u8,          // page address
                0x00 | (COL_OFFSET & 0x0F), // column low nibble
                0x10 | (COL_OFFSET >> 4),   // column high nibble
            ])
            .await?;

            let start = page * WIDTH;
            page_buf[1..].copy_from_slice(&self.buffer[start..start + WIDTH]);
            self.i2c.write(self.addr, &page_buf).await?;
        }
        Ok(())
    }

    /// Set display brightness (contrast).
    pub async fn set_brightness(&mut self, value: u8) -> Result<(), I::Error> {
        self.cmd(&[0x81, value]).await
    }

    /// Clear the framebuffer (call flush() to update the display).
    pub fn clear_buffer(&mut self) {
        self.buffer.fill(0);
    }
}

impl<I: I2c> DrawTarget for Sh1106<I> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<T>(&mut self, pixels: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(pos, color) in pixels {
            let x = pos.x;
            let y = pos.y;
            if x >= 0 && x < WIDTH as i32 && y >= 0 && y < HEIGHT as i32 {
                let x = x as usize;
                let y = y as usize;
                let idx = (y / 8) * WIDTH + x;
                let bit = (y % 8) as u8;
                if color.is_on() {
                    self.buffer[idx] |= 1 << bit;
                } else {
                    self.buffer[idx] &= !(1 << bit);
                }
            }
        }
        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        self.buffer.fill(if color.is_on() { 0xFF } else { 0x00 });
        Ok(())
    }
}

impl<I: I2c> OriginDimensions for Sh1106<I> {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}
