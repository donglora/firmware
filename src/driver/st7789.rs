//! ST7789 240×135 IPS TFT driver wrapper for the Heltec Mesh Node T114.
//!
//! Wraps `lcd_async::Display<…, models::ST7789, …>` (async fork of
//! mipidsi) with an in-RAM raw byte framebuffer (240×135 × 2 bytes
//! Rgb565 BE = 64,800 bytes) and an active-LOW PWM backlight on PWM0.
//!
//! Renderers draw into the framebuffer via `lcd_async::raw_framebuf::RawFrameBuf`;
//! `present()` awaits a single async DMA blit of the entire buffer to
//! the panel via `panel.show_raw_data(...).await`. The await lets
//! embassy-nrf's DMA-backed Spim yield the executor for the duration
//! of the ~65 ms SPI transfer, so the radio task can run alongside
//! and the SX1262 IRQ handler isn't starved during repaints.

use embassy_nrf::pwm::{DutyCycle, SimplePwm};
use embedded_graphics::pixelcolor::Rgb565;
use lcd_async::interface::Interface;
use lcd_async::models::ST7789;
use lcd_async::raw_framebuf::RawFrameBuf;
use lcd_async::Display;

use crate::display::{render_color, BoardDisplay, BoardInfo, DashboardCtx};

/// PWM countertop. Drives the backlight at ~1 kHz (well above flicker).
pub const BL_MAX: u16 = 1000;

fn bl_bright() -> DutyCycle {
    // Inverted polarity: HIGH for 0 counts → always LOW → full bright.
    DutyCycle::inverted(0)
}

fn bl_dim() -> DutyCycle {
    // ~80% HIGH → ~20% LOW → 20% brightness.
    DutyCycle::inverted((BL_MAX * 4) / 5)
}

#[allow(dead_code)]
fn bl_off() -> DutyCycle {
    DutyCycle::inverted(BL_MAX)
}

// ── Framebuffer ───────────────────────────────────────────────────────

pub const FB_W: usize = 240;
pub const FB_H: usize = 135;
/// Bytes for one full Rgb565 frame (2 bytes/pixel, big-endian — matches
/// what ST7789 expects in 16-bit color mode).
pub const FB_BYTES: usize = FB_W * FB_H * 2;

// ── St7789Color ───────────────────────────────────────────────────────

pub struct St7789Color<DI, RST>
where
    DI: Interface,
    RST: embedded_hal::digital::OutputPin,
{
    panel: Display<DI, ST7789, RST>,
    backlight: SimplePwm<'static>,
    fb: &'static mut [u8; FB_BYTES],
}

impl<DI, RST> St7789Color<DI, RST>
where
    DI: Interface,
    RST: embedded_hal::digital::OutputPin,
{
    pub fn new(
        panel: Display<DI, ST7789, RST>,
        backlight: SimplePwm<'static>,
        fb: &'static mut [u8; FB_BYTES],
    ) -> Self {
        Self { panel, backlight, fb }
    }

    /// Wrap the raw byte buffer as a `RawFrameBuf<Rgb565, _>` so
    /// `embedded-graphics` primitives can draw into it, then run `f`.
    /// The borrow ends when this returns, freeing `self.fb` for the
    /// subsequent blit in `present()`.
    fn with_fb<F: FnOnce(&mut RawFrameBuf<Rgb565, &mut [u8]>)>(&mut self, f: F) {
        let mut fbuf = RawFrameBuf::<Rgb565, _>::new(&mut self.fb[..], FB_W, FB_H);
        f(&mut fbuf);
    }
}

impl<DI, RST> BoardDisplay for St7789Color<DI, RST>
where
    DI: Interface<Word = u8>,
    RST: embedded_hal::digital::OutputPin,
{
    /// Blit the entire framebuffer to the panel via the async SPI
    /// interface. embassy-nrf's Spim runs the transfer over EasyDMA;
    /// the `.await` yields the executor for the duration so the radio
    /// task can process SX1262 IRQs concurrently.
    async fn present(&mut self) {
        let _ = self
            .panel
            .show_raw_data(0, 0, FB_W as u16, FB_H as u16, self.fb.as_ref())
            .await;
    }

    async fn set_bright(&mut self) {
        self.backlight.set_duty(0, bl_bright());
    }

    async fn set_dim(&mut self) {
        self.backlight.set_duty(0, bl_dim());
    }

    fn render_splash_learn_more(&mut self) {
        self.with_fb(|fb| render_color::splash_learn_more(fb));
    }

    fn render_splash_info(&mut self, info: &BoardInfo<'_>) {
        self.with_fb(|fb| render_color::splash_info(fb, info));
    }

    fn render_dashboard(&mut self, ctx: &DashboardCtx<'_>) {
        self.with_fb(|fb| render_color::dashboard(fb, ctx));
    }

    fn render_blank(&mut self) {
        self.with_fb(|fb| render_color::blank(fb));
    }
}
