//! Board abstraction trait.
//!
//! Every board must implement [`LoRaBoard`] with concrete associated types
//! for its radio, host communication, display, and LED peripherals. The
//! compiler enforces completeness — missing types or methods are compile errors.
//!
//! See `src/board/PORTING.md` for a step-by-step guide to adding a new board.

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::BinaryColor;

use crate::driver::DisplayBrightness;

/// Async RGB LED control. Boards without an LED use `()` which is a no-op.
pub trait RgbLed {
    fn set_rgb(&mut self, r: u8, g: u8, b: u8) -> impl core::future::Future<Output = ()>;
}

impl RgbLed for () {
    async fn set_rgb(&mut self, _r: u8, _g: u8, _b: u8) {}
}

/// Compile-time contract for a DongLoRa board.
pub trait LoRaBoard: Sized {
    /// Human-readable board name (shown on display splash screen).
    const NAME: &'static str;

    /// TX power range in dBm (min, max) for this board's radio + PA.
    const TX_POWER_RANGE: (i8, i8);

    /// Radio peripheral bundle (driver + delay).
    type RadioParts;

    /// Host communication peripheral bundle (USB or UART driver).
    type CommParts;

    /// Display peripheral bundle (I2C bus for display init).
    type DisplayParts;

    /// Concrete display driver type (must implement DrawTarget for rendering
    /// and DisplayBrightness for the two-level dim control).
    type DisplayDriver: DrawTarget<Color = BinaryColor> + DisplayBrightness;

    /// RGB LED driver. Boards without an LED use `()`.
    type LedDriver: RgbLed;

    /// Initialize the board hardware.
    fn init() -> Self;

    /// Read the board's unique hardware address (MAC, device ID, etc.).
    fn mac_address() -> [u8; 6];

    /// Radio chip identifier reported in `GET_INFO`. Defaults to SX1262
    /// (`0x0002`), matching every currently-supported board. Override for
    /// boards that carry a different chip (LLCC68, SX1261, LR11xx, …).
    fn radio_chip_id() -> u16 {
        // SX1262 — see PROTOCOL.md §8.
        0x0002
    }

    /// (freq_min_hz, freq_max_hz) — the RF front-end's effective tuning
    /// window on this specific board. Reported in `GET_INFO`. Defaults
    /// to the SX1262's nominal 150–960 MHz range.
    fn freq_range_hz() -> (u32, u32) {
        (150_000_000, 960_000_000)
    }

    /// Decompose initialized board into peripheral bundles for each task.
    fn into_parts(
        self,
    ) -> BoardParts<Self::RadioParts, Self::CommParts, Self::DisplayParts, Self::LedDriver>;
}

/// Peripheral bundles for each firmware task, produced by [`LoRaBoard::into_parts`].
pub struct BoardParts<R, C, D, L> {
    pub radio: R,
    pub host: C,
    pub display: Option<D>,
    pub led: L,
    pub mac: [u8; 6],
}
