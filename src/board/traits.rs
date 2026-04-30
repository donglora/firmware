//! Board abstraction trait.
//!
//! Every board must implement [`LoRaBoard`] with concrete associated types
//! for its radio, host communication, display, and LED peripherals. The
//! compiler enforces completeness — missing types or methods are compile errors.
//!
//! See `src/board/PORTING.md` for a step-by-step guide to adding a new board.

use crate::display::BoardDisplay;

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

    /// Concrete display driver type. Each board's driver implements
    /// [`BoardDisplay`] to render the splash + dashboard content into its
    /// native pixel format and resolution and to flush / dim the panel.
    type DisplayDriver: BoardDisplay;

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

    /// Bitmap of supported LoRa spreading factors — bit N = SF N. Defaults
    /// to SX126x's SF5–SF12 (`0x1FE0`). SX127x boards must override to
    /// `0x1FC0` (SF6–SF12); LLCC68 boards to `0x0FE0` (SF5–SF11).
    fn supported_sf_bitmap() -> u16 {
        0x1FE0
    }

    /// Bitmap of supported LoRa bandwidths. Bit positions match
    /// `LoRaBandwidth::as_u8()` (sub-GHz BW enum values 0..9). Defaults
    /// to all sub-GHz BWs (`0x03FF`). SX128x boards flip the 2.4 GHz
    /// bits when we add them.
    fn supported_bw_bitmap() -> u16 {
        0x03FF
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
