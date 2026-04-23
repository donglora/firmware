#[cfg(any(feature = "wio_tracker_l1", feature = "elecrow_thinknode_m2"))]
pub mod sh1106;

#[cfg(any(
    feature = "heltec_v3",
    feature = "heltec_v3_uart",
    feature = "heltec_v4",
    feature = "elecrow_thinknode_m2",
    feature = "wio_tracker_l1",
    feature = "waveshare_rp2040_lora"
))]
pub mod simple_led;

// ── Display brightness ────────────────────────────────────────────────
//
// The display statechart dims the OLED a few seconds after the last
// activity. Our two driver families (`ssd1306` crate and our in-tree
// `sh1106`) expose incompatible brightness APIs, so we wrap them in a
// single async trait that the state-machine actions can call uniformly.

/// Two-level backlight control used by the display dashboard.
pub trait DisplayBrightness {
    fn set_bright(&mut self) -> impl core::future::Future<Output = ()>;
    fn set_dim(&mut self) -> impl core::future::Future<Output = ()>;
}

// Contrast values for the two brightness levels, picked to be common across
// both driver families. `0xFF` is the SSD1306/SH1106 maximum; `0x08` is ~3%
// of max — visible in a dim room, barely noticeable in daylight. Tune here.
const CONTRAST_BRIGHT: u8 = 0xFF;
const CONTRAST_DIM: u8 = 0x08;

impl<DI, SIZE, MODE> DisplayBrightness for ssd1306::Ssd1306Async<DI, SIZE, MODE>
where
    DI: display_interface::AsyncWriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySizeAsync,
{
    async fn set_bright(&mut self) {
        // `Brightness::custom(precharge, contrast)`. Matches the shape of
        // the crate's BRIGHTEST preset (precharge 0x2) at max contrast.
        let _ = self
            .set_brightness(ssd1306::prelude::Brightness::custom(0x2, CONTRAST_BRIGHT))
            .await;
    }
    async fn set_dim(&mut self) {
        // Low precharge pairs with low contrast — same combination the
        // crate's DIMMEST preset uses.
        let _ = self
            .set_brightness(ssd1306::prelude::Brightness::custom(0x1, CONTRAST_DIM))
            .await;
    }
}

#[cfg(any(feature = "wio_tracker_l1", feature = "elecrow_thinknode_m2"))]
impl<I> DisplayBrightness for sh1106::Sh1106<I>
where
    I: embedded_hal_async::i2c::I2c,
{
    async fn set_bright(&mut self) {
        let _ = self.set_brightness(CONTRAST_BRIGHT).await;
    }
    async fn set_dim(&mut self) {
        let _ = self.set_brightness(CONTRAST_DIM).await;
    }
}
