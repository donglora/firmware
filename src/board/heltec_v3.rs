use esp_hal::gpio::{Level, Output, OutputConfig};

use super::esp32s3;
use super::traits::LoRaBoard;

#[allow(unused_imports)] // Re-exported for other modules (display, radio, usb tasks)
pub use super::esp32s3::{DisplayI2c, DisplayParts, RadioDriver, RadioParts, UsbDriver, UsbParts};

// ── Board init ───────────────────────────────────────────────────────

pub struct Board {
    p: esp_hal::peripherals::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "Heltec V3";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    fn init() -> Self {
        let p = esp_hal::init(esp_hal::Config::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        esp_hal::efuse::Efuse::mac_address()
    }
}

impl Board {
    pub fn into_parts(self) -> (RadioParts, UsbParts, Option<DisplayParts>) {
        let p = self.p;

        esp32s3::start_timer(p.TIMG0);

        // Vext power: GPIO21, active LOW to enable OLED
        let vext = Output::new(p.GPIO21, Level::Low, OutputConfig::default());
        core::mem::forget(vext); // hold pin low permanently; drop would reset it

        let radio = esp32s3::init_radio(
            p.SPI2, p.DMA_CH0, p.GPIO9, p.GPIO10, p.GPIO11, p.GPIO8, p.GPIO12, p.GPIO14,
            p.GPIO13,
        );
        let usb = esp32s3::init_usb(p.USB0, p.GPIO20, p.GPIO19);
        let i2c = esp32s3::init_display_i2c(p.I2C0, p.GPIO17, p.GPIO18);
        let mac = Self::mac_address();
        let display = Some(DisplayParts { i2c, mac });

        (radio, usb, display)
    }
}
