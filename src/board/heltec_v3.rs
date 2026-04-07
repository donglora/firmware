use esp_hal::gpio::{Level, Output, OutputConfig};

use super::esp32s3;
use super::traits::LoRaBoard;
use crate::hal::esp32s3 as mcu;

#[allow(unused_imports)] // Re-exported for other modules (display, radio, uart tasks)
pub use super::esp32s3::{DisplayI2c, DisplayParts, RadioDriver, RadioParts, UartDriver, UartParts};

// ── Board init ──────────────────────────────────────────────────────

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
        mcu::mac_address()
    }
}

impl Board {
    pub fn into_parts(self) -> (RadioParts, UartParts, Option<DisplayParts>) {
        let p = self.p;

        mcu::start_timer(p.TIMG0);

        // Vext power: GPIO36, active LOW to enable peripherals
        let vext = Output::new(p.GPIO36, Level::Low, OutputConfig::default());
        core::mem::forget(vext); // hold pin low permanently; drop would reset it

        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_CH0, p.GPIO9, p.GPIO10, p.GPIO11);
        let radio = esp32s3::init_radio(spi_bus, p.GPIO8, p.GPIO12, p.GPIO14, p.GPIO13);
        let uart = UartParts {
            driver: mcu::init_uart(p.UART0, p.GPIO43, p.GPIO44),
        };

        // SSD1306 display reset: pulse GPIO21 low->high before I2C init
        let mut display_rst = Output::new(p.GPIO21, Level::Low, OutputConfig::default());
        esp_hal::delay::Delay::new().delay_millis(10);
        display_rst.set_high();
        esp_hal::delay::Delay::new().delay_millis(10);
        core::mem::forget(display_rst); // hold reset high permanently

        let i2c = mcu::init_i2c(p.I2C0, p.GPIO17, p.GPIO18);
        let mac = Self::mac_address();
        let display = Some(DisplayParts { i2c, mac });

        (radio, uart, display)
    }
}
