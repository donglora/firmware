//! Heltec WiFi LoRa 32 V3 — stock UART (via CP2102 bridge).
//!
//! This is for unmodified V3 boards where USB goes through the CP2102
//! bridge to UART0. The board appears as /dev/ttyUSB* (not ttyACM*).
//!
//! If you've done the hardware mod (solder R29/R3, disconnect CP2102),
//! use the `heltec_v3` feature instead for native USB CDC-ACM support.

use esp_hal::gpio::{Level, Output, OutputConfig};

use super::esp32s3;
use super::traits::{BoardParts, LoRaBoard};
use crate::hal::esp32s3 as mcu;

#[allow(unused_imports)] // Re-exported for other modules (display, radio, host tasks)
pub use super::esp32s3::{
    create_display, DisplayDriver, DisplayI2c, DisplayParts, RadioDriver, RadioParts, UartDriver,
    UartParts,
};

pub struct Board {
    p: esp_hal::peripherals::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "Heltec V3 (UART)";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    type RadioParts = RadioParts;
    type CommParts = UartParts;
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;

    fn init() -> Self {
        let p = esp_hal::init(esp_hal::Config::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }

    fn into_parts(self) -> BoardParts<RadioParts, UartParts, DisplayParts> {
        let p = self.p;

        mcu::start_timer(p.TIMG0);

        // Vext power: GPIO36, active LOW to enable peripherals
        let vext = Output::new(p.GPIO36, Level::Low, OutputConfig::default());
        core::mem::forget(vext);

        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_CH0, p.GPIO9, p.GPIO10, p.GPIO11);
        let radio = esp32s3::init_radio(spi_bus, p.GPIO8, p.GPIO12, p.GPIO14, p.GPIO13);
        let host = UartParts {
            driver: mcu::init_uart(p.UART0, p.GPIO43, p.GPIO44),
        };

        // SSD1306 display reset: pulse GPIO21 low->high before I2C init
        let mut display_rst = Output::new(p.GPIO21, Level::Low, OutputConfig::default());
        esp_hal::delay::Delay::new().delay_millis(10);
        display_rst.set_high();
        esp_hal::delay::Delay::new().delay_millis(10);
        core::mem::forget(display_rst);

        let i2c = mcu::init_i2c(p.I2C0, p.GPIO17, p.GPIO18);
        let display = Some(DisplayParts { i2c });

        BoardParts {
            radio,
            host,
            display,
            mac: Self::mac_address(),
        }
    }
}
