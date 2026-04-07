use esp_hal::gpio::{Level, Output, OutputConfig};
use static_cell::StaticCell;

use super::esp32s3;
use super::traits::{BoardParts, LoRaBoard};
use super::esp32s3::SimpleLed;
use crate::hal::esp32s3 as mcu;

pub use super::esp32s3::{
    create_display, DisplayDriver, DisplayParts, RadioDriver, RadioParts,
};

pub type UsbDriver = esp_hal::otg_fs::asynch::Driver<'static>;

pub type LedDriver = SimpleLed;

pub struct UsbParts {
    pub driver: UsbDriver,
}

pub struct Board {
    p: esp_hal::peripherals::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "Heltec V4";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    type RadioParts = RadioParts;
    type CommParts = UsbParts;
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;
    type LedDriver = LedDriver;

    fn init() -> Self {
        let p = esp_hal::init(esp_hal::Config::default());
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }

    fn into_parts(self) -> BoardParts<RadioParts, UsbParts, DisplayParts, LedDriver> {
        let p = self.p;

        mcu::start_timer(p.TIMG0);

        // Vext power: GPIO36, active LOW to enable peripherals
        let vext = Output::new(p.GPIO36, Level::Low, OutputConfig::default());
        core::mem::forget(vext);

        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_CH0, p.GPIO9, p.GPIO10, p.GPIO11);
        let radio = esp32s3::init_radio(spi_bus, p.GPIO8, p.GPIO12, p.GPIO14, p.GPIO13);

        // Note: switches internal USB PHY from Serial-JTAG to OTG.
        let usb_inst = esp_hal::otg_fs::Usb::new(p.USB0, p.GPIO20, p.GPIO19);
        static EP_OUT_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
        let ep_out_buf = EP_OUT_BUF.init([0u8; 1024]);
        let host = UsbParts {
            driver: esp_hal::otg_fs::asynch::Driver::new(
                usb_inst,
                ep_out_buf,
                esp_hal::otg_fs::asynch::Config::default(),
            ),
        };

        // SSD1315 display reset: pulse GPIO21 low->high before I2C init
        let mut display_rst = Output::new(p.GPIO21, Level::Low, OutputConfig::default());
        esp_hal::delay::Delay::new().delay_millis(10);
        display_rst.set_high();
        esp_hal::delay::Delay::new().delay_millis(10);
        core::mem::forget(display_rst);

        let i2c = mcu::init_i2c(p.I2C0, p.GPIO17, p.GPIO18);
        let display = Some(DisplayParts { i2c });

        // Orange LED on GPIO35
        let led_pin = Output::new(p.GPIO35, Level::Low, OutputConfig::default());
        let led = Some(SimpleLed(led_pin));

        BoardParts {
            radio,
            host,
            display,
            led,
            mac: Self::mac_address(),
        }
    }
}
