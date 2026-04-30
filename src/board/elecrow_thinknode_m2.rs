//! ELECROW ThinkNode-M2 — ESP32-S3 Meshtastic handset with SH1106 OLED.
//!
//! Tamagotchi-form-factor handheld: ESP32-S3-WROOM-1-N4 + SX1262 + 1.3" SH1106
//! OLED + WCH CH340K UART bridge + 1000 mAh Li-Po. Sold pre-loaded with
//! Meshtastic; this firmware turns it into a DongLoRa radio bridge.
//!
//! Pin map (verified from Meshtastic's variants/esp32s3/ELECROW-ThinkNode-M2
//! and PR meshtastic/firmware#6354):
//!   LoRa SPI: SCK=12, MOSI=11, MISO=13, CS=10
//!   LoRa ctl: RESET=21, BUSY=14, DIO1=3
//!   LoRa pwr: GPIO48 (must be HIGH before any SPI traffic)
//!   OLED:     I2C0 SDA=16, SCL=15, SH1106 @ 0x3C
//!   VEXT:     GPIO46 (active HIGH — gates OLED supply)
//!   UART:     TX=43, RX=44 via CH340K bridge (USB 1a86:7522)
//!   LED:      GPIO6 (active HIGH indicator)
//!
//! Notable differences from the Heltec ESP32-S3 boards:
//!   * Display is SH1106 (not SSD1306) — uses the existing sh1106 driver.
//!   * VEXT_ENABLE polarity is reversed (active HIGH here, active LOW on Heltec).
//!   * Dedicated LoRa power enable on GPIO48 — must be sequenced before SPI init.
//!   * TCXO is driven at 3.3 V via DIO3 (Heltec uses 1.8 V).

use esp_hal::gpio::{Level, Output, OutputConfig};

use super::esp32s3;
use super::traits::{BoardParts, LoRaBoard};
use crate::driver::sh1106::Sh1106;
use crate::driver::simple_led::SimpleLed;
use crate::hal::esp32s3 as mcu;

pub use super::esp32s3::{DisplayI2c, RadioDriver, RadioParts};

// ── Concrete peripheral types ───────────────────────────────────────

pub type DisplayDriver = Sh1106<DisplayI2c>;
pub type UartDriver = esp_hal::uart::Uart<'static, esp_hal::Async>;
pub type LedDriver = SimpleLed<Output<'static>>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct UartParts {
    pub driver: UartDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
}

// ── Display init ────────────────────────────────────────────────────

pub async fn create_display(parts: DisplayParts) -> Option<DisplayDriver> {
    let mut display = Sh1106::new(parts.i2c, 0x3C);

    embassy_time::Timer::after_millis(100).await;
    if display.init().await.is_err() {
        defmt::error!("SH1106 display init failed");
        return None;
    }
    let _ = display.set_brightness(0xFF).await;
    Some(display)
}

// ── Board init ──────────────────────────────────────────────────────

pub struct Board {
    p: esp_hal::peripherals::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "ELECROW ThinkNode-M2";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    type RadioParts = RadioParts;
    type CommParts = UartParts;
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

    fn into_parts(self) -> BoardParts<RadioParts, UartParts, DisplayParts, LedDriver> {
        let p = self.p;

        mcu::start_timer(p.TIMG0, p.SW_INTERRUPT);

        // VEXT_ENABLE: GPIO46, active HIGH — powers OLED rail (opposite of Heltec).
        // Output has no Drop impl, so the pin stays HIGH for program lifetime.
        let _vext = Output::new(p.GPIO46, Level::High, OutputConfig::default());
        esp_hal::delay::Delay::new().delay_millis(10);

        // LORA_POWER_EN: GPIO48, active HIGH — must be HIGH before any SPI traffic.
        let _lora_pwr = Output::new(p.GPIO48, Level::High, OutputConfig::default());
        esp_hal::delay::Delay::new().delay_millis(10);

        // SPI2 for SX1262: SCK=12, MOSI=11, MISO=13
        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_CH0, p.GPIO12, p.GPIO11, p.GPIO13);

        // SX1262: CS=10, RESET=21, DIO1=3, BUSY=14, TCXO driven at 3.3 V via DIO3.
        let radio = esp32s3::init_radio(
            spi_bus,
            p.GPIO10,
            p.GPIO21,
            p.GPIO3,
            p.GPIO14,
            lora_phy::sx126x::TcxoCtrlVoltage::Ctrl3V3,
        );

        // UART0 via CP2102 bridge: TX=43, RX=44
        use esp_hal::uart::{Config as UartConfig, Uart};
        let host = UartParts {
            driver: Uart::new(p.UART0, UartConfig::default())
                .expect("UART init")
                .with_tx(p.GPIO43)
                .with_rx(p.GPIO44)
                .into_async(),
        };

        // I2C0 for SH1106 OLED: SDA=16, SCL=15
        let i2c = mcu::init_i2c(p.I2C0, p.GPIO16, p.GPIO15);
        let display = Some(DisplayParts { i2c });

        // LED on GPIO6 (active HIGH)
        let led_pin = Output::new(p.GPIO6, Level::Low, OutputConfig::default());
        let led = SimpleLed(led_pin);

        BoardParts {
            radio,
            host,
            display,
            led,
            mac: Self::mac_address(),
        }
    }
}
