//! LilyGo T-Beam S3 Supreme — ESP32-S3 + SX1262 + SH1106 OLED + AXP2101 PMIC.
//!
//! Big-brother of the classic T-Beam: ESP32-S3-WROOM-1 module, SX1262 with a
//! TCXO, 128×64 SH1106 OLED, AXP2101 PMIC on a dedicated I²C1 bus. Native USB
//! (no UART bridge chip).
//!
//! The AXP2101 gates power to the SX1262 and OLED. Without its init sequence
//! the LoRa chip never responds on SPI and `LoRa::new` fails.
//!
//! Pin map (verified from MeshCore variants/lilygo_tbeam_supreme_SX1262/):
//!   LoRa SPI: SCK=12, MOSI=11, MISO=13, CS=10
//!   LoRa ctl: RESET=5, DIO1=1, BUSY=4 (TCXO @ 1.6 V via DIO3)
//!   OLED I²C: SDA=17, SCL=18, SH1106 @ 0x3C (I²C0)
//!   PMIC I²C: SDA=42, SCL=41, AXP2101 @ 0x34 (I²C1, blocking, PMIC-only)
//!   USB:      native USB-C on GPIO19/20 (fixed ESP32-S3 D+/D-)
//!   LED:      GPIO6 (active HIGH indicator, reuses MeshCore's P_LORA_TX_LED)
//!
//! GPS / battery / IMU / SD / BME280 are unused — DongLoRa is a transparent
//! LoRa-over-USB bridge. The PMIC init only enables ALDO1 (OLED) and ALDO3
//! (LoRa); other rails are left at their AXP2101 power-on defaults.

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use static_cell::StaticCell;

use super::esp32s3;
use super::traits::{BoardParts, LoRaBoard};
use crate::driver::axp;
use crate::driver::sh1106::Sh1106;
use crate::driver::simple_led::SimpleLed;
use crate::hal::esp32s3 as mcu;

pub use super::esp32s3::{DisplayI2c, RadioDriver, RadioParts};

// ── Concrete peripheral types ───────────────────────────────────────

pub type DisplayDriver = Sh1106<DisplayI2c>;
pub type UsbDriver = esp_hal::otg_fs::asynch::Driver<'static>;
pub type LedDriver = SimpleLed<Output<'static>>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct UsbParts {
    pub driver: UsbDriver,
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
    const NAME: &'static str = "LilyGo T-Beam S3 Supreme";
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

        mcu::start_timer(p.TIMG0, p.SW_INTERRUPT);

        // AXP2101 PMIC on I²C1 (SDA=42, SCL=41). Blocking, one-shot: we only
        // need to flip the LoRa/OLED rails on, then the bus can be dropped.
        // If this fails the LoRa chip stays unpowered — log and keep going so
        // the rest of the boot path can still produce diagnostics.
        {
            let mut pmic = I2c::new(p.I2C1, I2cConfig::default())
                .expect("I2C1 init")
                .with_sda(p.GPIO42)
                .with_scl(p.GPIO41);
            if let Err(_e) = axp::axp2101_init_lora_oled(&mut pmic) {
                defmt::error!("AXP2101 init failed — LoRa rail may be dead");
            }
        }
        // Let ALDO1/ALDO3 settle before touching the LoRa SPI or OLED I²C.
        esp_hal::delay::Delay::new().delay_millis(20);

        // SPI2 for SX1262: SCK=12, MOSI=11, MISO=13
        let spi_bus = mcu::init_spi(p.SPI2, p.DMA_CH0, p.GPIO12, p.GPIO11, p.GPIO13);

        // SX1262: CS=10, RESET=5, DIO1=1, BUSY=4, TCXO @ 1.6 V via DIO3.
        let radio = esp32s3::init_radio(
            spi_bus,
            p.GPIO10,
            p.GPIO5,
            p.GPIO1,
            p.GPIO4,
            lora_phy::sx126x::TcxoCtrlVoltage::Ctrl1V6,
        );

        // Native USB-C on GPIO19/20. Switches the internal USB PHY from
        // Serial-JTAG to OTG.
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

        // I2C0 for SH1106 OLED: SDA=17, SCL=18 (async, handed to display task).
        let i2c = mcu::init_i2c(p.I2C0, p.GPIO17, p.GPIO18);
        let display = Some(DisplayParts { i2c });

        // Activity LED on GPIO6 (active HIGH).
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
