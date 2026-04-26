use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::twim;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Delay;

use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};

use super::traits::{BoardParts, LoRaBoard};
use crate::hal::nrf52840 as mcu;

// ── Concrete peripheral types ───────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, mcu::SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;
pub type UsbDriver = mcu::UsbNrfDriver;
pub type DisplayI2c = mcu::I2cBus;
pub type LedDriver = crate::driver::simple_led::SimpleLed<Output<'static>>;

// SH1106 hand-rolled driver instead of the ssd1306 crate. The RAK1921
// OLED ships under various Chinese-OEM packaging (e.g. JMD0.96C) that
// can be either SSD1306 or SH1106 silicon — physically and pin
// identical, but use different addressing modes. The bigger reason
// though: ssd1306 0.10's display-interface-i2c emits I2C transaction
// patterns (consecutive writes, 3+ ops) that embassy_nrf::twim's
// `unreachable!()` rejects, so even on real SSD1306 silicon the panic
// would fire. Our hand-rolled SH1106 driver issues only single writes
// per logical op and is twim-safe. SH1106 init opcodes are similar
// enough that real SSD1306 silicon mostly works (worst case: mirrored
// display until proper runtime detection lands).
pub type DisplayDriver = crate::driver::sh1106::Sh1106<DisplayI2c>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

pub struct UsbParts {
    pub driver: UsbDriver,
}

pub struct DisplayParts {
    pub i2c: DisplayI2c,
}

// ── Display init ────────────────────────────────────────────────────

pub async fn create_display(i2c: DisplayI2c) -> Option<DisplayDriver> {
    let mut display = crate::driver::sh1106::Sh1106::new(i2c, 0x3C);

    // Retry 3× with 100ms backoff. The SH1106's first transaction
    // after power-on can return Err if the chip's internal voltage
    // converter hasn't fully settled — observed empirically on
    // RAK4631/RAK1921. Each attempt is independent (full init
    // sequence). Display works visibly even if the FIRST attempt
    // failed, because subsequent attempts succeed.
    for attempt in 0..3u8 {
        embassy_time::Timer::after_millis(100).await;
        if display.init().await.is_ok() {
            let _ = display.set_brightness(0xFF).await;
            return Some(display);
        }
        if attempt < 2 {
            defmt::warn!(
                "SH1106 display init attempt {=u8}/3 failed; retrying",
                attempt + 1
            );
        }
    }
    defmt::error!("SH1106 display init failed after 3 attempts");
    None
}

// ── Board init ──────────────────────────────────────────────────────

pub struct Board {
    p: embassy_nrf::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "RAK WisBlock 4631";
    const TX_POWER_RANGE: (i8, i8) = (-9, 22);

    type RadioParts = RadioParts;
    type CommParts = UsbParts;
    type DisplayParts = DisplayParts;
    type DisplayDriver = DisplayDriver;
    type LedDriver = LedDriver;

    fn init() -> Self {
        // USB requires the 32 MHz HFXO (±50 ppm); the RC oscillator is out of spec.
        let mut config = embassy_nrf::config::Config::default();
        config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
        config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;
        let p = embassy_nrf::init(config);
        Self { p }
    }

    fn mac_address() -> [u8; 6] {
        mcu::mac_address()
    }

    fn into_parts(self) -> BoardParts<RadioParts, UsbParts, DisplayParts, LedDriver> {
        let p = self.p;

        // SX1262 is power-gated on P1.05 — must be HIGH before any SPI access.
        let sx_power_en = Output::new(p.P1_05, Level::High, OutputDrive::Standard);
        core::mem::forget(sx_power_en);

        // ── SPI bus for SX1262 ──────────────────────────────────
        let mut spi_cfg = spim::Config::default();
        spi_cfg.frequency = spim::Frequency::M1;
        // embassy_nrf::spim::Spim::new args: (spim, irq, sck, miso, mosi, config).
        // Per RAK4631 BSP: SCK=P1.11, MISO=P1.13, MOSI=P1.12.
        let spi = Spim::new(p.SPI3, mcu::Irqs, p.P1_11, p.P1_13, p.P1_12, spi_cfg);
        let spi_bus = mcu::share_spi_bus(spi);

        let nss = Output::new(p.P1_10, Level::High, OutputDrive::Standard);
        let spi_device = SpiDevice::new(spi_bus, nss);

        let reset = Output::new(p.P1_06, Level::High, OutputDrive::Standard);
        let dio1 = Input::new(p.P1_15, Pull::Down);
        // BUSY has an internal 20 kΩ pull-up on the SX1262 die (datasheet
        // p. 53); an MCU-side Pull::Down fights it and leaves the line at
        // an indeterminate voltage, making wait_on_busy unreliable. See
        // lora-rs/lora-rs#260.
        let busy = Input::new(p.P1_14, Pull::None);

        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None)
            .expect("SX1262 interface init");

        let sx_config = sx126x::Config {
            chip: Sx1262,
            tcxo_ctrl: Some(sx126x::TcxoCtrlVoltage::Ctrl1V8),
            use_dcdc: true,
            rx_boost: false,
        };

        let radio = RadioParts {
            driver: Sx126x::new(spi_device, iv, sx_config),
            delay: Delay,
        };

        // ── USB ─────────────────────────────────────────────────
        // SoftwareVbusDetect(vbus_detected, power_ready). Mark both true so the
        // driver doesn't block waiting for a power-ready signal we never raise.
        let vbus = mcu::alloc_vbus_detect(true, true);
        let host = UsbParts {
            driver: Driver::new(p.USBD, mcu::Irqs, vbus),
        };

        // ── Display (optional RAK1921 SSD1306 OLED on I2C) ─────
        let twim_buf = mcu::alloc_i2c_buffer();
        let i2c = embassy_nrf::twim::Twim::new(
            p.TWISPI0,
            mcu::Irqs,
            p.P0_13,
            p.P0_14,
            twim::Config::default(),
            twim_buf,
        );
        let display = Some(DisplayParts { i2c });

        // Green LED on P1.03 (RAK19007 Base, active HIGH).
        let led_pin = Output::new(p.P1_03, Level::Low, OutputDrive::Standard);
        let led = crate::driver::simple_led::SimpleLed(led_pin);

        BoardParts {
            radio,
            host,
            display,
            led,
            mac: Self::mac_address(),
        }
    }
}
