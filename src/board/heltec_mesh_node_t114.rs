//! Heltec Mesh Node T114 (nRF52840 + SX1262 + 1.14" ST7789 TFT).
//!
//! Hardware reference: cross-verified across MeshCore, Meshtastic, and
//! RNode firmwares. Pin maps quoted from
//! `meshcore-spec/references/MeshCore/variants/heltec_t114/variant.h` and
//! `meshtastic/firmware/variants/nrf52840/heltec_mesh_node_t114/variant.h`.
//!
//! Stage 2: panel comes up with static colors via the `St7789Color`
//! driver. Stage 3 swaps in the proper color renderer module.

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::pwm::{Prescaler, SimpleConfig, SimplePwm};
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Delay, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;
use lcd_async::interface::SpiInterface;
use lcd_async::models::ST7789;
use lcd_async::options::{ColorInversion, Orientation, Rotation};
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::sx126x::{self, Sx1262, Sx126x};
use static_cell::ConstStaticCell;

use super::traits::{BoardParts, LoRaBoard};
use crate::driver::inverted_pin::InvertedPin;
use crate::driver::simple_led::SimpleLed;
use crate::driver::st7789::{St7789Color, FB_BYTES};
use crate::hal::nrf52840 as mcu;

// ── Concrete peripheral types ───────────────────────────────────────

type Nss = Output<'static>;
type Iv = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioSpiDevice = SpiDevice<'static, NoopRawMutex, mcu::SpiBus, Nss>;
pub type RadioDriver = Sx126x<RadioSpiDevice, Iv, Sx1262>;
pub type UsbDriver = mcu::UsbNrfDriver;
// Green LED on P1.03, active LOW.
pub type LedDriver = SimpleLed<InvertedPin<Output<'static>>>;

// ── TFT bus and driver type aliases ─────────────────────────────────

type TftSpim = Spim<'static>;
type TftSpiDevice = ExclusiveDevice<TftSpim, Output<'static>, Delay>;
type TftInterface = SpiInterface<TftSpiDevice, Output<'static>>;
pub type DisplayDriver = St7789Color<TftInterface, Output<'static>>;

// ── Peripheral bundles ──────────────────────────────────────────────

pub struct RadioParts {
    pub driver: RadioDriver,
    pub delay: Delay,
}

pub struct UsbParts {
    pub driver: UsbDriver,
}

/// Assembled-but-not-yet-initialized TFT plumbing. Real initialization
/// (power gate, RST cycle, mipidsi init) happens in `create_display`.
pub struct DisplayParts {
    interface: TftInterface,
    rst_pin: Output<'static>,
    vdd_en: Output<'static>,
    backlight: SimplePwm<'static>,
}

// ── Display init ────────────────────────────────────────────────────

pub async fn create_display(parts: DisplayParts) -> Option<DisplayDriver> {
    let DisplayParts {
        interface,
        rst_pin,
        mut vdd_en,
        mut backlight,
    } = parts;

    // 1. Drive TFT VDD enable LOW (active LOW per Meshtastic) and let
    //    the rail settle. Heltec's schematic has the panel power gated
    //    by P0.03 — touching SPI before VDD is stable produces snow.
    vdd_en.set_low();
    Timer::after_millis(50).await;

    // 2. Build the lcd-async display.
    //    The T114 panel is wired natively portrait (135w × 240h) at
    //    chip framebuffer offset (52, 40) — the standard 1.14" 135×240
    //    ST7789 die mapping. Deg90 rotation rotates the user-facing
    //    surface to landscape (240w × 135h). Meshtastic's variant.h
    //    sets TFT_OFFSET_X=0/Y=0 because their fork of TFT_eSPI applies
    //    these offsets internally; lcd-async expects them in display_offset.
    //    If the image comes up mirrored or upside-down, swap
    //    Deg90 ↔ Deg270 or set `mirrored = true`.
    let mut delay = Delay;
    let panel = match lcd_async::Builder::new(ST7789, interface)
        .reset_pin(rst_pin)
        .display_size(135, 240)
        .display_offset(52, 40)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .invert_colors(ColorInversion::Inverted)
        .init(&mut delay)
        .await
    {
        Ok(p) => p,
        Err(_) => {
            defmt::error!("ST7789 init failed");
            return None;
        }
    };
    defmt::info!("ST7789 init OK");

    // 3. Bring the backlight up at full brightness. Stage 2 uses the
    //    full-on duty; the splash dimming logic handled by the display
    //    statechart will hand off to set_dim/set_bright shortly.
    backlight.set_duty(0, embassy_nrf::pwm::DutyCycle::inverted(0));

    // Forget vdd_en so it stays driven for the lifetime of the program.
    // We took it by move so we could call set_low() during init.
    core::mem::forget(vdd_en);

    // Take the static framebuffer (240×135 × 2 bytes Rgb565 BE
    // = 64,800 bytes zero-initialized in .bss). Stored as raw bytes
    // so present() can DMA-blit the slice straight to the panel
    // without an iterator-conversion pass — the lcd-async
    // `RawFrameBuf<Rgb565, _>` wrapper handles BE byte packing on
    // each pixel write.
    static FRAMEBUFFER: ConstStaticCell<[u8; FB_BYTES]> =
        ConstStaticCell::new([0u8; FB_BYTES]);
    let fb = FRAMEBUFFER.take();

    Some(St7789Color::new(panel, backlight, fb))
}

// ── Board init ──────────────────────────────────────────────────────

pub struct Board {
    p: embassy_nrf::Peripherals,
}

impl LoRaBoard for Board {
    const NAME: &'static str = "Heltec Mesh Node T114";
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

        // ── VEXT_ENABLE (P0.21, active HIGH) ────────────────────
        // Powers the SX1262 *and* the TFT/peripheral 3V3 rail. Must be
        // driven HIGH before any SPI traffic to the radio or panel —
        // without this, lora-phy init can succeed (SPI bus works
        // electrically) but TX_DONE never fires because the PA is
        // unpowered. Held forever via core::mem::forget. Mirrors
        // Meshtastic's `VEXT_ENABLE = P0.21, VEXT_ON_VALUE = HIGH`.
        let vext_en = Output::new(p.P0_21, Level::High, OutputDrive::Standard);
        core::mem::forget(vext_en);

        // ── SPI3 bus for SX1262 ─────────────────────────────────
        // Per Heltec T114 schematic + Meshtastic variant.h:
        //   SCK=P0.19, MOSI=P0.22, MISO=P0.23
        //   NSS=P0.24, RST=P0.25, BUSY=P0.17, DIO1=P0.20
        // No external RXEN/TXEN — antenna switch is driven internally
        // by SX1262 DIO2. TCXO is on DIO3 at 1.8 V.
        let mut spi_cfg = spim::Config::default();
        spi_cfg.frequency = spim::Frequency::M1;
        let spi = Spim::new(p.SPI3, mcu::Irqs, p.P0_19, p.P0_23, p.P0_22, spi_cfg);
        let spi_bus = mcu::share_spi_bus(spi);

        let nss = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
        let spi_device = SpiDevice::new(spi_bus, nss);

        let reset = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
        let dio1 = Input::new(p.P0_20, Pull::Down);
        // BUSY has an internal 20 kΩ pull-up on the SX1262 die (datasheet
        // p. 53); an MCU-side Pull fights it. See lora-rs/lora-rs#260.
        let busy = Input::new(p.P0_17, Pull::None);

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
        let vbus = mcu::alloc_vbus_detect(true, true);
        let host = UsbParts {
            driver: Driver::new(p.USBD, mcu::Irqs, vbus),
        };

        // Battery divider control (P0.06) LOW disables the divider —
        // saves a few tens of µA. Battery sense (P0.04 / AIN2) deferred
        // until the protocol gets a battery field.
        let bat_ctl = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
        core::mem::forget(bat_ctl);

        // ── SPI2 bus for ST7789 TFT ─────────────────────────────
        // Per Meshtastic variant.h:
        //   SCK=P1.08, MOSI=P1.09, CS=P0.11, DC=P0.12, RST=P0.02
        //   VDD enable=P0.03 (active HIGH), Backlight=P0.15 (active LOW PWM)
        // Write-only bus — embassy_nrf::Spim still requires a MISO slot,
        // so we wire a free unused pin (P1.06) as a no-op input.
        let mut tft_spi_cfg = spim::Config::default();
        // 8 MHz lets us repaint the 240×135 panel comfortably. Higher
        // rates work with mipidsi but are platform-dependent (board
        // routing, pull strengths). 8 MHz is the safe default.
        tft_spi_cfg.frequency = spim::Frequency::M8;
        let tft_spi = Spim::new(p.SPI2, mcu::Irqs, p.P1_08, p.P1_06, p.P1_09, tft_spi_cfg);

        let tft_cs = Output::new(p.P0_11, Level::High, OutputDrive::Standard);
        let tft_dc = Output::new(p.P0_12, Level::Low, OutputDrive::Standard);
        let tft_rst = Output::new(p.P0_02, Level::High, OutputDrive::Standard);
        // VTFT_CTRL is active LOW per Meshtastic
        // (`#define TFT_BACKLIGHT_ON LOW`, `digitalWrite(VTFT_CTRL, LOW)`
        // at boot). Start HIGH (panel rail off); create_display drives
        // LOW to enable the panel.
        let tft_vdd = Output::new(p.P0_03, Level::High, OutputDrive::Standard);

        let tft_spi_device = ExclusiveDevice::new(tft_spi, tft_cs, Delay)
            .expect("TFT ExclusiveDevice init");

        // lcd-async's SpiInterface streams pixel data directly from
        // the caller's slice — no scratch buffer required (mipidsi's
        // sync version needed one for batching).
        let tft_interface = SpiInterface::new(tft_spi_device, tft_dc);

        // ── PWM backlight (active-LOW on P0.15) ─────────────────
        // Default SimpleConfig is fine — countertop=1000 → ~1 kHz
        // (well above flicker threshold), prescaler default. Channel 0
        // idle level matches the active-LOW wiring.
        let mut bl_cfg = SimpleConfig::default();
        bl_cfg.max_duty = crate::driver::st7789::BL_MAX;
        bl_cfg.prescaler = Prescaler::Div128;
        bl_cfg.ch0_idle_level = Level::High;
        let backlight = SimplePwm::new_1ch(p.PWM0, p.P0_15, &bl_cfg);

        let display_parts = DisplayParts {
            interface: tft_interface,
            rst_pin: tft_rst,
            vdd_en: tft_vdd,
            backlight,
        };

        // ── Green LED (P1.03, active LOW) ───────────────────────
        // Idle level HIGH = LED off. InvertedPin flips polarity so
        // SimpleLed can use normal "set_high = on" semantics.
        let led_pin = Output::new(p.P1_03, Level::High, OutputDrive::Standard);
        let led = SimpleLed(InvertedPin(led_pin));

        BoardParts {
            radio,
            host,
            display: Some(display_parts),
            led,
            mac: Self::mac_address(),
        }
    }
}
