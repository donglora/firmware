//! nRF52840 MCU initialization primitives.
//!
//! Provides shared boilerplate for nRF52840 boards: interrupt bindings,
//! type aliases, StaticCell bus wrapping, USB init, and MAC address reading.

use embassy_nrf::bind_interrupts;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use static_cell::StaticCell;

// ── Interrupts (shared by all nRF52840 boards) ─────────────────────

bind_interrupts!(pub struct Irqs {
    USBD => embassy_nrf::usb::InterruptHandler<embassy_nrf::peripherals::USBD>;
    SPIM3 => embassy_nrf::spim::InterruptHandler<embassy_nrf::peripherals::SPI3>;
    TWISPI0 => embassy_nrf::twim::InterruptHandler<embassy_nrf::peripherals::TWISPI0>;
    // SPIM2 is the second free SPI peripheral on nRF52840. Only used
    // today by the Heltec T114 (ST7789 TFT panel on a write-only bus
    // that can't share with the SX1262 on SPIM3 because the pins
    // differ). Boards that don't use SPI2 don't construct anything
    // against it and this binding is a no-op for them.
    SPI2 => embassy_nrf::spim::InterruptHandler<embassy_nrf::peripherals::SPI2>;
});

// ── MCU-level types ─────────────────────────────────────────────────

pub type SpiBus = embassy_nrf::spim::Spim<'static>;
// I2C is used by some nRF boards (RAK4631 / Wio Tracker L1 OLED) but not
// others (T114 uses SPI for the TFT). The `#[allow(dead_code)]` keeps
// the type available for any board that wants it without per-feature
// gating; rustc otherwise fires a dead-code lint when the active board
// doesn't construct one.
#[allow(dead_code)]
pub type I2cBus = embassy_nrf::twim::Twim<'static>;
pub type UsbNrfDriver =
    embassy_nrf::usb::Driver<'static, &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect>;

// ── SPI bus sharing ─────────────────────────────────────────────────

/// Wrap an initialized SPI peripheral in a shared bus (StaticCell + Mutex).
pub fn share_spi_bus(spi: SpiBus) -> &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus> {
    static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>> =
        StaticCell::new();
    SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi))
}

// ── I2C DMA buffer ──────────────────────────────────────────────────

/// Allocate a DMA-safe buffer for Twim I2C.
///
/// Every nRF52840 Twim user needs a `&'static mut [u8]` buffer.
/// This provides one via StaticCell so board files don't repeat the pattern.
/// Unused on nRF boards that don't expose I2C (e.g. T114 uses SPI for the TFT).
#[allow(dead_code)]
pub fn alloc_i2c_buffer() -> &'static mut [u8; 256] {
    static TWIM_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    TWIM_BUF.init([0u8; 256])
}

// ── USB VBUS detection ──────────────────────────────────────────────

/// Allocate and initialize software VBUS detection.
pub fn alloc_vbus_detect(
    vbus_detect: bool,
    self_powered: bool,
) -> &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect {
    static VBUS: StaticCell<embassy_nrf::usb::vbus_detect::SoftwareVbusDetect> = StaticCell::new();
    VBUS.init(embassy_nrf::usb::vbus_detect::SoftwareVbusDetect::new(
        vbus_detect,
        self_powered,
    ))
}

// ── Reset reason ────────────────────────────────────────────────────

/// Read the POWER.RESETREAS register, log a human-readable summary,
/// then clear it so the next boot reflects only the next reset cause.
///
/// This is the post-mortem diagnostic for soak runs: if the firmware
/// crashes and the chip resets, the next boot's RESETREAS tells us
/// what happened (cortex-m lockup, watchdog, soft reset, debugger,
/// pin reset, etc.). Without this, a self-rebooting firmware just
/// looks like log restarts with no signal as to why.
///
/// nRF52840 `POWER.RESETREAS` (PS §15.4.1): bits are write-1-to-clear.
/// The register accumulates causes across resets until cleared, so
/// always clear after reading.
pub fn dump_reset_reason() {
    // POWER.RESETREAS lives at 0x40000400 (POWER base 0x40000000 +
    // offset 0x400). Direct memory access avoids requiring embassy_nrf's
    // `unstable-pac` feature. Bit layout per nRF52840 PS §15.4.1:
    //   0  RESETPIN  - external NRESET pin asserted
    //   1  DOG       - watchdog reset
    //   2  SREQ      - software-requested reset (cortex SYSRESETREQ)
    //   3  LOCKUP    - cortex-m core lockup
    //  16  OFF       - wake from System OFF via DETECT
    //  17  LPCOMP    - wake from System OFF via ANADETECT
    //  18  DIF       - wake from System OFF via debug interface
    //  24  NFC       - wake from System OFF via NFC field
    const RESETREAS_ADDR: *mut u32 = 0x4000_0400 as *mut u32;
    // SAFETY: 0x40000400 is the POWER.RESETREAS register on nRF52840;
    // read/write_volatile is correct for memory-mapped hardware.
    let raw = unsafe { core::ptr::read_volatile(RESETREAS_ADDR) };
    let dog = (raw & (1 << 1)) != 0;
    let lockup = (raw & (1 << 3)) != 0;
    if raw == 0 {
        defmt::info!("boot: cold POR (RESETREAS=0)");
    } else if dog || lockup {
        // Real problems: watchdog timeout (hung task) OR CPU lockup
        // (HardFault → escalated). Worth a WARN — these are bugs.
        defmt::warn!(
            "boot: ABNORMAL reset RESETREAS=0x{=u32:08x} resetpin={=bool} dog={=bool} sreq={=bool} lockup={=bool} off={=bool} lpcomp={=bool} dif={=bool} nfc={=bool}",
            raw,
            (raw & (1 << 0)) != 0,
            dog,
            (raw & (1 << 2)) != 0,
            lockup,
            (raw & (1 << 16)) != 0,
            (raw & (1 << 17)) != 0,
            (raw & (1 << 18)) != 0,
            (raw & (1 << 24)) != 0,
        );
    } else {
        // Normal reset paths: RESETPIN (UF2 bootloader pulses NRESET on
        // app start; user reset button), SREQ (deliberate sys_reset),
        // OFF/LPCOMP/DIF/NFC (wake from System OFF). All expected
        // operational reasons; INFO not WARN.
        defmt::info!(
            "boot: RESETREAS=0x{=u32:08x} resetpin={=bool} sreq={=bool} off={=bool} lpcomp={=bool} dif={=bool} nfc={=bool}",
            raw,
            (raw & (1 << 0)) != 0,
            (raw & (1 << 2)) != 0,
            (raw & (1 << 16)) != 0,
            (raw & (1 << 17)) != 0,
            (raw & (1 << 18)) != 0,
            (raw & (1 << 24)) != 0,
        );
    }
    // Clear all bits — write-1-to-clear per nRF52840 PS §15.4.1.
    // SAFETY: same justification as the read above.
    unsafe { core::ptr::write_volatile(RESETREAS_ADDR, 0xFFFF_FFFF) };
}

// ── MAC address ─────────────────────────────────────────────────────

/// Read the factory-programmed device address from FICR registers.
pub fn mac_address() -> [u8; 6] {
    // SAFETY: 0x10000000 is the nRF52840 FICR (Factory Information Configuration
    // Registers) base address. Offsets 0x0A4 and 0x0A8 are the DEVICEADDR[0] and
    // DEVICEADDR[1] registers containing the factory-programmed device address.
    // read_volatile is correct for memory-mapped hardware registers.
    unsafe {
        let ficr = 0x10000000 as *const u32;
        let addr0 = core::ptr::read_volatile(ficr.byte_add(0x0A4));
        let addr1 = core::ptr::read_volatile(ficr.byte_add(0x0A8));
        [
            addr0 as u8,
            (addr0 >> 8) as u8,
            (addr0 >> 16) as u8,
            (addr0 >> 24) as u8,
            addr1 as u8,
            (addr1 >> 8) as u8,
        ]
    }
}
