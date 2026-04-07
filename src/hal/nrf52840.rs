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
});

// ── MCU-level types ─────────────────────────────────────────────────

pub type SpiBus = embassy_nrf::spim::Spim<'static>;
pub type I2cBus = embassy_nrf::twim::Twim<'static>;
pub type UsbNrfDriver =
    embassy_nrf::usb::Driver<'static, &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect>;

// ── SPI bus sharing ─────────────────────────────────────────────────

/// Wrap an initialized SPI peripheral in a shared bus (StaticCell + Mutex).
pub fn share_spi_bus(
    spi: SpiBus,
) -> &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus> {
    static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>> =
        StaticCell::new();
    SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi))
}

// ── I2C DMA buffer ──────────────────────────────────────────────────

/// Allocate a DMA-safe buffer for Twim I2C.
///
/// Every nRF52840 Twim user needs a `&'static mut [u8]` buffer.
/// This provides one via StaticCell so board files don't repeat the pattern.
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
