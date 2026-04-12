//! RP2040 MCU initialization primitives.
//!
//! Provides shared boilerplate for RP2040 boards: interrupt bindings,
//! type aliases, StaticCell bus wrapping, USB init, and MAC address reading.

use embassy_rp::bind_interrupts;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use static_cell::StaticCell;

// ── Interrupts (shared by all RP2040 boards) ──────────────────────

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<embassy_rp::peripherals::USB>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
        embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>;
});

// ── MCU-level types ────────────────────────────────────────────────

pub type SpiBus =
    embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>;
pub type UsbRpDriver = embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>;

// ── SPI bus sharing ────────────────────────────────────────────────

/// Wrap an initialized SPI peripheral in a shared bus (StaticCell + Mutex).
pub fn share_spi_bus(spi: SpiBus) -> &'static embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus> {
    static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<NoopRawMutex, SpiBus>> =
        StaticCell::new();
    SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi))
}

// ── MAC address ────────────────────────────────────────────────────

/// Store for the MAC address derived from flash unique ID.
/// Populated once during `init_mac()`, read by `mac_address()`.
static MAC: StaticCell<[u8; 6]> = StaticCell::new();
static mut MAC_REF: Option<&'static [u8; 6]> = None;

/// Read the flash chip's unique ID and derive a 6-byte MAC address.
///
/// Must be called once during board init before `mac_address()`.
pub fn init_mac<const FLASH_SIZE: usize>(
    flash: &mut embassy_rp::flash::Flash<
        '_,
        embassy_rp::peripherals::FLASH,
        embassy_rp::flash::Blocking,
        FLASH_SIZE,
    >,
) {
    let mut uid = [0u8; 8];
    flash.blocking_unique_id(&mut uid).expect("flash unique ID");

    // XOR the first 2 bytes into bytes 2..8 to fold 8 bytes into 6,
    // then set the locally-administered bit (byte 0, bit 1).
    let mac = [
        uid[2] ^ uid[0] | 0x02,
        uid[3] ^ uid[1],
        uid[4],
        uid[5],
        uid[6],
        uid[7],
    ];

    let stored = MAC.init(mac);
    // SAFETY: Written once during single-threaded init, before the executor
    // starts. All subsequent reads are from &'static reference.
    unsafe {
        MAC_REF = Some(stored);
    }
}

/// Return the MAC address derived from the flash unique ID.
///
/// Panics if `init_mac()` has not been called.
pub fn mac_address() -> [u8; 6] {
    // SAFETY: MAC_REF is set once during init before the executor starts,
    // and only read (never written) after that.
    unsafe { *MAC_REF.expect("init_mac() must be called before mac_address()") }
}
