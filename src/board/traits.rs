//! Board abstraction trait.
//!
//! Every board must implement [`LoRaBoard`]. The compiler enforces completeness —
//! missing constants or methods are compile errors, not runtime surprises.
//!
//! Board modules also export concrete types (`RadioParts`, `UsbParts`,
//! `DisplayParts`, `RadioDriver`, `UsbDriver`, `DisplayI2c`) that the tasks
//! use directly. The trait verifies metadata and initialization; the types
//! verify hardware compatibility.
//!
//! See `src/board/PORTING.md` for a step-by-step guide to adding a new board.

/// Compile-time contract for a DongLoRa board.
///
/// Every board's `Board` struct must implement this trait. It verifies that
/// the board provides all required metadata and initialization methods.
/// The tasks use the board's concrete types directly (not the trait's
/// associated types) because Embassy tasks must be concrete.
pub trait LoRaBoard {
    /// Human-readable board name (shown on display splash screen).
    const NAME: &'static str;

    /// TX power range in dBm (min, max) for this board's radio + PA.
    const TX_POWER_RANGE: (i8, i8);

    /// Initialize the board hardware.
    fn init() -> Self;

    /// Read the board's unique hardware address (MAC, device ID, etc.).
    fn mac_address() -> [u8; 6];
}
