//! Generic `OutputPin` wrapper that inverts logic levels.
//!
//! When a peripheral is wired active-low (LED anode to V+, cathode to GPIO;
//! reset line idle-high, etc.) the physical polarity is a property of the
//! board, not of the driver. Wrapping the pin in `InvertedPin` lets drivers
//! keep "set_high = assert" semantics while the board file expresses the
//! wiring reality at construction time.
//!
//! ```ignore
//! let pin = Output::new(p.GPIO4, Level::High, OutputConfig::default()); // off
//! let led = SimpleLed(InvertedPin(pin)); // driver sees normal polarity
//! ```

use embedded_hal::digital::{ErrorType, OutputPin};

pub struct InvertedPin<P>(pub P);

impl<P: ErrorType> ErrorType for InvertedPin<P> {
    type Error = P::Error;
}

impl<P: OutputPin> OutputPin for InvertedPin<P> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.0.set_high()
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.0.set_low()
    }
}
