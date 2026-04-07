//! OLED display driver with radio status dashboard.

mod render;
#[cfg(feature = "wio_tracker_l1")]
pub mod sh1106;
mod task;
pub use task::display_task;
