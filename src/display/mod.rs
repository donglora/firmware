//! OLED display driver with radio status dashboard.

mod render;
pub mod sh1106;
mod task;
pub use task::display_task;
