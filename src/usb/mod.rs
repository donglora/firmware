//! USB CDC-ACM interface with COBS-framed command/response protocol.

mod task;
pub use task::usb_task;
