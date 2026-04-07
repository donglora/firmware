//! Inter-task communication channels.
//!
//! Each channel connects exactly two tasks:
//! - [`CommandChannel`]: USB → Radio (host commands)
//! - [`ResponseChannel`]: Radio → USB (firmware responses)
//! - [`DisplayCommandChannel`]: USB → Display (on/off/reset)
//! - [`StatusWatch`]: Radio → Display (observable radio state)

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;

use crate::protocol::{Command, RadioConfig, Response};

/// Display commands routed from host_task to display_task.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum DisplayCommand {
    On,
    Off,
    Reset,
}

/// USB-to-display command channel (depth 4: display commands are rare events).
pub type DisplayCommandChannel = Channel<CriticalSectionRawMutex, DisplayCommand, 4>;

/// Host-to-radio command channel (depth 16: buffer bursty host command sequences).
pub type CommandChannel = Channel<CriticalSectionRawMutex, Command, 16>;

/// Radio-to-host response channel (depth 32: buffer RX packets while USB writes).
pub type ResponseChannel = Channel<CriticalSectionRawMutex, Response, 32>;

/// Observable radio status for the display task (2 receivers max).
/// Display always sees the latest state — intermediate updates may be
/// skipped, which is fine for a status display.
pub type StatusWatch = Watch<CriticalSectionRawMutex, RadioStatus, 2>;

/// Current radio state exposed to observers (e.g. display).
#[derive(Debug, Clone, defmt::Format)]
pub struct RadioStatus {
    pub state: RadioState,
    pub config: Option<RadioConfig>,
    pub rx_count: u32,
    pub tx_count: u32,
    pub last_rssi: Option<i16>,
    pub last_snr: Option<i16>,
}

/// Radio state machine states.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RadioState {
    Idle,
    Receiving,
    Transmitting,
}

impl Default for RadioStatus {
    fn default() -> Self {
        Self {
            state: RadioState::Idle,
            config: None,
            rx_count: 0,
            tx_count: 0,
            last_rssi: None,
            last_snr: None,
        }
    }
}
