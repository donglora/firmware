//! Inter-task communication channels.
//!
//! - [`CommandChannel`]: USB → Radio (host commands)
//! - [`ResponseChannel`]: Radio → USB (firmware responses)
//! - [`DisplayCommandChannel`]: USB → Display (on/off/reset)
//! - [`RadioEventChannel`]: Radio → Display (typed events from the radio)

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::protocol::{Command, RadioConfig, Response};

/// Display commands routed from host_task to display_task.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum DisplayCommand {
    On,
    Off,
    #[allow(dead_code)] // only used by USB host task, not UART
    Reset,
}

/// USB-to-display command channel (depth 4: display commands are rare events).
pub type DisplayCommandChannel = Channel<CriticalSectionRawMutex, DisplayCommand, 4>;

/// Host-to-radio command channel (depth 16: buffer bursty host command sequences).
pub type CommandChannel = Channel<CriticalSectionRawMutex, Command, 16>;

/// Radio-to-host response channel (depth 32: buffer RX packets while USB writes).
pub type ResponseChannel = Channel<CriticalSectionRawMutex, Response, 32>;

/// Radio → downstream typed events. The radio task emits one of these at
/// every mode transition and every RF packet. Mode events and data events
/// are separate: `EnteredRx` / `EnteredTx` / `Idle` describe what the radio
/// is *doing now*, while `PacketRx` / `PacketTx` describe *what just
/// happened* without implying a mode change.
#[derive(Debug, Clone, defmt::Format)]
pub enum RadioEvent {
    /// Radio is now in RX mode.
    EnteredRx,
    /// Radio is now in TX mode.
    EnteredTx,
    /// Radio is now idle.
    Idle,
    /// A packet was received with these link metrics.
    PacketRx { rssi: i16, snr: Option<i16> },
    /// A packet was transmitted successfully.
    PacketTx,
    /// `SetConfig` applied a new validated config.
    ConfigChanged(RadioConfig),
}

/// Radio → Display event stream (depth 32: buffer RX bursts while the
/// display drains slowly).
pub type RadioEventChannel = Channel<CriticalSectionRawMutex, RadioEvent, 32>;
