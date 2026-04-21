//! Inter-task communication channels for DongLoRa Protocol v2.
//!
//! - [`CommandChannel`]: host_task → radio_task (tag-correlated commands)
//! - [`OutboundChannel`]: radio_task → host_task (tag-correlated OK/ERR/TX_DONE
//!   and tag-0 async RX/ERR events in one queue)
//! - [`DisplayCommandChannel`]: host_task → display_task (internal only —
//!   `On` on connect, `Reset` on disconnect; no longer driven from the wire)
//! - [`RadioEventChannel`]: radio_task → display_task (typed mode/packet events)

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::protocol::{Command, DeviceMessage, LoRaConfig};

// ── Host → Radio: tag-correlated commands ───────────────────────────

/// An event flowing from the host task into the radio task.
///
/// `Command` carries a non-zero correlation tag (the host framing layer
/// rejects tag=0 before it reaches here). `Reset` is a lifecycle signal
/// from the host transport layer (DTR drop, UART teardown, or spec §3.4
/// inactivity timeout) telling the radio to park back to Unconfigured
/// and drain queues silently.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum InboundEvent {
    Command { tag: u16, cmd: Command },
    Reset,
}

/// Host-to-radio event channel (depth 8).
pub type CommandChannel = Channel<CriticalSectionRawMutex, InboundEvent, 8>;

// ── Radio → Host: OK / ERR / TX_DONE / RX / async ERR ──────────────

/// One outbound D→H frame carrying its tag. Tag-correlated messages
/// (OK, ERR in response, TX_DONE) carry the originating command's tag;
/// async events (RX, async ERR) carry `tag = 0x0000`.
#[derive(Debug, Clone)]
pub struct OutboundFrame {
    pub tag: u16,
    pub msg: DeviceMessage,
}

/// Radio-to-host outbound channel (depth 32: enough to hold a burst of
/// RX events + queued command responses without stalling the radio ISR).
pub type OutboundChannel = Channel<CriticalSectionRawMutex, OutboundFrame, 32>;

// ── Internal: display commands (no longer wire-visible) ────────────

/// Internal display command routed from host_task to display_task. The
/// wire protocol no longer carries DisplayOn/Off (`PROTOCOL.md §15.2`);
/// these values drive the display statechart from local events only.
#[derive(Debug, Clone, Copy, defmt::Format)]
#[allow(dead_code)] // `Off` is reserved for future diagnostic paths.
pub enum DisplayCommand {
    /// Host connected (DTR rose on USB, or UART came up). Wake display.
    On,
    /// Host disconnected. Park display.
    Off,
    /// Reset cumulative counters (e.g., on config change).
    Reset,
}

/// Host-to-display command channel.
pub type DisplayCommandChannel = Channel<CriticalSectionRawMutex, DisplayCommand, 4>;

// ── Radio → Display: typed packet/mode events ──────────────────────

/// Radio → downstream typed events. These drive the display; they are
/// NOT the same thing as `OutboundFrame` (which goes out the host
/// transport). Mode events describe what the radio is doing now; packet
/// events describe what just happened.
#[derive(Debug, Clone, defmt::Format)]
pub enum RadioEvent {
    EnteredRx,
    EnteredTx,
    Idle,
    PacketRx { rssi: i16, snr: Option<i16> },
    PacketTx,
    ConfigChanged(LoRaConfig),
}

pub type RadioEventChannel = Channel<CriticalSectionRawMutex, RadioEvent, 32>;
