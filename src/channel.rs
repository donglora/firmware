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
use portable_atomic::AtomicBool;

use crate::protocol::{Command, DeviceMessage, LoRaConfig};

/// True while a Reset has been queued in `CommandChannel` but not yet
/// consumed by the radio task. Dedupes injections from any host chart
/// (USB DTR-driven OR UART inactivity-driven) that fires `reset_radio`
/// repeatedly during quiet/inactive periods. Without this dedup, a slow
/// or wedged radio causes the host chart's exit action to inject Reset
/// every cycle, filling the commands channel and producing a "queue
/// full" warn flood. Cleared by the radio chart's Reset handler
/// (`reset_to_unconfigured`).
///
/// Uses `portable_atomic::AtomicBool` (not `core::sync::atomic`) because
/// Cortex-M0+ targets (RP2040) lack native CAS for `swap`. portable-atomic
/// transparently uses native atomics on M3+/M4 and critical-section
/// emulation on M0/M0+.
pub static RESET_PENDING: AtomicBool = AtomicBool::new(false);

/// True while the host transport's USB IN endpoint is observed-dead
/// (write_packet returned Err). Set by `host_task::protocol_loop` on the
/// first failed write; cleared on the next successful write. Producers of
/// **async** D→H events — `radio_task::record_packet` (RX) and
/// `host::framing::emit_async_err` — check this flag before pushing into
/// `OutboundChannel` and silently skip when set, so the queue does not
/// fill while the host is gone.
///
/// Tag-correlated responses (OK / sync ERR / TX_DONE) are NOT gated by
/// this flag: they're a small, bounded volume and the protocol contract
/// requires them to be delivered if we ever reach the host again.
///
/// Same `portable_atomic::AtomicBool` rationale as `RESET_PENDING`.
///
/// **Initial value `true`**: at boot we have no proof the host can
/// drain the outbound queue. Until host_task observes its first
/// successful `write_packet` (and clears this), async-RX producers
/// silently skip pushing. Otherwise — at boot, with no client
/// attached or with mux-rs idle — RX events would fill the queue
/// faster than host_task could drain it (which it can't: `write_packet`
/// blocks until the host actually polls), and the depth-64 ring would
/// flood `[WARN] RX dropped: outbound queue full` hundreds of times
/// per second. Pre-throttle is the structural fix.
pub static RADIO_THROTTLED: AtomicBool = AtomicBool::new(true);

/// Set by `radio::lora_setup` / `lora_setup_with` when a SHORT-DURATION
/// lora-phy op blew through its wall-clock budget (almost always BUSY
/// stuck high — chip is wedged). Read by chart actions and `during:`
/// activities that called the helper: when they observe a downstream
/// `Err(_)` AND see this flag set, they emit `WedgeDetected` (which
/// drives the radio chart into `Recovering` for an in-place NRESET +
/// `lora.init()` retry) instead of the per-action failure event. The
/// flag is consumed (`swap(false)`) by whoever reads it so the next
/// independent op gets a fresh signal.
///
/// This is the cleanest plumbing path for "wedge timeout vs ordinary
/// driver error" given the helper sits below the chart-action layer
/// (free function, no chart-emit access). One atomic, one concern,
/// matching the established `RESET_PENDING` / `RADIO_THROTTLED` pattern.
pub static LORA_WEDGED: AtomicBool = AtomicBool::new(false);

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

/// Radio-to-host outbound channel.
///
/// Depth 64: absorbs bursts of RX events plus queued command responses
/// plus async ERRs while the host transport drains. Each slot is ~280 B
/// (max RxPayload + envelope), so 64 slots is ~18 KB — well within RAM
/// budget on every supported MCU. The previous depth-32 was provoking
/// "outbound queue full" floods during host stalls; the larger queue
/// plus `RADIO_THROTTLED` backpressure flag (set by host_task on a
/// failed write_packet) together stop the cascade.
pub type OutboundChannel = Channel<CriticalSectionRawMutex, OutboundFrame, 64>;

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
