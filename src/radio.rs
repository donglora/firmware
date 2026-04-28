//! LoRa radio task: SX1262 statechart driven by DongLoRa Protocol v2 host commands.
//!
//! ```text
//! Radio
//! ├── Booting                  (default) cold LoRa::new with timeout. NO event
//! │                                      handlers — cold_init runs uninterruptible.
//! │                                      → Operational on success, → Dead on fail.
//! ├── Operational                       super-state owning all chip-touching
//! │   │                                 lifecycle. PING/GET_INFO/Reset/
//! │   │                                 WedgeDetected handlers live here so
//! │   │                                 they're STRUCTURALLY unreachable in
//! │   │                                 Booting and Dead by event bubbling.
//! │   ├── Unconfigured       (default)  no config cached → RF ops rejected
//! │   ├── Configured
//! │   │   ├── Idle           (default)  ready, waiting
//! │   │   ├── StartingRx                brief: HW init in response to host
//! │   │   │                             RxStart; responds Ok/Err then →
//! │   │   │                             Receiving (Ok) or Idle (Err)
//! │   │   ├── Receiving                 continuous RX
//! │   │   ├── ResumingRx                brief: post-TX HW re-arm; silent →
//! │   │   │                             Receiving (Ok) or Idle (Err)
//! │   │   └── Transmitting
//! │   │       ├── FromIdle  (default)   TxDone → Idle
//! │   │       └── FromRx               TxDone → ResumingRx
//! │   ├── Recovering                    in-place NRESET pulse + lora.init();
//! │   │                                 on success → Unconfigured, on failure
//! │   │                                 → CheckRecoveryBudget
//! │   └── CheckRecoveryBudget           bounded-retry decision: → Recovering
//! │                                     while under MAX_RECOVERY_ATTEMPTS,
//! │                                     → Dead when exhausted
//! └── Dead                              terminal: cold init failed OR recovery
//!                                       exhausted. dead_drain consumes commands
//!                                       inline replying ERadio (PING/INFO still
//!                                       answer); only cold POR (USB unplug)
//!                                       escapes. Per the Harel rule, the chart
//!                                       MUST be non-terminating — radio_task's
//!                                       run() call is followed by panic, since
//!                                       reaching past it implies a chart bug.
//! ```
//!
//! The task body is a single `machine.run().await`. Host commands flow in
//! via the `CommandChannel` as `InboundCommand { tag, cmd }` envelopes;
//! outbound D→H messages flow out via `OutboundChannel` as `OutboundFrame
//! { tag, msg }` envelopes (tag `0` for async RX / async ERR, echoed tag
//! for tag-correlated OK / ERR / TX_DONE).
//!
//! The `RadioInput` event variants intentionally do NOT carry the
//! command tag. Tags are routing metadata — the `next_command` during
//! activity stashes `incoming_tag` into context before emitting the
//! corresponding event, and action handlers read it back from context
//! when building responses. This keeps the hsmc DSL free of
//! bindings-with-transition combos, which hsmc's macro forbids.

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_time::{Delay, Instant};
use heapless::Vec as HVec;
use hsmc::statechart;
use lora_phy::mod_params::RadioError;
use lora_phy::{LoRa, RxMode};

use crate::board::{RadioDriver, RadioParts};
use crate::channel::{
    CommandChannel, InboundEvent, OutboundChannel, OutboundFrame, RadioEvent, RadioEventChannel,
};
use crate::protocol::{
    Command, DeviceMessage, ErrorCode, Info, LoRaBandwidth, LoRaCodingRate, LoRaConfig,
    LoRaHeaderMode, Modulation, OkPayload, Owner, RxOrigin, RxPayload, SetConfigResult,
    SetConfigResultCode, TxDonePayload, TxResult, MAX_OTA_PAYLOAD,
};

// LBT retry timing lives in `crate::lbt` — the math is pure and
// host-testable, so it's split out of this (HW-gated) module.
use crate::lbt::{
    cad_retries, jittered_sleep_us, toa_us, CAD_ASSUMED_PEER_PAYLOAD_B, CAD_BUDGET_US,
};

type LoRaDriver = LoRa<RadioDriver, Delay>;

// ── Pending TX staging ──────────────────────────────────────────────

/// A TX request accepted by the radio task but not yet on the air.
/// Carries the tag so the eventual `TX_DONE` echoes it.
#[derive(Debug, Clone)]
pub struct PendingTx {
    pub tag: u16,
    pub skip_cad: bool,
    pub data: HVec<u8, MAX_OTA_PAYLOAD>,
    pub config: LoRaConfig,
}

// ── Events driving the statechart ───────────────────────────────────
//
// Per-event tags are NOT carried on these variants (see module doc):
// `next_command` stashes `ctx.incoming_tag` before emitting.

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum RadioInput {
    CmdPing,
    CmdGetInfo,
    CmdSetConfig(Modulation),
    /// TX request. The stashed tag + flags + data sit on the context's
    /// `pending_tx` slot (filled by `next_command`); this event merely
    /// wakes the chart.
    CmdTx,
    CmdRxStart,
    CmdRxStop,

    /// Host-transport disconnect / liveness timeout: park the radio.
    Reset,

    /// Internal: emitted by `apply_set_config` on a successful LoRa
    /// configuration apply. Drives the Unconfigured → Configured
    /// transition. No-op when dispatched in Configured.
    ConfigApplied,
    /// Internal: emitted by `apply_set_config` when the SPI write to the
    /// radio failed mid-configuration (spec §6.3). Drives Configured →
    /// Unconfigured; no-op in Unconfigured.
    ConfigFailedRadio,

    PacketReceived(RxPayload),
    RxFailed,
    StartRxOk,
    StartRxFailed,
    TransmitDone {
        airtime_us: u32,
    },
    ChannelBusy,
    TransmitFailed,

    /// Internal: emitted by `bump_resume_retries` when another
    /// `try_start_rx_hw` attempt is allowed. Drives a self-transition on
    /// `ResumingRx` so the entry action runs again after a short backoff.
    ResumeRxRetry,
    /// Internal: emitted by `bump_resume_retries` after
    /// `MAX_RESUME_RETRIES` attempts have failed — the chip is genuinely
    /// wedged; fall through to `Idle` so the host can reconfigure.
    ResumeRxExhausted,

    // ── Cold-boot / Wedge / Recovery / Dead lifecycle (chart-modeled) ──
    /// Internal: emitted by `Booting`'s `during:` after `LoRa::new`
    /// succeeded. Drives `Booting → Unconfigured`.
    BootOk,
    /// Internal: emitted by `Booting`'s `during:` after `LoRa::new`
    /// failed or timed out. Drives `Booting → Dead`.
    BootFailed,
    /// Emitted by an action / `during:` activity when it observes a
    /// `lora_setup*` timeout (i.e. `take_wedge_flag()` returned true on
    /// the failure path). Drives the chart from any operational state
    /// into `Recovering` for an in-place NRESET pulse + `lora.init()`.
    WedgeDetected,
    /// Internal: emitted by `Recovering`'s `during:` when `lora.init()`
    /// succeeded. Drives `Recovering → Unconfigured` (host re-issues
    /// SET_CONFIG to re-arm).
    RecoveryOk,
    /// Internal: emitted by `Recovering`'s `during:` when `lora.init()`
    /// failed (or itself wedged). Drives `Recovering →
    /// CheckRecoveryBudget` so the chart decides to retry or give up.
    RecoveryFailed,
    /// Internal: emitted by `assess_recovery_budget` when the recovery
    /// counter is still under `MAX_RECOVERY_ATTEMPTS`. Drives
    /// `CheckRecoveryBudget → Recovering` for another attempt.
    RetryRecovery,
    /// Internal: emitted by `assess_recovery_budget` when the recovery
    /// counter has reached `MAX_RECOVERY_ATTEMPTS`. Drives
    /// `CheckRecoveryBudget → Dead`.
    RecoveryExhausted,
}

/// How many back-to-back `try_start_rx_hw` failures to tolerate in
/// `ResumingRx` before giving up and parking in `Idle`. 5 × 25 ms ≈
/// 125 ms worst-case resume window, enough to ride out SX1262
/// post-TX timing flukes without pegging the CPU on a dead chip.
const MAX_RESUME_RETRIES: u8 = 5;

/// Backoff between `ResumingRx` retry attempts.
const RESUME_RX_RETRY_BACKOFF_MS: u64 = 25;

/// How many back-to-back `Recovering` (NRESET-pulse + `lora.init()`)
/// attempts to tolerate before declaring the chip permanently dead and
/// parking the chart in `Dead`. Each attempt is bounded to ~2 s by the
/// `lora_setup_with` budget in `try_lora_init`, so 4 attempts give the
/// chart ~8 s of recovery effort before going terminal — long enough
/// to ride out a transient hardware glitch without burning CPU on a
/// chip that is genuinely dead.
const MAX_RECOVERY_ATTEMPTS: u8 = 4;

/// Wall-clock budget for `lora.init()` in the `Recovering` state. Cold
/// init normally completes in well under 100 ms, but the NRESET pulse
/// adds chip wake-up latency. 2 s is ~20× headroom for the documented
/// path; if it blows, the chip is unrecoverable and we fall through to
/// `CheckRecoveryBudget`.
const LORA_INIT_RECOVERY_TIMEOUT_MS: u64 = 2000;

/// Wall-clock budget for cold `LoRa::new` in the `Booting` state. Same
/// 1.5 s headroom we gave the imperative cold-init path before this
/// moved into the chart — covers a stuck-BUSY chip on first power-on.
/// On timeout the chart parks in `Dead`; user power-cycles to escape.
const LORA_COLD_INIT_TIMEOUT_MS: u64 = 1500;

// ── Context ─────────────────────────────────────────────────────────

pub struct RadioContext {
    pub commands: &'static CommandChannel,
    pub outbound: &'static OutboundChannel,
    pub events: &'static RadioEventChannel,
    pub info: &'static Info,
    /// Cold-init parts (driver pins + delay). `Some` at boot; consumed
    /// by `Booting`'s `during:` (`cold_init`) which calls `LoRa::new`.
    /// `None` afterwards — pin ownership has moved into `lora`.
    pub parts: Option<RadioParts>,
    /// Live LoRa driver. `None` while in `Booting` (before `LoRa::new`
    /// returns) or after `Booting → Dead` (cold init failed). `Some`
    /// in every operational state. Action paths invariant: chart never
    /// dispatches operational actions when `lora` is `None`, so an
    /// `Option::expect` there would indicate a chart bug — actions
    /// instead defensive-unwrap and emit a failure event so the chart
    /// self-recovers rather than panicking.
    pub lora: Option<LoRaDriver>,
    pub rx_buf: [u8; MAX_OTA_PAYLOAD],
    /// Active LoRa config. `None` in `Unconfigured`; `Some(cfg)` in
    /// `Configured` (LoRa is the only modulation the firmware drives
    /// today; others are rejected upstream with `EMODULATION`).
    pub config: Option<LoRaConfig>,
    pub pending_tx: Option<PendingTx>,
    /// Packets dropped since the previous successfully-delivered RX event.
    pub packets_dropped: u16,
    /// Tag of the command currently being handled. Written by
    /// `next_command` just before emitting the event; read by action
    /// handlers to construct tag-echoed responses.
    ///
    /// Type-level guard: `None` means "no command is currently being
    /// handled" — actions that read this MUST be reachable only as a
    /// consequence of a chart event dispatched by `next_command`. If an
    /// action reads `None`, the chart's invariant ("commands always go
    /// through next_command first") was violated; we `.expect()` with a
    /// clear panic message rather than silently continuing with tag=0.
    /// `NonZeroU16` because protocol §2.2 reserves tag=0 for async-only
    /// frames; tag-correlated commands are NEVER 0.
    pub incoming_tag: Option<core::num::NonZeroU16>,
    /// Tag of the TX currently on the wire. Captured on entry to
    /// `Transmitting`; consumed by `respond_tx_*` handlers.
    pub last_tx_tag: Option<core::num::NonZeroU16>,
    /// Tag of a pending `RX_START`, pulled by `respond_ok_start_rx` /
    /// `respond_err_start_rx` once the chip reports back.
    pub pending_rx_start_tag: Option<core::num::NonZeroU16>,
    /// Number of consecutive `try_start_rx_hw` failures while in
    /// `ResumingRx`. Reset on any `StartRxOk`. See `MAX_RESUME_RETRIES`
    /// for the limit before falling through to `Idle`.
    pub resume_retries: u8,
    /// Number of `Recovering` attempts the chart has burned this wedge
    /// session. Bumped on `Recovering` entry; cleared on `RecoveryOk`
    /// (chip came back). When it reaches `MAX_RECOVERY_ATTEMPTS` the
    /// chart transitions to `Dead` (terminal until cold POR).
    pub recovery_attempts: u8,
    /// Per-CAD-retry sleep in microseconds: time-on-air for a
    /// `CAD_ASSUMED_PEER_PAYLOAD_B`-byte packet under the current
    /// config. Recomputed on every successful `SET_CONFIG`; `0` while
    /// `Unconfigured` (the CAD path is unreachable without a config).
    pub cad_retry_delay_us: u32,
}

impl RadioContext {
    fn announce(&self, ev: RadioEvent) {
        if self.events.try_send(ev).is_err() {
            warn!("radio event dropped: downstream queue full");
        }
    }

    async fn send(&self, tag: u16, msg: DeviceMessage) {
        self.outbound.send(OutboundFrame { tag, msg }).await;
    }
}

// ── Statechart ─────────────────────────────────────────────────────

statechart! {
    Radio {
        context: RadioContext;
        events: RadioInput;
        // Auto-trace every state entry/exit, action invocation, and
        // transition. Backend (defmt / log / tracing / no-op) is picked
        // by which `hsmc/trace-*` cargo feature is enabled — see
        // `Cargo.toml`. When no backend feature is enabled this line
        // expands to zero code at every emission site.
        trace;
        default(Booting);

        // ── Top-level: three states. Booting (cold init), Operational
        //    (everything that touches the chip), Dead (terminal until
        //    cold POR). Operational is a super-state so that PING /
        //    GET_INFO / Reset / WedgeDetected are STRUCTURALLY
        //    UNREACHABLE in Booting — cold_init's `during:` cannot be
        //    cancelled mid-`LoRa::new` by a stray host event, so the
        //    cancel-safety hazard of `parts.take()` before the await
        //    is unreachable by chart shape, not by defensive code.
        //    Dead handles its own commands inline via `dead_drain`.

        // Cold init lives in the chart, not in `radio_task`. The chart
        // takes ownership of `RadioParts` from context, calls
        // `LoRa::new` with a wall-clock budget, and routes the outcome:
        // success → `Operational/Unconfigured`; failure → `Dead`. No
        // `sys_reset`, no GPREGRET retry. Booting has NO event handlers
        // on purpose: nothing can interrupt cold_init.
        state Booting {
            during: cold_init(parts, lora);
            on(BootOk) => Operational;
            on(BootFailed) => Dead;
        }

        // Operational: every state where the chart is allowed to touch
        // the radio chip. Handlers that only make sense AFTER cold init
        // succeeded live here, not at root, so they cannot fire in
        // Booting or Dead by event bubbling. Per hsmc README rule §3
        // (event bubbling, leaf-first), an event with no handler in
        // Booting bubbles to root and silently drops — the right
        // semantics for "host sent something while we were starting up."
        state Operational {
            // PING / GET_INFO at this level: always answer while
            // operational. In Booting they drop (host retries). In
            // Dead, dead_drain answers them inline.
            on(CmdPing) => respond_pong;
            on(CmdGetInfo) => respond_info;
            // Reset at this level: drives the Operational subtree back
            // to Unconfigured. Resets in Booting drop (cold init must
            // complete first). Resets in Dead drop (chart's terminal).
            on(Reset) => reset_to_unconfigured, Unconfigured;
            // Wedge detected mid-operation: enter Recovering for
            // NRESET-pulse + lora.init() retry. Same reachability
            // guards as Reset.
            on(WedgeDetected) => Recovering;
            default(Unconfigured);

            state Unconfigured {
                during: next_command(commands, incoming_tag, pending_tx);
                on(CmdSetConfig(m: Modulation)) => apply_set_config;
                on(ConfigApplied) => Configured;
                on(CmdTx) => reject_tx_not_configured;
                on(CmdRxStart) => reject_not_configured;
                on(CmdRxStop) => reject_not_configured;
            }

            state Configured {
                on(CmdSetConfig(m: Modulation)) => apply_set_config;
                on(ConfigFailedRadio) => Unconfigured;
                // Default post-reconfigure landing is Idle. The RX ring
                // is always cleared as part of apply_set_config.
                // Receiving overrides this handler to auto-resume RX
                // with the new parameters (spec §3.5) so callers do not
                // need to re-issue RX_START after SET_CONFIG.
                on(ConfigApplied) => Idle;
                default(Idle);

                state Idle {
                    entry: announce_idle;
                    during: next_command(commands, incoming_tag, pending_tx);
                    on(CmdRxStart) => stage_rx_start;
                    on(CmdRxStart) => StartingRx;
                    on(CmdTx) => TransmittingFromIdle;
                    on(CmdRxStop) => respond_ok;
                }

                state StartingRx {
                    entry: try_start_rx_hw;
                    // Per hsmc README rule §1: leaf states need an
                    // active during somewhere on the path so the run
                    // loop has something to await. `pending()` blocks
                    // forever — the chart can only leave via
                    // StartRxOk/Failed.
                    during: pending();
                    on(StartRxOk) => respond_ok_start_rx, Receiving;
                    on(StartRxFailed) => respond_err_start_rx, Idle;
                }

                state Receiving {
                    entry: announce_entered_rx;
                    during: next_command(commands, incoming_tag, pending_tx);
                    during: next_packet(lora, rx_buf, config);
                    on(CmdRxStart) => respond_ok;
                    on(PacketReceived(rx: RxPayload)) => record_packet;
                    // Transient driver error during `next_packet` —
                    // re-arm the chip via ResumingRx rather than
                    // parking in Idle (which would bounce the OLED and
                    // leave the radio waiting for a client-initiated
                    // RX_START that will never come because clients
                    // already think RX is up).
                    on(RxFailed) => ResumingRx;
                    on(CmdRxStop) => stop_rx_hw, respond_ok;
                    on(CmdRxStop) => Idle;
                    on(CmdTx) => TransmittingFromRx;
                    // Spec §3.5: a SET_CONFIG issued while receiving
                    // clears the RX ring (already handled by
                    // apply_set_config) but auto-rearms RX with the new
                    // parameters — the caller does not need to send a
                    // fresh RX_START. Override the parent's
                    // `on(ConfigApplied) => Idle` by landing in
                    // ResumingRx, whose entry kicks the chip back into
                    // RX.
                    on(ConfigApplied) => ResumingRx;
                }

                // Bounded-retry RX re-arm. Entered after every TX while
                // already receiving, on RX driver errors, and on
                // mid-session SET_CONFIG. If `try_start_rx_hw` succeeds
                // we land cleanly in Receiving; if it fails we back off
                // and retry, only conceding to Idle after
                // MAX_RESUME_RETRIES attempts (chip is genuinely wedged
                // at that point).
                state ResumingRx {
                    entry: try_start_rx_hw;
                    during: pending();
                    on(StartRxOk) => reset_resume_retries, Receiving;
                    on(StartRxFailed) => bump_resume_retries;
                    on(ResumeRxRetry) => ResumingRx;
                    on(ResumeRxExhausted) => Idle;
                }

                state Transmitting {
                    entry: announce_entered_tx;
                    // Per hsmc README rule §6, exit actions fire on every
                    // departure from Transmitting — including the
                    // Operational-level `on(WedgeDetected) => Recovering`
                    // transition that bypasses Transmitting's own
                    // `on(TransmitFailed) => respond_tx_failed` handler.
                    // Without this the host never gets a TX_DONE/TX_FAILED
                    // reply on a wedge mid-TX and waits its 2 s firmware-
                    // timeout. `respond_tx_failed` no-ops if `last_tx_tag`
                    // is None (the normal-completion path consumed it via
                    // `take()`), so it's a clean tag-correlated reply
                    // GUARANTEE: every CmdTx gets a response no matter
                    // how Transmitting is exited.
                    exit: respond_tx_failed;
                    exit: announce_packet_tx;
                    default(TransmittingFromIdle);

                    state TransmittingFromIdle {
                        during: perform_tx(lora, pending_tx, cad_retry_delay_us);
                        on(TransmitDone { airtime_us: u32 }) => respond_tx_done;
                        on(TransmitDone) => Idle;
                        on(ChannelBusy) => respond_tx_channel_busy, Idle;
                        on(TransmitFailed) => respond_tx_failed, Idle;
                    }

                    state TransmittingFromRx {
                        during: perform_tx(lora, pending_tx, cad_retry_delay_us);
                        on(TransmitDone { airtime_us: u32 }) => respond_tx_done;
                        on(TransmitDone) => ResumingRx;
                        // Even on failure, we were receiving before the
                        // TX — return to RX (the client still gets a
                        // TX_FAILED reply via respond_tx_failed).
                        // Dropping to Idle here would break the "RX
                        // session stays up" invariant.
                        on(ChannelBusy) => respond_tx_channel_busy, ResumingRx;
                        on(TransmitFailed) => respond_tx_failed, ResumingRx;
                    }
                }
            }

            // In-place wedge recovery: NRESET pulse + lora.init()
            // retry. Entered from any Operational sub-state via the
            // Operational-level `on(WedgeDetected)` handler when a
            // `lora_setup*` op blew past its wall-clock budget (chip
            // BUSY stuck high). NOT a sys_reset — we keep the embassy
            // task alive, just kick the chip. STRUCTURALLY UNREACHABLE
            // from Booting (Booting can't emit WedgeDetected; cold_init
            // doesn't go through lora_setup).
            state Recovering {
                entry: bump_recovery_attempt;
                during: try_lora_init(lora);
                on(RecoveryOk) => clear_recovery_attempts, Unconfigured;
                on(RecoveryFailed) => CheckRecoveryBudget;
            }

            // Decide whether to attempt another recovery or give up.
            // Lives as its own state (rather than inline guard logic)
            // so the budget policy is readable from the chart shape —
            // see `assess_recovery_budget` for the threshold check.
            state CheckRecoveryBudget {
                entry: assess_recovery_budget;
                during: pending();
                on(RetryRecovery) => Recovering;
                on(RecoveryExhausted) => Dead;
            }
        }

        // Terminal: cold init failed OR chart-internal recovery
        // exhausted. Stays here until cold POR (USB unplug → fresh
        // boot, fresh chart). Drains the commands channel inline
        // replying ERadio for any radio-touching command and Ok for
        // PING/GET_INFO so the host protocol stays alive even though
        // the chip is unreachable. No event handlers — `dead_drain`
        // owns the entire command-response loop and never returns.
        state Dead {
            entry: announce_dead;
            during: dead_drain(commands, outbound, info);
        }
    }
}

// ── Lora setup-call wedge protection ──────────────────────────────────
//
// Background: lora-phy's IV calls `wait_on_busy().await` before every
// SPI command. That's a GPIO sense interrupt with NO TIMEOUT in the
// upstream-clean version of lora-phy (which we keep upstreamable and
// thus un-modified here). If the SX1262's BUSY pin is ever stuck high
// — sticky chip state from a prior wedge, hardware glitch, etc. — any
// lora-phy SPI op blocks the radio_task forever.
//
// `lora_setup` wraps a SHORT-DURATION lora-phy op (one that should
// complete in well under 100 ms per datasheet §13) with a generous wall-
// clock bound. On timeout it returns `Err(RadioError::Busy)` — the same
// shape the existing `?` chains in chart actions already handle, so the
// chart's failure events (TxFailed, ConfigFailedRadio, RxFailed,
// StartRxFailed) carry the firmware back to a recoverable state.
//
// Budget rationale: 500 ms = ~5× the worst documented BUSY-high window
// per SX1262 datasheet §13, with slack for cold-boot calibration variance,
// post-TX hot-PA settling, and SPI/DMA arbitration jitter under embassy
// executor load. Genuine wedge (BUSY stuck high for hardware reasons) is
// the only thing this rejects.
//
// Do NOT wrap `lora.rx(&pkt, rx_buf)` — single-packet RX legitimately
// waits indefinitely for an incoming packet. The wedge we're guarding
// against is in setup paths, not the await-IRQ path.

const LORA_SETUP_TIMEOUT_MS: u64 = 500;

async fn lora_setup<T, F>(label: &'static str, fut: F) -> Result<T, RadioError>
where
    F: core::future::Future<Output = Result<T, RadioError>>,
{
    match embassy_time::with_timeout(
        embassy_time::Duration::from_millis(LORA_SETUP_TIMEOUT_MS),
        fut,
    )
    .await
    {
        Ok(r) => r,
        Err(_) => {
            warn!("lora_setup timeout: {}", label);
            crate::channel::LORA_WEDGED.store(true, portable_atomic::Ordering::Release);
            Err(RadioError::Busy)
        }
    }
}

/// Variant for ops with a known-larger budget (TX with long ToA, etc.).
async fn lora_setup_with<T, F>(
    label: &'static str,
    timeout_ms: u64,
    fut: F,
) -> Result<T, RadioError>
where
    F: core::future::Future<Output = Result<T, RadioError>>,
{
    match embassy_time::with_timeout(
        embassy_time::Duration::from_millis(timeout_ms),
        fut,
    )
    .await
    {
        Ok(r) => r,
        Err(_) => {
            warn!("lora_setup timeout: {} ({=u64} ms)", label, timeout_ms);
            crate::channel::LORA_WEDGED.store(true, portable_atomic::Ordering::Release);
            Err(RadioError::Busy)
        }
    }
}

/// True if a `lora_setup*` timeout fired since the last call. Consumes
/// the flag (clears on read) so the next call sees a fresh signal.
/// Use immediately after a lora helper returned `Err(_)` to decide
/// whether to emit `WedgeDetected` (chart enters `Recovering`) or the
/// per-action failure event (chart self-recovers in place).
fn take_wedge_flag() -> bool {
    crate::channel::LORA_WEDGED.swap(false, portable_atomic::Ordering::AcqRel)
}

// ── during: activities ──────────────────────────────────────────────

/// Forever-pending activity. Used as a `during:` for chart states that
/// have no other concurrent activity, to defeat hsmc's run-loop exit
/// condition (`next.is_none() && !has_any_during`). The chart can only
/// leave the state via an explicit transition triggered by an emitted
/// event. Spec property: the radio chart never exits silently.
async fn pending() -> RadioInput {
    core::future::pending::<RadioInput>().await
}

/// Read the next host command, stash its tag into context, and (for TX)
/// stage a placeholder pending-TX struct before emitting the corresponding
/// event. The `apply_set_config` / `try_start_rx_hw` / TX-staging actions
/// patch in the real config from ctx when they run.
async fn next_command(
    commands: &mut &'static CommandChannel,
    incoming_tag: &mut Option<core::num::NonZeroU16>,
    pending_tx: &mut Option<PendingTx>,
) -> RadioInput {
    match commands.receive().await {
        InboundEvent::Reset => {
            // Reset isn't tag-correlated — clear the slot so any stray
            // action that reads incoming_tag panics with a clear msg
            // rather than echoing a stale tag from a prior command.
            *incoming_tag = None;
            RadioInput::Reset
        }
        InboundEvent::Command { tag, cmd } => {
            // Protocol §2.2: tag=0 is rejected by the host framing
            // layer before reaching the commands channel. If we see 0
            // here it's a serious framing-layer bug — fail loudly.
            *incoming_tag = Some(
                core::num::NonZeroU16::new(tag)
                    .expect("protocol §2.2: command tag must be non-zero (host framing bug)"),
            );
            match cmd {
                Command::Ping => RadioInput::CmdPing,
                Command::GetInfo => RadioInput::CmdGetInfo,
                Command::SetConfig(m) => RadioInput::CmdSetConfig(m),
                Command::Tx { flags, data } => {
                    // Placeholder config: the real `ctx.config` is patched
                    // in by `announce_entered_tx`. `reject_tx_not_configured`
                    // (Unconfigured state) consumes this slot without
                    // touching the config field.
                    *pending_tx = Some(PendingTx {
                        tag,
                        skip_cad: flags.skip_cad,
                        data,
                        config: dummy_lora_config(),
                    });
                    RadioInput::CmdTx
                }
                Command::RxStart => RadioInput::CmdRxStart,
                Command::RxStop => RadioInput::CmdRxStop,
            }
        }
    }
}

/// `during:` for the `Booting` state. Owns the cold `LoRa::new` call:
/// takes the consumed-once driver pins out of context, hands them to
/// lora-phy with a wall-clock budget, and stores the result back in
/// context. Emits `BootOk` on success (chart proceeds to `Unconfigured`
/// where the host can `SET_CONFIG`); `BootFailed` on any failure
/// (chart parks in `Dead`; user power-cycles the dongle to escape).
///
/// This replaces the imperative `radio_dead_loop` /
/// `handle_radio_init_failure` / GPREGRET-retry machinery that used to
/// live in `radio_task`. Cold init is now a chart state, not a function.
async fn cold_init(
    parts: &mut Option<RadioParts>,
    lora: &mut Option<LoRaDriver>,
) -> RadioInput {
    // ── DIAGNOSTIC PROBE 1 ──
    // Per hsmc README rule §6, a state's `during:` activity STARTS after
    // the state's entry actions finish. So if this fires, the chart MUST
    // have already emitted `[Radio] enter __Root` and `[Radio] enter
    // Booting` from `enter_state_no_descent`'s trace call. If chart
    // traces are missing from the log but THIS line appears, the
    // problem is specifically in the `__chart_trace!` macro's defmt
    // path — not chart reachability and not the chart engine itself.
    info!("[probe] cold_init enter (chart trace path SHOULD have fired BEFORE this)");
    if lora.is_some() {
        // Already initialized — re-entry into Booting (e.g., user
        // injected a Reset before we even left Booting on first boot).
        // Just acknowledge what's already true.
        return RadioInput::BootOk;
    }
    let Some(p) = parts.take() else {
        // No parts AND no lora — we already failed cold init in a
        // previous Booting visit (parts was consumed by `take()` then).
        // Chart should have transitioned to Dead; if we're here again
        // something forced us back. Stay Failed.
        return RadioInput::BootFailed;
    };
    match embassy_time::with_timeout(
        embassy_time::Duration::from_millis(LORA_COLD_INIT_TIMEOUT_MS),
        LoRa::new(p.driver, false, p.delay),
    )
    .await
    {
        Ok(Ok(l)) => {
            *lora = Some(l);
            // Successful cold init clears any stale wedge flag carried
            // from a prior Recovering attempt — fresh chip, fresh state.
            let _ = take_wedge_flag();
            info!("radio: cold init OK");
            // ── DIAGNOSTIC PROBE 2 ──
            // After this, we return BootOk; the chart's run loop pushes
            // it into the queue and processes it on the next iteration:
            // Booting → Operational. Expected trace lines AFTER this:
            //   [Radio] exit  Booting
            //   [Radio] Booting -> Operational
            //   [Radio] enter Operational
            //   [Radio] enter Unconfigured
            // If those don't appear, the trace path is broken.
            info!("[probe] cold_init OK — about to return BootOk (chart should now exit Booting → Operational)");
            RadioInput::BootOk
        }
        Ok(Err(e)) => {
            error!("radio: cold init failed: {:?}", defmt::Debug2Format(&e));
            RadioInput::BootFailed
        }
        Err(_) => {
            error!(
                "radio: cold init TIMED OUT after {=u64} ms — chip BUSY likely stuck",
                LORA_COLD_INIT_TIMEOUT_MS,
            );
            RadioInput::BootFailed
        }
    }
}

async fn next_packet(
    lora: &mut Option<LoRaDriver>,
    rx_buf: &mut [u8; MAX_OTA_PAYLOAD],
    config: &mut Option<LoRaConfig>,
) -> RadioInput {
    // Chart invariant: Receiving is unreachable from Booting/Dead, so
    // lora is Some here. Defensive bail-out instead of panic so a chart
    // bug self-recovers via RxFailed → ResumingRx rather than dying.
    let Some(lora) = lora.as_mut() else {
        return RadioInput::RxFailed;
    };
    let Some(cfg) = config.as_ref() else {
        return RadioInput::RxFailed;
    };
    let mdltn = match to_lora_mod_params(lora, cfg) {
        Ok(m) => m,
        Err(_) => return RadioInput::RxFailed,
    };
    let pkt = match lora.create_rx_packet_params(
        cfg.preamble_len,
        matches!(cfg.header_mode, LoRaHeaderMode::Implicit),
        MAX_OTA_PAYLOAD as u8,
        cfg.payload_crc,
        cfg.iq_invert,
        &mdltn,
    ) {
        Ok(p) => p,
        Err(_) => return RadioInput::RxFailed,
    };
    match lora.rx(&pkt, rx_buf).await {
        Ok((len, status)) => {
            let ts_us = Instant::now().as_micros();
            let copy_len = (len as usize).min(MAX_OTA_PAYLOAD);
            let mut data: HVec<u8, MAX_OTA_PAYLOAD> = HVec::new();
            let _ = data.extend_from_slice(&rx_buf[..copy_len]);
            RadioInput::PacketReceived(RxPayload {
                rssi_tenths_dbm: (status.rssi as i32 * 10) as i16,
                snr_tenths_db: (status.snr as i32 * 10) as i16,
                // lora-phy 3.0's RxStatus doesn't expose FreqErr; 0 for now.
                freq_err_hz: 0,
                timestamp_us: ts_us,
                // SX126x silently drops bad-CRC packets at the chip; any
                // packet arriving here passed CRC.
                crc_valid: true,
                packets_dropped: 0, // filled in by record_packet
                origin: RxOrigin::Ota,
                data,
            })
        }
        // RX path is the long-await IRQ path that lora_setup is NOT
        // wrapped around (RX legitimately waits indefinitely for a
        // packet). But the helpers in to_lora_mod_params /
        // create_rx_packet_params above DO go through lora_setup-style
        // SPI calls — if any of THOSE wedged earlier in this poll, the
        // flag is set and we should escalate to Recovering rather than
        // bounce through the per-action ResumingRx loop with a still-
        // wedged chip.
        Err(_) => {
            if take_wedge_flag() {
                RadioInput::WedgeDetected
            } else {
                RadioInput::RxFailed
            }
        }
    }
}

async fn perform_tx(
    lora: &mut Option<LoRaDriver>,
    pending: &mut Option<PendingTx>,
    cad_retry_delay_us: &mut u32,
) -> RadioInput {
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(6);
    // Same defensive-bail rationale as `next_packet`.
    let Some(lora) = lora.as_mut() else {
        return RadioInput::TransmitFailed;
    };
    let Some(tx) = pending.take() else {
        return RadioInput::TransmitFailed;
    };
    // Scale CAD retry count to the cached per-retry sleep: fewer retries
    // when each one-full-packet wait is expensive (slow SFs). When the
    // one-packet wait exceeds the total budget, the helper returns
    // `retries = 1` — a single CAD, bail on busy, no sleep.
    let (retries, sleep_us) = cad_retries(*cad_retry_delay_us);
    let start = Instant::now();
    match do_tx(lora, &tx, retries, sleep_us).await {
        Ok(TxOutcome::Transmitted) => RadioInput::TransmitDone {
            airtime_us: Instant::now().duration_since(start).as_micros() as u32,
        },
        Ok(TxOutcome::ChannelBusy) => RadioInput::ChannelBusy,
        Err(_) => {
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(15);
            // Wedge takes precedence over per-action TransmitFailed so
            // the chart escalates to Recovering instead of bouncing
            // back to Idle/ResumingRx with a still-wedged chip.
            if take_wedge_flag() {
                RadioInput::WedgeDetected
            } else {
                RadioInput::TransmitFailed
            }
        }
    }
}

/// `during:` for `Recovering`. Pulses NRESET and re-runs lora-phy's cold
/// init sequence (radio_kind.reset → ensure_ready → set_standby →
/// init_lora → set_tx_power → set_irq_params). Wraps with a generous
/// 2 s budget — `lora.init()` itself contains multiple SPI commands
/// that each can wedge if the chip is genuinely dead.
///
/// Emits `RecoveryOk` on success, `RecoveryFailed` on any error
/// (including a fresh wedge inside this attempt). The chart counts
/// attempts via `bump_recovery_attempt` (entry) and gives up to `Dead`
/// after `MAX_RECOVERY_ATTEMPTS`.
async fn try_lora_init(lora: &mut Option<LoRaDriver>) -> RadioInput {
    let Some(lora) = lora.as_mut() else {
        // Recovering reachable only from operational states (which had
        // a live driver) — None here means the chart is in a bad place.
        // Treat as RecoveryFailed; CheckRecoveryBudget will route us
        // toward Dead which is the right resting state.
        return RadioInput::RecoveryFailed;
    };
    match lora_setup_with(
        "Recovering.lora.init",
        LORA_INIT_RECOVERY_TIMEOUT_MS,
        lora.init(),
    )
    .await
    {
        Ok(()) => {
            // Successful re-init clears any wedge flag the prior failure
            // had set — we know the chip is responsive again.
            let _ = take_wedge_flag();
            RadioInput::RecoveryOk
        }
        Err(_) => RadioInput::RecoveryFailed,
    }
}

/// `during:` for `Dead`. Drains the commands channel forever, replying
/// `Ok` to PING / GET_INFO and `ERadio` to anything else. Never returns
/// — the chart stays in `Dead` until cold POR (USB unplug clears
/// GPREGRET; next boot re-runs cold init from scratch).
///
/// Mirrors the imperative `radio_dead_loop` semantics so the host-side
/// protocol stays alive whether the chip died at cold init (handled by
/// `radio_dead_loop`) or mid-operation (handled here).
async fn dead_drain(
    commands: &mut &'static CommandChannel,
    outbound: &mut &'static OutboundChannel,
    info: &mut &'static Info,
) -> RadioInput {
    loop {
        match commands.receive().await {
            InboundEvent::Reset => {
                // Reset in Dead is a no-op: there's no chip-state to
                // park. Clear the dedup flag so the host chart can
                // queue a fresh Reset later if it wants to.
                crate::channel::RESET_PENDING
                    .store(false, portable_atomic::Ordering::Release);
            }
            InboundEvent::Command { tag, cmd } => {
                let msg = match cmd {
                    Command::Ping => DeviceMessage::Ok(OkPayload::Empty),
                    Command::GetInfo => DeviceMessage::Ok(OkPayload::Info(**info)),
                    _ => DeviceMessage::Err(ErrorCode::ERadio),
                };
                outbound.send(OutboundFrame { tag, msg }).await;
            }
        }
    }
}

// ── Actions ─────────────────────────────────────────────────────────

/// Unwrap an `Option<NonZeroU16>` slot (incoming_tag, last_tx_tag, etc.)
/// to a raw u16 for sending over the wire, panicking with a clear
/// message if the slot is `None`. Reaching `None` here means a chart
/// invariant was violated — an action that constructs a tag-echoed
/// response ran without a preceding `next_command` having stashed the
/// tag, OR the tag-echo path read a slot before the producer wrote it.
#[inline]
fn tag_or_panic(t: Option<core::num::NonZeroU16>, slot: &'static str) -> u16 {
    t.unwrap_or_else(|| {
        defmt::panic!("chart invariant violated: tag slot `{}` was None", slot);
    })
    .get()
}

impl RadioActions for RadioActionContext<'_> {
    // ── PING / GET_INFO ───────────────────────────────────────────────

    async fn respond_pong(&mut self) {
        let tag = tag_or_panic(self.incoming_tag, "respond_pong");
        self.send(tag, DeviceMessage::Ok(OkPayload::Empty)).await;
    }

    async fn respond_info(&mut self) {
        let tag = tag_or_panic(self.incoming_tag, "respond_info");
        let info = *self.info;
        self.send(tag, DeviceMessage::Ok(OkPayload::Info(info)))
            .await;
    }

    // ── SET_CONFIG ────────────────────────────────────────────────────

    async fn apply_set_config(&mut self, m: Modulation) {
        let tag = tag_or_panic(self.incoming_tag, "apply_set_config");
        info!("apply_set_config: tag={=u16}", tag);
        match m {
            Modulation::LoRa(cfg) => {
                if let Err(code) = validate_lora(&cfg, self.info) {
                    // Validation failure: radio + state unchanged (spec §3.5).
                    self.send(tag, DeviceMessage::Err(code)).await;
                    return;
                }
                // Cancel any queued-but-not-started TX per spec §3.5.
                if let Some(cancelled) = self.pending_tx.take() {
                    let td = TxDonePayload {
                        result: TxResult::Cancelled,
                        airtime_us: 0,
                    };
                    self.send(cancelled.tag, DeviceMessage::TxDone(td)).await;
                }
                let recfg_result = match self.lora.as_mut() {
                    Some(lora) => reconfigure_radio(lora, &cfg).await,
                    None => Err(RadioError::Busy),
                };
                if recfg_result.is_err() {
                    // Hardware failure: drop to Unconfigured (spec §6.3).
                    // Clear all per-session state — including any in-flight
                    // RxStart tag — so the next CmdRxStart in Unconfigured
                    // is rejected cleanly (rather than racing a stale tag
                    // through respond_err_start_rx on an unrelated path).
                    self.config = None;
                    self.cad_retry_delay_us = 0;
                    self.pending_rx_start_tag = None;
                    info!("config cleared (apply_set_config failure tag={=u16})", tag);
                    self.send(tag, DeviceMessage::Err(ErrorCode::ERadio)).await;
                    // Wedge takes precedence over ConfigFailedRadio so the
                    // chart enters Recovering (NRESET + lora.init()) instead
                    // of bouncing back to Unconfigured with a still-wedged
                    // chip that the next SET_CONFIG would also fail on.
                    if take_wedge_flag() {
                        let _ = self.emit(RadioInput::WedgeDetected);
                    } else {
                        let _ = self.emit(RadioInput::ConfigFailedRadio);
                    }
                    return;
                }
                self.config = Some(cfg);
                info!("config set (apply_set_config success tag={=u16})", tag);
                self.cad_retry_delay_us = toa_us(&cfg, CAD_ASSUMED_PEER_PAYLOAD_B);
                if self.cad_retry_delay_us > CAD_BUDGET_US {
                    defmt::debug!(
                        "LBT degraded: assumed-packet ToA {=u32} us > budget {=u32} us; single-CAD-and-bail",
                        self.cad_retry_delay_us,
                        CAD_BUDGET_US
                    );
                }
                self.packets_dropped = 0;
                self.announce(RadioEvent::ConfigChanged(cfg));
                let result = SetConfigResult {
                    result: SetConfigResultCode::Applied,
                    owner: Owner::Mine,
                    current: Modulation::LoRa(cfg),
                };
                self.send(tag, DeviceMessage::Ok(OkPayload::SetConfig(result)))
                    .await;
                // Drive the Unconfigured → Configured transition (no-op in
                // Configured — there's no handler for this event there).
                let _ = self.emit(RadioInput::ConfigApplied);
            }
            _ => {
                self.send(tag, DeviceMessage::Err(ErrorCode::EModulation))
                    .await;
            }
        }
    }

    // ── TX ────────────────────────────────────────────────────────────

    async fn reject_tx_not_configured(&mut self) {
        if let Some(pt) = self.pending_tx.take() {
            self.send(pt.tag, DeviceMessage::Err(ErrorCode::ENotConfigured))
                .await;
        }
    }

    async fn respond_tx_done(&mut self, airtime_us: u32) {
        if let Some(tag) = self.last_tx_tag.take() {
            let td = TxDonePayload {
                result: TxResult::Transmitted,
                airtime_us,
            };
            self.send(tag.get(), DeviceMessage::TxDone(td)).await;
        }
    }

    async fn respond_tx_channel_busy(&mut self) {
        if let Some(tag) = self.last_tx_tag.take() {
            let td = TxDonePayload {
                result: TxResult::ChannelBusy,
                airtime_us: 0,
            };
            self.send(tag.get(), DeviceMessage::TxDone(td)).await;
        }
    }

    async fn respond_tx_failed(&mut self) {
        if let Some(tag) = self.last_tx_tag.take() {
            self.send(tag.get(), DeviceMessage::Err(ErrorCode::ERadio))
                .await;
        }
    }

    // ── RX_START / RX_STOP ────────────────────────────────────────────

    async fn stage_rx_start(&mut self) {
        // incoming_tag is already Option<NonZeroU16>; copy directly.
        // If None, the chart invariant was violated (CmdRxStart fired
        // without next_command) — propagate the None so the eventual
        // respond_*_start_rx site silently does nothing rather than
        // echoing a stale tag.
        self.pending_rx_start_tag = self.incoming_tag;
    }

    async fn try_start_rx_hw(&mut self) {
        let Some(cfg) = self.config else {
            // Defensive: shouldn't happen on the documented chart paths
            // (StartingRx is only entered from Idle which requires
            // Configured; ResumingRx is only entered from Receiving or
            // self-loop, both of which had a valid config when reached).
            // If it does fire, the chart self-recovers via StartRxFailed
            // → respond_err_start_rx (replies ERadio to host) → Idle.
            // Logged at WARN with diagnostic tag context so the next soak
            // log captures the chart-state at the moment of the violation.
            warn!(
                "try_start_rx_hw: no config; incoming_tag={=u16}, pending_rx_start_tag={=u16}, last_tx_tag={=u16} (0 = None); emitting StartRxFailed (chart will recover to Idle)",
                self.incoming_tag.map(|t| t.get()).unwrap_or(0),
                self.pending_rx_start_tag.map(|t| t.get()).unwrap_or(0),
                self.last_tx_tag.map(|t| t.get()).unwrap_or(0),
            );
            let _ = self.emit(RadioInput::StartRxFailed);
            return;
        };
        let result = match self.lora.as_mut() {
            Some(lora) => start_rx(lora, &cfg).await,
            None => Err(RadioError::Busy),
        };
        match result {
            Ok(()) => {
                let _ = self.emit(RadioInput::StartRxOk);
            }
            Err(e) => {
                warn!("start_rx failed: {}", e);
                // Wedge → Recovering takes precedence over the per-action
                // self-recover-to-Idle path, so a stuck chip doesn't get
                // re-armed for RX with the still-stuck SPI bus.
                if take_wedge_flag() {
                    let _ = self.emit(RadioInput::WedgeDetected);
                } else {
                    let _ = self.emit(RadioInput::StartRxFailed);
                }
            }
        }
    }

    async fn reset_resume_retries(&mut self) {
        self.resume_retries = 0;
    }

    async fn bump_resume_retries(&mut self) {
        self.resume_retries = self.resume_retries.saturating_add(1);
        if self.resume_retries >= MAX_RESUME_RETRIES {
            warn!(
                "ResumingRx: {} failed attempts, giving up to Idle",
                self.resume_retries
            );
            self.resume_retries = 0;
            let _ = self.emit(RadioInput::ResumeRxExhausted);
            return;
        }
        // Short backoff before the next `try_start_rx_hw` cycle. The
        // self-transition on ResumingRx (driven by ResumeRxRetry) will
        // re-run the entry action.
        embassy_time::Timer::after_millis(RESUME_RX_RETRY_BACKOFF_MS).await;
        let _ = self.emit(RadioInput::ResumeRxRetry);
    }

    async fn respond_ok(&mut self) {
        let tag = tag_or_panic(self.incoming_tag, "respond_ok");
        self.send(tag, DeviceMessage::Ok(OkPayload::Empty)).await;
    }

    async fn respond_ok_start_rx(&mut self) {
        if let Some(tag) = self.pending_rx_start_tag.take() {
            self.send(tag.get(), DeviceMessage::Ok(OkPayload::Empty))
                .await;
        }
    }

    async fn respond_err_start_rx(&mut self) {
        if let Some(tag) = self.pending_rx_start_tag.take() {
            self.send(tag.get(), DeviceMessage::Err(ErrorCode::ERadio))
                .await;
        }
    }

    async fn reject_not_configured(&mut self) {
        let tag = tag_or_panic(self.incoming_tag, "reject_not_configured");
        self.send(tag, DeviceMessage::Err(ErrorCode::ENotConfigured))
            .await;
    }

    async fn stop_rx_hw(&mut self) {
        if let Some(lora) = self.lora.as_mut() {
            let _ = lora_setup("stop_rx_hw.enter_standby", lora.enter_standby()).await;
        }
        // If the call wedged the chip, escalate to Recovering (chart
        // root handler `on(WedgeDetected) => Recovering`) so the next
        // operational action doesn't run against a stuck SPI bus.
        if take_wedge_flag() {
            let _ = self.emit(RadioInput::WedgeDetected);
        }
    }

    // ── Reset (disconnect / inactivity timeout) ──────────────────────

    async fn reset_to_unconfigured(&mut self) {
        info!("config cleared (reset_to_unconfigured)");
        // Clear the dedup flag — host_task can now queue another Reset
        // when needed. Done at the START of this action so even if the
        // chip-side enter_standby below times out / errors, future
        // Resets aren't blocked.
        crate::channel::RESET_PENDING
            .store(false, portable_atomic::Ordering::Release);
        self.pending_tx = None;
        self.last_tx_tag = None;
        self.pending_rx_start_tag = None;
        self.config = None;
        self.packets_dropped = 0;
        // Start each Reset cycle from a clean wedge state — old timeout
        // signal from a prior session shouldn't auto-escalate this one.
        let _ = take_wedge_flag();
        if let Some(lora) = self.lora.as_mut() {
            let _ =
                lora_setup("reset_to_unconfigured.enter_standby", lora.enter_standby()).await;
        }
        // If THIS standby attempt wedged, escalate to Recovering so the
        // chart doesn't loop reset_to_unconfigured every host-inactivity
        // cycle against a chip that's stuck. The root WedgeDetected
        // handler routes us into NRESET-pulse + lora.init() recovery
        // before we re-enter Unconfigured.
        if take_wedge_flag() {
            let _ = self.emit(RadioInput::WedgeDetected);
        }
        self.announce(RadioEvent::Idle);
    }

    // ── Announcements ─────────────────────────────────────────────────

    async fn announce_idle(&mut self) {
        self.announce(RadioEvent::Idle);
    }

    async fn announce_entered_rx(&mut self) {
        self.announce(RadioEvent::EnteredRx);
    }

    async fn announce_entered_tx(&mut self) {
        // Patch the staged TX's config to whatever's active now. The
        // tag and data stay exactly as the host sent them.
        if let (Some(cfg), Some(pt)) = (self.config, self.pending_tx.as_mut()) {
            pt.config = cfg;
        }
        if let Some(pt) = self.pending_tx.as_ref() {
            // PendingTx.tag came from next_command, which already
            // enforced non-zero — fail loudly if that invariant slipped.
            self.last_tx_tag = Some(
                core::num::NonZeroU16::new(pt.tag)
                    .expect("PendingTx.tag was 0 — next_command should have rejected"),
            );
        }
        self.announce(RadioEvent::EnteredTx);
    }

    async fn announce_packet_tx(&mut self) {
        self.announce(RadioEvent::PacketTx);
    }

    // ── Recovery / Dead lifecycle ─────────────────────────────────────

    async fn bump_recovery_attempt(&mut self) {
        self.recovery_attempts = self.recovery_attempts.saturating_add(1);
        info!(
            "Recovering: attempt {=u8}/{=u8}",
            self.recovery_attempts, MAX_RECOVERY_ATTEMPTS,
        );
    }

    async fn clear_recovery_attempts(&mut self) {
        if self.recovery_attempts > 0 {
            info!(
                "Recovering: success after {=u8} attempt(s); chip back online",
                self.recovery_attempts,
            );
        }
        self.recovery_attempts = 0;
        // A successful recovery means the prior config no longer applies
        // to the chip — clear it so the chart's Unconfigured invariant
        // ("config is None") holds when we land there.
        self.config = None;
        self.cad_retry_delay_us = 0;
        self.pending_tx = None;
        self.last_tx_tag = None;
        self.pending_rx_start_tag = None;
        self.announce(RadioEvent::Idle);
    }

    async fn assess_recovery_budget(&mut self) {
        if self.recovery_attempts >= MAX_RECOVERY_ATTEMPTS {
            error!(
                "Recovering: exhausted ({=u8} attempts); transitioning to Dead",
                self.recovery_attempts,
            );
            let _ = self.emit(RadioInput::RecoveryExhausted);
        } else {
            let _ = self.emit(RadioInput::RetryRecovery);
        }
    }

    async fn announce_dead(&mut self) {
        error!(
            "radio chart entered Dead state — chip permanently wedged after {=u8} recovery attempts. Power-cycle the dongle (USB unplug) to reset.",
            self.recovery_attempts,
        );
        // Mirror the cold-init dead path: park display + flush any
        // session state so subsequent host commands get clean ERadio
        // replies (handled by the dead_drain `during:`).
        self.config = None;
        self.cad_retry_delay_us = 0;
        self.pending_tx = None;
        self.last_tx_tag = None;
        self.pending_rx_start_tag = None;
        self.announce(RadioEvent::Idle);
    }

    // ── Packet bookkeeping ────────────────────────────────────────────

    async fn record_packet(&mut self, rx: RxPayload) {
        let rssi_dbm = rx.rssi_tenths_dbm / 10;
        let snr_db = Some(rx.snr_tenths_db / 10);
        self.announce(RadioEvent::PacketRx {
            rssi: rssi_dbm,
            snr: snr_db,
        });
        // Backpressure: if the host transport's IN endpoint is observed-
        // dead (host_task set RADIO_THROTTLED on a write_packet failure),
        // skip the outbound push entirely. Otherwise we'd just fill the
        // queue and the frame would fall out the bottom of the depth-64
        // ring — the drop is the same outcome with extra log noise.
        // Cleared by host_task on the first successful write_packet.
        if crate::channel::RADIO_THROTTLED.load(portable_atomic::Ordering::Acquire) {
            self.packets_dropped = self.packets_dropped.saturating_add(1);
            return;
        }
        let mut rx = rx;
        rx.packets_dropped = self.packets_dropped;
        let delivered = self
            .outbound
            .try_send(OutboundFrame {
                tag: 0,
                msg: DeviceMessage::Rx(rx),
            })
            .is_ok();
        if delivered {
            self.packets_dropped = 0;
        } else {
            warn!("RX dropped: outbound queue full");
            self.packets_dropped = self.packets_dropped.saturating_add(1);
        }
    }

}

// ── LoRa helpers ────────────────────────────────────────────────────

enum TxOutcome {
    Transmitted,
    ChannelBusy,
}

fn to_bw(bw: LoRaBandwidth) -> Option<lora_phy::mod_params::Bandwidth> {
    use lora_phy::mod_params::Bandwidth::*;
    Some(match bw {
        LoRaBandwidth::Khz7 => _7KHz,
        LoRaBandwidth::Khz10 => _10KHz,
        LoRaBandwidth::Khz15 => _15KHz,
        LoRaBandwidth::Khz20 => _20KHz,
        LoRaBandwidth::Khz31 => _31KHz,
        LoRaBandwidth::Khz41 => _41KHz,
        LoRaBandwidth::Khz62 => _62KHz,
        LoRaBandwidth::Khz125 => _125KHz,
        LoRaBandwidth::Khz250 => _250KHz,
        LoRaBandwidth::Khz500 => _500KHz,
        LoRaBandwidth::Khz200
        | LoRaBandwidth::Khz400
        | LoRaBandwidth::Khz800
        | LoRaBandwidth::Khz1600 => return None,
    })
}

fn to_sf(sf: u8) -> Option<lora_phy::mod_params::SpreadingFactor> {
    use lora_phy::mod_params::SpreadingFactor::*;
    Some(match sf {
        5 => _5,
        6 => _6,
        7 => _7,
        8 => _8,
        9 => _9,
        10 => _10,
        11 => _11,
        12 => _12,
        _ => return None,
    })
}

fn to_cr(cr: LoRaCodingRate) -> lora_phy::mod_params::CodingRate {
    use lora_phy::mod_params::CodingRate::*;
    match cr {
        LoRaCodingRate::Cr4_5 => _4_5,
        LoRaCodingRate::Cr4_6 => _4_6,
        LoRaCodingRate::Cr4_7 => _4_7,
        LoRaCodingRate::Cr4_8 => _4_8,
    }
}

fn to_lora_mod_params(
    lora: &mut LoRaDriver,
    cfg: &LoRaConfig,
) -> Result<lora_phy::mod_params::ModulationParams, RadioError> {
    let sf = to_sf(cfg.sf).ok_or(RadioError::UnavailableSpreadingFactor)?;
    let bw = to_bw(cfg.bw).ok_or(RadioError::UnavailableBandwidth)?;
    let cr = to_cr(cfg.cr);
    lora.create_modulation_params(sf, bw, cr, cfg.freq_hz)
}

async fn start_rx(lora: &mut LoRaDriver, cfg: &LoRaConfig) -> Result<(), RadioError> {
    let mdltn = to_lora_mod_params(lora, cfg)?;
    let pkt = lora.create_rx_packet_params(
        cfg.preamble_len,
        matches!(cfg.header_mode, LoRaHeaderMode::Implicit),
        MAX_OTA_PAYLOAD as u8,
        cfg.payload_crc,
        cfg.iq_invert,
        &mdltn,
    )?;
    // RX SETUP — wraps with a setup-budget timeout. The ACTUAL receive
    // (lora.rx call in next_packet activity) is INTENTIONALLY un-wrapped
    // because RX-continuous legitimately waits indefinitely for a packet.
    lora_setup(
        "start_rx.prepare_for_rx",
        lora.prepare_for_rx(RxMode::Continuous, &mdltn, &pkt),
    )
    .await
}

async fn reconfigure_radio(lora: &mut LoRaDriver, cfg: &LoRaConfig) -> Result<(), RadioError> {
    lora_setup("reconfigure_radio.enter_standby", lora.enter_standby()).await?;
    // Calibrate image etc. via a throwaway create_modulation_params call
    // — inside the SET_CONFIG OK window so first TX/RX is fast.
    let _ = to_lora_mod_params(lora, cfg)?;
    Ok(())
}

async fn do_tx(
    lora: &mut LoRaDriver,
    tx: &PendingTx,
    retries: u8,
    sleep_us: u32,
) -> Result<TxOutcome, RadioError> {
    let mdltn = to_lora_mod_params(lora, &tx.config)?;

    if !tx.skip_cad {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(7);
        lora_setup("do_tx.prepare_for_cad", lora.prepare_for_cad(&mdltn)).await?;
        // Reaches here → prepare_for_cad (SPI setup) completed.
        // Hang past here means lora.cad() is waiting on DIO1.
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(19);
        let mut saw_activity = false;
        for i in 0..retries {
            // CAD with 8 symbols at SF12/BW125 is ~262 ms; 1500 ms covers
            // worst-case configs + chip post-processing slack.
            if !lora_setup_with("do_tx.cad", 1500, lora.cad(&mdltn)).await? {
                saw_activity = false;
                break;
            }
            saw_activity = true;
            // Skip the trailing sleep: if the last CAD was busy, we're
            // about to return ChannelBusy, so sleeping + re-preparing
            // is wasted work.
            if i + 1 == retries {
                break;
            }
            let entropy = embassy_time::Instant::now().as_ticks() as u32;
            let jittered_us = jittered_sleep_us(sleep_us, entropy);
            embassy_time::Timer::after(embassy_time::Duration::from_micros(jittered_us as u64))
                .await;
            lora_setup("do_tx.prepare_for_cad(retry)", lora.prepare_for_cad(&mdltn)).await?;
        }
        if saw_activity {
            return Ok(TxOutcome::ChannelBusy);
        }
    }

    let mut tx_pkt = lora.create_tx_packet_params(
        tx.config.preamble_len,
        matches!(tx.config.header_mode, LoRaHeaderMode::Implicit),
        tx.config.payload_crc,
        tx.config.iq_invert,
        &mdltn,
    )?;
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(8);
    lora_setup(
        "do_tx.prepare_for_tx",
        lora.prepare_for_tx(&mdltn, &mut tx_pkt, tx.config.tx_power_dbm as i32, &tx.data),
    )
    .await?;
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(9);
    // TX budget = on-air ToA + 1000 ms slack for PA ramp, IRQ delivery,
    // chip overhead. Skips the wedge-protection if data is huge / config
    // is at SF12/BW125 — but ToA is bounded by config and known up front.
    let payload_len = tx.data.len().min(u16::MAX as usize) as u16;
    let toa_ms = (toa_us(&tx.config, payload_len) / 1000) as u64;
    lora_setup_with("do_tx.tx", toa_ms.saturating_add(1000), lora.tx()).await?;
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(10);
    Ok(TxOutcome::Transmitted)
}

fn validate_lora(cfg: &LoRaConfig, info: &Info) -> Result<(), ErrorCode> {
    if cfg.freq_hz < info.freq_min_hz || cfg.freq_hz > info.freq_max_hz {
        return Err(ErrorCode::EParam);
    }
    if !(5..=12).contains(&cfg.sf) {
        return Err(ErrorCode::EParam);
    }
    if info.supported_sf_bitmap & (1u16 << cfg.sf) == 0 {
        return Err(ErrorCode::EParam);
    }
    if info.supported_bw_bitmap & (1u16 << cfg.bw.as_u8()) == 0 {
        return Err(ErrorCode::EParam);
    }
    if cfg.tx_power_dbm < info.tx_power_min_dbm || cfg.tx_power_dbm > info.tx_power_max_dbm {
        return Err(ErrorCode::EParam);
    }
    if cfg.preamble_len < 6 {
        return Err(ErrorCode::EParam);
    }
    Ok(())
}

fn dummy_lora_config() -> LoRaConfig {
    LoRaConfig {
        freq_hz: 915_000_000,
        sf: 7,
        bw: LoRaBandwidth::Khz125,
        cr: LoRaCodingRate::Cr4_5,
        preamble_len: 8,
        sync_word: 0x1424,
        tx_power_dbm: 0,
        header_mode: LoRaHeaderMode::Explicit,
        payload_crc: true,
        iq_invert: false,
    }
}

// ── Task entry ──────────────────────────────────────────────────────

#[task]
pub async fn radio_task(
    parts: RadioParts,
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
    events: &'static RadioEventChannel,
    info: &'static Info,
) {
    // The chart owns everything: cold init, operational lifecycle,
    // wedge recovery, dead-loop. radio_task is just a thin shim that
    // hands `parts` (consumable pin ownership) into context and runs
    // the chart. See `Radio` chart's `Booting` / `Recovering` / `Dead`
    // states for the lifecycle policy.
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(1);

    let ctx = RadioContext {
        commands,
        outbound,
        events,
        info,
        parts: Some(parts),
        lora: None,
        rx_buf: [0u8; MAX_OTA_PAYLOAD],
        config: None,
        pending_tx: None,
        packets_dropped: 0,
        incoming_tag: None,
        last_tx_tag: None,
        pending_rx_start_tag: None,
        resume_retries: 0,
        recovery_attempts: 0,
        cad_retry_delay_us: 0,
    };

    // Per hsmc 0.3 README: channel capacity MUST match the machine's
    // internal event-deque capacity. Both are 32 here (default is 8):
    // action chains can emit a transient burst (e.g. apply_set_config
    // emits ConfigApplied + possibly WedgeDetected; reset_to_unconfigured
    // may emit WedgeDetected; etc.). 32 is comfortable headroom before
    // hsmc's `take_overflow()` would surface Err(QueueFull) out of run().
    static RADIO_INTERNAL_CH: embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        RadioInput,
        32,
    > = embassy_sync::channel::Channel::new();
    let mut machine =
        Radio::with_queue_capacity::<_, 32>(ctx, &RADIO_INTERNAL_CH);
    // Per the chart's Harel structure: the radio chart is a non-
    // terminating reactive system. There is no `terminate:` event
    // declared, and Booting/Operational/Dead all have `during:`
    // activities that prevent the run loop's silent-exit path. The
    // ONLY way `run()` can return is `Err(QueueFull)` — which would
    // be a chart-design bug, not a runtime condition to recover from.
    // Halt loud so the bug is visible at the source rather than
    // letting the task end silently and leaving the dongle limping.
    let result = machine.run().await;
    defmt::panic!(
        "radio chart returned: {:?} — this is structurally impossible per chart design",
        defmt::Debug2Format(&result),
    );
}
