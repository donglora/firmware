//! LoRa radio task: SX1262 statechart driven by DongLoRa Protocol v2 host commands.
//!
//! ## Cancel-safety design rule
//!
//! lora-phy explicitly documents `lora.rx()` / `lora.cad()` / `lora.tx()` /
//! `complete_rx()` / `process_irq_event()` as "not safe to drop or cancel —
//! must run to completion". Dropping a cancel-unsafe future mid-await leaves
//! the chip in an indeterminate state from which only an NRESET pulse can
//! recover. Per hsmc spec §6, a state's `during:` activity is dropped when
//! the state is exited by a transition.
//!
//! The chart honors lora-phy's contract by ensuring **every cancel-unsafe
//! lora-phy future runs to natural completion**:
//!
//! 1. **RX uses cancel-safe primitives only.** `irq_pump` (RxSession's during)
//!    composes only `wait_for_irq` (GPIO-sense), `get_irq_state` (single SPI
//!    read), and `clear_irq_status` (single SPI write) — all documented
//!    cancel-safe. The post-IRQ `get_rx_result` is in the `collect_rx_packet`
//!    action, also cancel-safe (single SPI read pair). RxActive transitions
//!    to RxDraining on CmdRxStop/CmdTx/ConfigApplied — RxSession's `irq_pump`
//!    is dropped; RxDraining's entry calls `enter_standby` (cancel-safe;
//!    forces the chip out of RX cleanly per SX1262 §8.5).
//!
//! 2. **TX sandboxes cancel-unsafe `lora.cad()` and `lora.tx()` AS-IS.**
//!    `TxCadWait` and `TxOnAir` declare zero outbound event handlers other
//!    than the events the during itself emits on natural completion. The
//!    chart cannot drop these futures via a transition because no handler
//!    can fire. Wedge timers (`on(after CAD_OVERLONG_MS) => mark_attempt_overlong`)
//!    set context flags only — no transition. The during checks the flag at
//!    its next cancel-safe wakeup and returns cleanly.
//!
//! 3. **Reset arriving mid-TX is DEFERRED.** TxSession's `on(Reset) =>
//!    mark_pending_reset` only sets a flag. `route_tx_outcome` (TxFanout's
//!    entry) checks the flag AFTER natural TX completion and emits
//!    RadioInput::Reset then — Operational's `on(Reset) =>
//!    reset_to_unconfigured` runs against a chip in known-clean state.
//!
//! ## Chart shape
//!
//! ```text
//! Radio
//! ├── Booting                       (default) cold LoRa::new — uninterruptible.
//! │                                           NO event handlers; LoRa::new runs
//! │                                           to natural completion. → Operational
//! │                                           on success, → Dead on fail.
//! ├── Operational                            owns all chip-touching lifecycle.
//! │   │                                      PING/GET_INFO/Reset/WedgeDetected
//! │   │                                      handlers live here, structurally
//! │   │                                      unreachable in Booting and Dead.
//! │   ├── Unconfigured             (default) no config cached → RF ops rejected
//! │   ├── Configured
//! │   │   ├── Idle                 (default) ready, waiting
//! │   │   ├── RxSession                      super-state owning irq_pump
//! │   │   │   │                              (cancel-safe IRQ wakeup loop)
//! │   │   │   ├── RxArming         (default) brief: HW arming for RX_START
//! │   │   │   ├── RxActive                   continuous RX; collect_rx_packet
//! │   │   │   │                              fires on each Dio1RxDone
//! │   │   │   ├── RxDraining                 cancel-safe RX wind-down;
//! │   │   │   │                              enter_standby clears chip state
//! │   │   │   └── RxResuming                 bounded-retry HW re-arm post-TX
//! │   │   └── TxSession                      super-state owning TX lifecycle;
//! │   │       │                              exit:respond_tx_failed guarantees
//! │   │       │                              tag-correlated reply
//! │   │       ├── TxCadPrep        (default) SPI prep for CAD (or skip)
//! │   │       ├── TxCadWait                  cancel-unsafe `lora.cad()` runs in
//! │   │       │                              during; no preempting handlers
//! │   │       ├── TxBackoff                  jittered inter-CAD sleep (timer)
//! │   │       ├── TxPrep                     SPI prep for TX
//! │   │       ├── TxOnAir                    cancel-unsafe `lora.tx()` runs in
//! │   │       │                              during; no preempting handlers
//! │   │       └── TxFanout                   route post-TX: Idle | RxResuming |
//! │   │                                      pending-Reset honored here
//! │   ├── Recovering                        in-place NRESET + lora.init() (also
//! │   │                                     cancel-unsafe; no preempting events
//! │   │                                     in Recovering — runs to completion).
//! │   └── CheckRecoveryBudget               bounded-retry: Recovering vs Dead.
//! └── Dead                                  terminal: cold init failed OR
//!                                           recovery exhausted. dead_drain
//!                                           replies ERadio inline. Cold POR
//!                                           is the only escape. radio_task
//!                                           panics if run() returns (chart
//!                                           is non-terminating by design).
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
use embassy_time::Delay;
use heapless::Vec as HVec;
use hsmc::{statechart, Duration};
use lora_phy::mod_params::{ModulationParams, PacketParams, RadioError};
use lora_phy::mod_traits::IrqState;
use lora_phy::{LoRa, RxMode};

// `IrqState` is opaque to RadioInput (doesn't derive Debug, which
// RadioInput requires). irq_pump matches on it locally and emits
// Dio1RxDone / Dio1RxIntermediate accordingly. ModulationParams and
// PacketParams are cached on RadioContext so the new RX/TX
// cancel-safe subtree's actions don't have to rebuild them.

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
    StartRxOk,
    StartRxFailed,
    TransmitFailed,

    // ── New cancel-safe RX/TX subtree events ──
    /// Emitted by `irq_pump` (RxSession's during) on a DIO1 rising
    /// edge where the chip's IRQ state read back as `IrqState::Done`
    /// (RX completed, packet ready). RxActive dispatches to
    /// `collect_rx_packet` action which calls `lora.get_rx_result()`.
    Dio1RxDone,
    /// Emitted by `irq_pump` on a DIO1 rising edge for non-completion
    /// IrqStates (currently just `PreambleReceived`). The chart
    /// self-transitions on this — irq_pump just re-runs.
    Dio1RxIntermediate,
    /// Emitted by `irq_pump` when wait_for_irq or get_irq_state
    /// returned Err. Drives RxActive → RxResuming for retry.
    IrqWaitFailed,
    /// Emitted by `drain_rx_hw` after enter_standby completes. Carries
    /// the destination state (Idle vs TxCadPrep vs Reconfiguring) that
    /// was stashed in ctx.rx_drain_dest by the RxActive handler that
    /// triggered the drain.
    RxDrained,
    /// Emitted by `cad_one_step` when CAD completed and channel was
    /// clear (CAD didn't detect activity). Drives TxCadWait → TxPrep.
    CadIdle,
    /// Emitted by `cad_one_step` when CAD completed and detected
    /// activity. Drives TxCadWait → TxBackoff (or TxFanout if max
    /// attempts reached).
    CadBusy,
    /// Emitted by `prep_for_cad` action on success. Drives
    /// TxCadPrep → TxCadWait.
    CadPrepOk,
    /// Emitted by `prep_for_tx` action on success. Drives
    /// TxPrep → TxOnAir.
    TxPrepOk,
    /// Emitted by `jittered_sleep_during` (TxBackoff's during) when
    /// the inter-CAD sleep elapses. Drives TxBackoff → TxCadPrep
    /// (re-prepare for next CAD attempt).
    CadBackoffDone,
    /// Emitted by `tx_complete` (TxOnAir's during) on successful TX
    /// completion. Drives TxOnAir → TxFanout. Carries airtime so the
    /// host's TX_DONE response includes it.
    TxOnAirOk { airtime_us: u32 },
    /// Emitted by `route_tx_outcome` (TxFanout's entry action) when
    /// the TX session entered from Idle. Drives TxFanout → Idle.
    TxFanoutToIdle,
    /// Emitted by `route_tx_outcome` (TxFanout's entry action) when
    /// the TX session entered from RX. Drives TxFanout → RxResuming
    /// to re-arm RX with the current config.
    TxFanoutToRx,

    /// Emitted by `prep_for_cad_act` when the in-flight TX has
    /// `skip_cad=true`. Drives TxCadPrep → TxPrep, bypassing CAD.
    TxCadSkipped,

    /// Emitted by `bump_cad_attempt` when cad_attempts_used reaches
    /// cad_attempts_max. Drives TxBackoff → TxFanout via stage_tx_busy
    /// (the host gets ChannelBusy).
    CadBackoffMaxed,

    // ── Routing events from check_overlong_route_* actions ──
    /// Emitted by check_overlong_route_idle (no overlong) — channel
    /// clear, proceed to TxPrep.
    TxCadGoToPrep,
    /// Emitted by check_overlong_route_busy (no overlong) — channel
    /// busy, back off and retry.
    TxCadGoToBackoff,
    /// Emitted by check_overlong_route_fail OR by any overlong route
    /// — go to TxFanout with TxOutcomeKind::Failed staged.
    TxCadGoToFanoutFailed,

    // ── Routing events from route_after_rx_drain ──
    /// Emitted by route_after_rx_drain when ctx.rx_drain_dest=Idle.
    RxRouteToIdle,
    /// Emitted by route_after_rx_drain when ctx.rx_drain_dest=Tx.
    RxRouteToTx,
    /// Emitted by route_after_rx_drain when ctx.rx_drain_dest=ResumingRx.
    RxRouteToResume,

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

/// Soft-warning budget for a single CAD attempt. Exceeded only on
/// chips that are misbehaving (typical SF12/BW125 CAD ≈ 262 ms; lower
/// SF much faster). When the chart timer fires, `mark_attempt_overlong`
/// sets a context flag and logs — NO transition. The during keeps
/// running. The flag is checked by routing actions when the during
/// finally completes naturally.
const CAD_OVERLONG_MS: u64 = 1500;

/// Hard-abort budget for a single CAD attempt. After this, the chart
/// timer sets `cad_must_abort` — `cad_one_step`'s NEXT call (after
/// natural completion of the current `lora.cad()`) sees the flag,
/// cancel-safely walks the chip back to standby, and returns
/// TransmitFailed. The chart THEN routes to Recovering. Generous
/// because the chip MAY be slowly servicing the CAD request and we
/// want to honor lora-phy's "must run to completion" contract before
/// escalating.
const CAD_MUST_ABORT_MS: u64 = 15_000;

/// Hard-abort budget for `lora.tx()`. Worst-case ToA at SF12/BW125
/// for MAX_OTA_PAYLOAD ≈ 6 s; +PA ramp + IRQ latency + chart overhead
/// = ≤10 s realistic. 30 s gives 3× safety multiple before escalation.
const TX_MUST_ABORT_MS: u64 = 30_000;

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

    // ── Cancel-safe RX/TX subtree state ──
    /// Cached `ModulationParams` derived from `config`. Built once in
    /// `apply_set_config` so the new TX subtree's actions don't have
    /// to rebuild it (every CAD attempt would otherwise call
    /// `lora.create_modulation_params` — pure compute, but redundant).
    /// Cleared when chart leaves `Configured`.
    pub mdltn: Option<ModulationParams>,
    /// Cached `PacketParams` for RX, derived from `config`. Built at
    /// `RxArming` entry, used by the `collect_rx_packet` action when
    /// `Dio1RxDone` fires. Cleared when chart leaves `RxSession`.
    pub rx_pkt: Option<PacketParams>,
    /// Number of CAD attempts in the current TX session. Bumped on
    /// every TxBackoff entry; compared against the `cad_retries` budget
    /// from `lbt::cad_retries(...)` to decide TxBackoff → TxCadPrep
    /// vs TxFanout(ChannelBusy).
    pub cad_attempts_used: u8,
    /// Total CAD attempts allowed for the current TX session. Computed
    /// once in `snapshot_pending_tx` (TxSession entry) from the active
    /// config via `lbt::cad_retries`.
    pub cad_attempts_max: u8,
    /// Soft-warning flag: a single CAD attempt ran past the
    /// `CAD_OVERLONG_MS` chart-timer budget. `mark_attempt_overlong`
    /// sets it; the routing actions (`check_overlong_*`) read it AFTER
    /// the during's natural completion to decide whether to escalate
    /// to `WedgeDetected`. Reset on every TxCadPrep entry.
    pub cad_attempt_overlong: bool,
    /// Hard-abort flag: chart timer at `CAD_MUST_ABORT_MS` set this.
    /// `cad_one_step` checks the flag at its next cancel-safe wakeup,
    /// calls `lora.enter_standby()` (cancel-safe) and returns
    /// `TransmitFailed`. The chart then routes to `Recovering` AFTER
    /// the during has cleanly returned — honors lora-phy's "must run
    /// to completion" contract.
    pub cad_must_abort: bool,
    /// Same shape as `cad_must_abort` but for TxOnAir's `tx_complete`
    /// during. Set by `TxOnAir`'s `TX_MUST_ABORT_MS` chart timer.
    pub tx_must_abort: bool,
    /// Was the current TX session entered from RX (so TxFanout routes
    /// to RxResuming) or from Idle (routes to Idle)? Set by the
    /// `snapshot_pending_tx` entry action by inspecting the chart
    /// transition trace; consumed by `route_tx_outcome` in TxFanout.
    pub from_rx_session: bool,
    /// Staged TX outcome to be routed by TxFanout. Set by the various
    /// TxCadWait/TxOnAir handlers (CadIdle/CadBusy/TxOnAirOk/...) just
    /// before they transition to TxFanout; consumed by
    /// `route_tx_outcome`.
    pub tx_outcome: Option<TxOutcomeKind>,
    /// Set by `mark_pending_reset` action when Reset bubbles to
    /// `TxSession` mid-TX. After natural completion of the in-flight
    /// op, `route_tx_outcome` checks this and emits `RadioInput::Reset`
    /// AFTER the chart leaves TxSession via the normal fanout path,
    /// so reset_to_unconfigured runs against a chip that lora-phy has
    /// finished talking to.
    pub pending_reset: bool,
    /// Destination state for the `RxDraining → ?` transition. Set by
    /// the `RxActive` event handler that fires `=> RxDraining` (e.g.,
    /// CmdTx sets this to TxCadPrep; CmdRxStop sets to Idle;
    /// ConfigApplied sets to RxResuming). Consumed by
    /// `route_after_rx_drain`.
    pub rx_drain_dest: Option<RxDrainDest>,
}

/// Outcome of a TX session, staged by handlers in TxCadWait/TxOnAir
/// and consumed by `route_tx_outcome` in TxFanout.
#[derive(Debug, Clone, Copy)]
pub enum TxOutcomeKind {
    /// TX completed on the air. `airtime_us` was reported via
    /// `respond_tx_done` in the same handler chain that set this.
    Transmitted,
    /// All CAD attempts saw activity; we never transmitted.
    /// `respond_tx_channel_busy` already fired.
    ChannelBusy,
    /// SPI prep, CAD itself, or TX itself errored. `respond_tx_failed`
    /// already fired (or will via TxSession's exit guarantee).
    Failed,
}

/// Destination for the `RxDraining → ?` transition, stashed by the
/// RxActive handler that triggered the drain.
#[derive(Debug, Clone, Copy)]
pub enum RxDrainDest {
    /// CmdRxStop — host wants RX off; fanout to Idle.
    Idle,
    /// CmdTx during RX — fanout to TxCadPrep to start a TX session.
    /// `from_rx_session` is set so post-TX returns to RxResuming.
    Tx,
    /// ConfigApplied during RX — config changed; fanout to RxResuming
    /// which re-arms RX with the new config.
    ResumingRx,
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
                // RxActive overrides this handler to auto-resume RX
                // with the new parameters (spec §3.5) so callers do not
                // need to re-issue RX_START after SET_CONFIG.
                on(ConfigApplied) => Idle;
                default(Idle);

                // ── Idle ──
                // Quiescent leaf. Receives host commands via
                // next_command, routes RxStart → RxArming (entering
                // RxSession super-state), Tx → snapshot_tx_from_idle +
                // TxCadPrep (entering TxSession), RxStop → respond_ok
                // (idempotent ack).
                state Idle {
                    entry: announce_idle;
                    during: next_command(commands, incoming_tag, pending_tx);
                    on(CmdRxStart) => stage_rx_start;
                    on(CmdRxStart) => RxArming;
                    on(CmdTx) => stage_tx_from_idle;
                    on(CmdTx) => TxCadPrep;
                    on(CmdRxStop) => respond_ok;
                }

                // ── RxSession ──
                // Super-state owning the cancel-safe IRQ pump. Active
                // across all RX leaf states. Per hsmc spec §3, the
                // during runs while ANY descendant is active. `irq_pump`
                // is composed entirely of cancel-safe lora-phy
                // primitives (`wait_for_irq`, `get_irq_state`,
                // `clear_irq_status`), so dropping it on RxSession exit
                // never wedges the chip — `RxDraining`'s
                // `enter_standby` cleans up RX-mode regardless.
                //
                // `cache_rx_pkt` builds the PacketParams once on entry
                // so the per-IRQ `collect_rx_packet` action doesn't
                // rebuild it. `clear_rx_pkt` clears on exit.
                state RxSession {
                    entry: cache_rx_pkt;
                    exit: clear_rx_pkt;
                    during: irq_pump(lora);
                    default(RxArming);

                    // Brief: chip prep for RX. Same role as the old
                    // StartingRx — entry calls try_start_rx_hw which
                    // ultimately emits StartRxOk or StartRxFailed.
                    state RxArming {
                        entry: try_start_rx_hw;
                        on(StartRxOk) => respond_ok_start_rx, RxActive;
                        on(StartRxFailed) => respond_err_start_rx, Idle;
                    }

                    // Continuous-RX active state. The parent's
                    // `irq_pump` drives `Dio1RxDone` /
                    // `Dio1RxIntermediate` events; the local
                    // `next_command` during pulls host commands
                    // concurrently. CmdTx / CmdRxStop / ConfigApplied
                    // all route through `RxDraining` (with the
                    // destination stashed in ctx.rx_drain_dest) so the
                    // chip is cleanly walked back to standby BEFORE
                    // any TX prep — honors the SX1262's "RX → standby
                    // → CAD/TX" sequencing per datasheet §8.
                    state RxActive {
                        entry: announce_entered_rx;
                        during: next_command(commands, incoming_tag, pending_tx);
                        on(Dio1RxDone) => collect_rx_packet;
                        on(Dio1RxIntermediate) => RxActive;
                        on(PacketReceived(rx: RxPayload)) => record_packet;
                        on(IrqWaitFailed) => RxResuming;
                        on(CmdRxStart) => respond_ok;
                        on(CmdRxStop) => stash_drain_dest_idle;
                        on(CmdRxStop) => respond_ok;
                        on(CmdRxStop) => RxDraining;
                        on(CmdTx) => stash_tx_from_rx;
                        on(CmdTx) => stash_drain_dest_tx;
                        on(CmdTx) => RxDraining;
                        // Spec §3.5: a SET_CONFIG issued while
                        // receiving clears the RX ring (handled by
                        // apply_set_config) but auto-rearms RX with the
                        // new parameters via the drain → resume path.
                        on(ConfigApplied) => stash_drain_dest_resuming_rx;
                        on(ConfigApplied) => RxDraining;
                    }

                    // Cancel-safe RX wind-down. Entry calls
                    // `enter_standby()` (single SPI; cancel-safe;
                    // forces the chip out of RX-continuous and clears
                    // pending DIO assertions per SX1262 §8.5). Then
                    // emits `RxDrained`; routing action reads
                    // ctx.rx_drain_dest to decide where to go.
                    state RxDraining {
                        entry: drain_rx_hw;
                        during: pending();
                        on(RxDrained) => route_after_rx_drain;
                        on(RxRouteToIdle) => Idle;
                        on(RxRouteToTx) => TxCadPrep;
                        on(RxRouteToResume) => RxResuming;
                    }

                    // Bounded-retry RX re-arm. Entered after every TX
                    // while we were originally receiving, on RX driver
                    // errors (IrqWaitFailed), and on mid-session
                    // SET_CONFIG via the drain path. If
                    // `try_start_rx_hw` succeeds we land cleanly in
                    // RxActive; if it fails we back off and retry,
                    // only conceding to Idle after MAX_RESUME_RETRIES
                    // attempts (chip is genuinely wedged at that point).
                    state RxResuming {
                        entry: try_start_rx_hw;
                        during: pending();
                        on(StartRxOk) => reset_resume_retries, RxActive;
                        on(StartRxFailed) => bump_resume_retries;
                        on(ResumeRxRetry) => RxResuming;
                        on(ResumeRxExhausted) => Idle;
                    }
                }

                // ── TxSession ──
                // Super-state owning the TX lifecycle. Entry stages
                // the in-flight TX (cad_attempts_max, last_tx_tag,
                // tx_outcome=None). Exit guarantees a tag-correlated
                // reply: respond_tx_failed is a no-op if the success
                // path consumed last_tx_tag via .take(), so it's safe
                // to fire unconditionally.
                //
                // Reset arriving mid-TX is STASHED via
                // mark_pending_reset (chart can't preempt the cancel-
                // unsafe lora.cad/tx without violating lora-phy's
                // contract). The deferred Reset is honored by
                // route_tx_outcome AFTER natural TX completion, by
                // emitting RadioInput::Reset which Operational handles
                // normally — the chip is in a known-clean state at
                // that point.
                state TxSession {
                    entry: snapshot_pending_tx;
                    exit: respond_tx_failed;
                    exit: announce_packet_tx;
                    on(Reset) => mark_pending_reset;
                    on(CmdTx) => reject_tx_busy;
                    default(TxCadPrep);

                    // SPI prep for CAD. `prep_for_cad_act` calls
                    // `lora.prepare_for_cad()` (wrapped in lora_setup
                    // — single SPI sequence; cancel-safe drop). On
                    // success emits CadPrepOk; on the SKIP_CAD flag
                    // emits TxCadSkipped (jumps directly to TxPrep);
                    // on failure emits TransmitFailed → TxFanout.
                    state TxCadPrep {
                        entry: clear_cad_overlong;
                        entry: prep_for_cad_act;
                        during: pending();
                        on(CadPrepOk) => TxCadWait;
                        on(TxCadSkipped) => TxPrep;
                        on(TransmitFailed) => stage_tx_failed;
                        on(TransmitFailed) => TxFanout;
                    }

                    // The cancel-UNSAFE `lora.cad()` runs in this
                    // state's during. ZERO event handlers other than
                    // those `cad_one_step` itself emits on natural
                    // completion — ensures the chart cannot drop the
                    // future via a preempting transition.
                    //
                    // Wedge timers fire WHILE the during runs; they
                    // only set context flags (mark_attempt_overlong /
                    // mark_cad_must_abort), no transition. The during
                    // checks `cad_must_abort` at its next cancel-safe
                    // wakeup (start of next cad_one_step call) and
                    // bails cleanly.
                    state TxCadWait {
                        on(after Duration::from_millis(CAD_OVERLONG_MS)) => mark_attempt_overlong;
                        on(after Duration::from_millis(CAD_MUST_ABORT_MS)) => mark_cad_must_abort;
                        during: cad_one_step(lora, mdltn, cad_must_abort);
                        on(CadIdle) => check_overlong_route_idle;
                        on(CadBusy) => check_overlong_route_busy;
                        on(TransmitFailed) => check_overlong_route_fail;
                        on(WedgeDetected) => Recovering;
                        on(TxCadGoToPrep) => TxPrep;
                        on(TxCadGoToBackoff) => TxBackoff;
                        on(TxCadGoToFanoutFailed) => stage_tx_failed;
                        on(TxCadGoToFanoutFailed) => TxFanout;
                    }

                    // Inter-CAD jittered sleep. `bump_cad_attempt`
                    // increments cad_attempts_used; if it reaches
                    // cad_attempts_max, emits CadBackoffMaxed →
                    // ChannelBusy fanout. Otherwise the during sleeps
                    // and emits CadBackoffDone → next CAD prep.
                    state TxBackoff {
                        entry: bump_cad_attempt;
                        during: jittered_sleep_during(cad_retry_delay_us);
                        on(CadBackoffDone) => TxCadPrep;
                        on(CadBackoffMaxed) => stage_tx_busy;
                        on(CadBackoffMaxed) => TxFanout;
                    }

                    // SPI prep for TX. Same shape as TxCadPrep —
                    // single SPI sequence wrapped in lora_setup, on
                    // success emits TxPrepOk → TxOnAir.
                    state TxPrep {
                        entry: prep_for_tx_act;
                        during: pending();
                        on(TxPrepOk) => TxOnAir;
                        on(TransmitFailed) => stage_tx_failed;
                        on(TransmitFailed) => TxFanout;
                    }

                    // The cancel-UNSAFE `lora.tx()` runs in this
                    // state's during. Same sandboxing pattern as
                    // TxCadWait. On success carries airtime in
                    // TxOnAirOk; the chart fires respond_tx_done with
                    // it before staging TxOutcomeKind::Transmitted and
                    // routing through TxFanout.
                    state TxOnAir {
                        on(after Duration::from_millis(TX_MUST_ABORT_MS)) => mark_tx_must_abort;
                        during: tx_complete(lora, tx_must_abort);
                        on(TxOnAirOk { airtime_us: u32 }) => respond_tx_done;
                        on(TxOnAirOk) => stage_tx_transmitted;
                        on(TxOnAirOk) => TxFanout;
                        on(TransmitFailed) => stage_tx_failed;
                        on(TransmitFailed) => TxFanout;
                        on(WedgeDetected) => Recovering;
                    }

                    // Terminal of the TX sub-FSM. `route_tx_outcome`
                    // reads ctx.from_rx_session and ctx.pending_reset
                    // to decide whether to emit TxFanoutToIdle (post-
                    // TX-from-Idle), TxFanoutToRx (post-TX-from-RX
                    // → re-arm RX), or Reset (pending Reset stashed
                    // mid-TX, now safe to honor).
                    state TxFanout {
                        entry: route_tx_outcome;
                        during: pending();
                        on(TxFanoutToIdle) => Idle;
                        on(TxFanoutToRx) => RxResuming;
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
    // Cancel-safe by chart construction: Booting has zero event
    // handlers, so `LoRa::new` cannot be dropped mid-await by a
    // preempting transition. lora-phy's "must run to completion"
    // contract is honored. If the chip's BUSY pin is stuck high on
    // first power-on, this future hangs forever — Stephen's rule 5
    // ("hang in a state longer than we want is acceptable"). The
    // user power-cycles to escape; there is no chart-level recovery
    // for cold init that can't talk to the chip at all.
    match LoRa::new(p.driver, false, p.delay).await {
        Ok(l) => {
            *lora = Some(l);
            // Successful cold init clears any stale wedge flag from
            // a prior Recovering attempt — fresh chip, fresh state.
            let _ = take_wedge_flag();
            info!("radio: cold init OK");
            RadioInput::BootOk
        }
        Err(e) => {
            error!("radio: cold init failed: {:?}", defmt::Debug2Format(&e));
            RadioInput::BootFailed
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
    // Cancel-safe by chart construction: Recovering has no event
    // handlers other than the events this during emits on natural
    // completion. The chart cannot drop `lora.init()` mid-await via a
    // preempting transition. lora.init() is the cancel-UNSAFE NRESET +
    // cold-init sequence; we honor lora-phy's "must run to completion"
    // contract by letting it run to completion. If the chip is wedged
    // hard enough that lora.init() never returns, the chart sits in
    // Recovering forever — Stephen's rule 5 ("hang around in a state
    // longer than we want is acceptable").
    match lora.init().await {
        Ok(()) => {
            // Successful re-init clears any stale wedge flag from the
            // prior failure that landed us here.
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

// ── Cancel-safe RX/TX during functions ──────────────────────────────
//
// These replace `next_packet` (RX) and `perform_tx` (TX). Each one is
// either purely composed of cancel-safe lora-phy primitives (irq_pump,
// jittered_sleep_during) or hosts a single cancel-unsafe call inside a
// chart state with no preempting handlers (cad_one_step, tx_complete).
// Per hsmc README rule §6 ("durings are cancelled before exits"), the
// chart can drop these futures only when their state is exited; we
// build the chart shape so that exit cannot happen mid-await for the
// cancel-unsafe ones.

/// `during:` for `RxSession` super-state. Awaits the chip's DIO1 IRQ,
/// reads the IRQ state, clears, emits the corresponding chart event.
/// Cancel-safe by construction: every `.await` is on a cancel-safe
/// lora-phy primitive (`wait_for_irq` is GPIO-sense; `get_irq_state`
/// and `clear_irq_status` are single SPI ops). Drop at any point
/// leaves the chip in RX-continuous with the IRQ either just-cleared
/// or pending — `RxDraining`'s `enter_standby` cleans up either way.
async fn irq_pump(lora: &mut Option<LoRaDriver>) -> RadioInput {
    let Some(lora) = lora.as_mut() else {
        // Defensive: chart invariant says RxSession is unreachable
        // when lora is None, but if it slips, fail to RxResuming.
        return RadioInput::IrqWaitFailed;
    };
    if lora.wait_for_irq().await.is_err() {
        return RadioInput::IrqWaitFailed;
    }
    let st = match lora.get_irq_state().await {
        Ok(s) => s,
        Err(_) => return RadioInput::IrqWaitFailed,
    };
    let _ = lora.clear_irq_status().await;
    match st {
        Some(IrqState::Done) => RadioInput::Dio1RxDone,
        Some(IrqState::PreambleReceived) | None => RadioInput::Dio1RxIntermediate,
    }
}

/// `during:` for `TxCadWait`. Hosts the cancel-UNSAFE `lora.cad()`
/// call AS-IS — sandboxed because `TxCadWait` declares ZERO event
/// handlers other than the events this function emits on natural
/// completion. The chart cannot drop this future via a transition
/// because no handler can fire in TxCadWait.
///
/// Wedge escalation: chart timer at `CAD_MUST_ABORT_MS` sets
/// `cad_must_abort` (no transition). On the during's NEXT call (after
/// natural completion of the current `cad()`), it sees the flag,
/// cancel-safely walks the chip back to standby, returns
/// `TransmitFailed`. The chart THEN routes to Recovering AFTER the
/// during has cleanly returned — honors lora-phy's "must run to
/// completion" contract.
async fn cad_one_step(
    lora: &mut Option<LoRaDriver>,
    mdltn: &mut Option<ModulationParams>,
    cad_must_abort: &mut bool,
) -> RadioInput {
    let Some(lora) = lora.as_mut() else {
        return RadioInput::TransmitFailed;
    };
    let Some(m) = mdltn.as_ref() else {
        return RadioInput::TransmitFailed;
    };
    if *cad_must_abort {
        // Wedge timer set this between calls. Cancel-safely abort:
        // enter_standby is a single SPI command. Then return failure;
        // chart's check_overlong_* sees overlong flag and routes to
        // Recovering.
        let _ = lora.enter_standby().await;
        *cad_must_abort = false;
        return RadioInput::TransmitFailed;
    }
    match lora.cad(m).await {
        Ok(true) => RadioInput::CadBusy,
        Ok(false) => RadioInput::CadIdle,
        Err(_) => RadioInput::TransmitFailed,
    }
}

/// `during:` for `TxOnAir`. Same sandboxing as `cad_one_step`: hosts
/// the cancel-UNSAFE `lora.tx()` call AS-IS in a state with no
/// preempting handlers. On natural completion emits
/// `TxOnAirOk { airtime_us }` carrying the elapsed-on-air time for
/// the host's TX_DONE response.
async fn tx_complete(
    lora: &mut Option<LoRaDriver>,
    tx_must_abort: &mut bool,
) -> RadioInput {
    let Some(lora) = lora.as_mut() else {
        return RadioInput::TransmitFailed;
    };
    if *tx_must_abort {
        let _ = lora.enter_standby().await;
        *tx_must_abort = false;
        return RadioInput::TransmitFailed;
    }
    let start = embassy_time::Instant::now();
    match lora.tx().await {
        Ok(()) => RadioInput::TxOnAirOk {
            airtime_us: embassy_time::Instant::now().duration_since(start).as_micros() as u32,
        },
        Err(_) => RadioInput::TransmitFailed,
    }
}

/// `during:` for `TxBackoff`. Pure timer — sleep for the jittered
/// per-CAD-retry interval, then emit `CadBackoffDone`. Cancel-safe
/// (timer drop is harmless).
async fn jittered_sleep_during(cad_retry_delay_us: &mut u32) -> RadioInput {
    let entropy = embassy_time::Instant::now().as_ticks() as u32;
    let jittered_us = jittered_sleep_us(*cad_retry_delay_us, entropy);
    embassy_time::Timer::after(embassy_time::Duration::from_micros(jittered_us as u64)).await;
    RadioInput::CadBackoffDone
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
        // Inline what the old `start_rx` helper did: build mod params,
        // build packet params, call prepare_for_rx wrapped in lora_setup.
        // ALL these primitives are cancel-safe (single SPI sequences);
        // the cancel-unsafe `complete_rx`/`rx()` loop is now the chart's
        // RxActive `irq_pump` + `collect_rx_packet` (cancel-safe by
        // construction).
        let result: Result<(), RadioError> = {
            let ctx: &mut RadioContext = &mut *self;
            match ctx.lora.as_mut() {
                Some(lora) => match to_lora_mod_params(lora, &cfg) {
                    Ok(mdltn) => match lora.create_rx_packet_params(
                        cfg.preamble_len,
                        matches!(cfg.header_mode, LoRaHeaderMode::Implicit),
                        MAX_OTA_PAYLOAD as u8,
                        cfg.payload_crc,
                        cfg.iq_invert,
                        &mdltn,
                    ) {
                        Ok(pkt) => {
                            let prep = lora_setup(
                                "try_start_rx_hw.prepare_for_rx",
                                lora.prepare_for_rx(RxMode::Continuous, &mdltn, &pkt),
                            )
                            .await;
                            // prepare_for_rx only stages mod/packet/IRQ
                            // params; the chip is not actually receiving
                            // until start_rx issues the do_rx command.
                            // Both are single SPI sequences (cancel-safe);
                            // chain them so a successful prepare always
                            // arms the receiver before we declare RX up.
                            match prep {
                                Ok(()) => {
                                    let started = lora_setup(
                                        "try_start_rx_hw.start_rx",
                                        lora.start_rx(),
                                    )
                                    .await;
                                    if started.is_ok() {
                                        // Cache so RxSession's
                                        // collect_rx_packet doesn't rebuild.
                                        ctx.mdltn = Some(mdltn);
                                        ctx.rx_pkt = Some(pkt);
                                    }
                                    started
                                }
                                Err(e) => Err(e),
                            }
                        }
                        Err(e) => Err(e),
                    },
                    Err(e) => Err(e),
                },
                None => Err(RadioError::Busy),
            }
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

    // ── Cancel-safe RX/TX subtree actions ───────────────────────────

    /// Idle's `on(CmdTx)` handler chain — sets `from_rx_session = false`
    /// before the transition to TxCadPrep. snapshot_pending_tx (TxSession
    /// entry) reads this to know whether to fanout to Idle (true) or
    /// RxResuming (false) post-TX.
    async fn stage_tx_from_idle(&mut self) {
        self.from_rx_session = false;
    }

    /// RxActive's `on(CmdTx)` handler chain — sets `from_rx_session =
    /// true`. The post-TX fanout will route back to RxResuming so the
    /// RX session resumes after the TX completes (spec §3.5: TX during
    /// RX returns to RX).
    async fn stash_tx_from_rx(&mut self) {
        self.from_rx_session = true;
    }

    /// RxActive handlers stash the post-drain destination in context.
    /// `route_after_rx_drain` reads this to decide where to go after
    /// `enter_standby` cleans up the chip.
    async fn stash_drain_dest_idle(&mut self) {
        self.rx_drain_dest = Some(RxDrainDest::Idle);
    }

    async fn stash_drain_dest_tx(&mut self) {
        self.rx_drain_dest = Some(RxDrainDest::Tx);
    }

    async fn stash_drain_dest_resuming_rx(&mut self) {
        self.rx_drain_dest = Some(RxDrainDest::ResumingRx);
    }

    /// RxSession entry. Builds and caches the RxPacketParams from
    /// the active config so collect_rx_packet doesn't have to rebuild
    /// them on every IRQ. Skipped if no config (defensive — chart
    /// shouldn't reach RxSession without Configured).
    async fn cache_rx_pkt(&mut self) {
        // Split-borrow self via DerefMut to get disjoint field
        // mutable borrows (lora) + immutable reads (config) + Option
        // writes (mdltn, rx_pkt) — all on the same RadioContext.
        let ctx: &mut RadioContext = &mut *self;
        let Some(lora) = ctx.lora.as_mut() else {
            return;
        };
        let Some(cfg) = ctx.config else {
            return;
        };
        let Ok(mdltn) = to_lora_mod_params(lora, &cfg) else {
            return;
        };
        if let Ok(pkt) = lora.create_rx_packet_params(
            cfg.preamble_len,
            matches!(cfg.header_mode, LoRaHeaderMode::Implicit),
            MAX_OTA_PAYLOAD as u8,
            cfg.payload_crc,
            cfg.iq_invert,
            &mdltn,
        ) {
            ctx.rx_pkt = Some(pkt);
            ctx.mdltn = Some(mdltn);
        }
    }

    /// RxSession exit. Drops the cached RxPacketParams; next entry
    /// to RxSession will rebuild them via cache_rx_pkt.
    async fn clear_rx_pkt(&mut self) {
        self.rx_pkt = None;
    }

    /// RxActive's Dio1RxDone handler. Reads the just-arrived packet
    /// via lora.get_rx_result (cancel-safe SPI read pair), builds the
    /// RxPayload, and emits PacketReceived which routes to record_packet.
    async fn collect_rx_packet(&mut self) {
        // Build the payload inside an inner scope where ctx holds the
        // disjoint field borrows. Then emit AFTER ctx is dropped.
        let rx_opt: Option<RxPayload> = {
            let ctx: &mut RadioContext = &mut *self;
            match (ctx.lora.as_mut(), ctx.rx_pkt.as_ref()) {
                (Some(lora), Some(pkt)) => {
                    match lora.get_rx_result(pkt, &mut ctx.rx_buf).await {
                        Ok((len, status)) => {
                            let ts_us = embassy_time::Instant::now().as_micros();
                            let copy_len = (len as usize).min(MAX_OTA_PAYLOAD);
                            let mut data: HVec<u8, MAX_OTA_PAYLOAD> = HVec::new();
                            let _ = data.extend_from_slice(&ctx.rx_buf[..copy_len]);
                            Some(RxPayload {
                                rssi_tenths_dbm: (status.rssi as i32 * 10) as i16,
                                snr_tenths_db: (status.snr as i32 * 10) as i16,
                                freq_err_hz: 0,
                                timestamp_us: ts_us,
                                crc_valid: true,
                                packets_dropped: 0,
                                origin: RxOrigin::Ota,
                                data,
                            })
                        }
                        Err(_) => None,
                    }
                }
                _ => None,
            }
        };
        match rx_opt {
            Some(rx) => {
                let _ = self.emit(RadioInput::PacketReceived(rx));
            }
            None => {
                let _ = self.emit(RadioInput::IrqWaitFailed);
            }
        }
    }

    /// RxDraining entry. Cancel-safely walks the chip out of RX into
    /// Standby before the chart proceeds to whichever drain destination
    /// was stashed in ctx.rx_drain_dest. enter_standby is single SPI;
    /// per SX1262 §8.5 entering STDBY_RC clears any pending DIO
    /// assertions, leaving the chip in a known-clean state for the
    /// next CAD or RX prep.
    async fn drain_rx_hw(&mut self) {
        if let Some(lora) = self.lora.as_mut() {
            let _ = lora_setup("drain_rx_hw.enter_standby", lora.enter_standby()).await;
        }
        let _ = self.emit(RadioInput::RxDrained);
    }

    /// RxDraining's RxDrained handler. Reads ctx.rx_drain_dest and
    /// emits the corresponding routing event. The chart's transition
    /// arms in RxDraining match these.
    async fn route_after_rx_drain(&mut self) {
        let dest = self.rx_drain_dest.take();
        match dest {
            Some(RxDrainDest::Idle) => {
                let _ = self.emit(RadioInput::RxRouteToIdle);
            }
            Some(RxDrainDest::Tx) => {
                let _ = self.emit(RadioInput::RxRouteToTx);
            }
            Some(RxDrainDest::ResumingRx) => {
                let _ = self.emit(RadioInput::RxRouteToResume);
            }
            None => {
                // Defensive: chart bug if we got here without a dest.
                // Fall back to Idle so chart isn't stuck in RxDraining.
                warn!("route_after_rx_drain: no rx_drain_dest stashed; falling back to Idle");
                let _ = self.emit(RadioInput::RxRouteToIdle);
            }
        }
    }

    /// TxSession entry. Captures the in-flight TX's tag, reads the
    /// CAD retry budget for the current config, and resets per-session
    /// counters. Patches pending_tx.config from the active config
    /// (next_command staged a dummy). Announces RadioEvent::EnteredTx
    /// for the display.
    async fn snapshot_pending_tx(&mut self) {
        if let (Some(cfg), Some(pt)) = (self.config, self.pending_tx.as_mut()) {
            pt.config = cfg;
        }
        if let Some(pt) = self.pending_tx.as_ref() {
            self.last_tx_tag = Some(
                core::num::NonZeroU16::new(pt.tag)
                    .expect("PendingTx.tag was 0 — next_command should have rejected"),
            );
        }
        let (max, _sleep) = cad_retries(self.cad_retry_delay_us);
        self.cad_attempts_max = max;
        self.cad_attempts_used = 0;
        self.cad_attempt_overlong = false;
        self.cad_must_abort = false;
        self.tx_must_abort = false;
        self.tx_outcome = None;
        self.pending_reset = false;
        self.announce(RadioEvent::EnteredTx);
    }

    /// TxSession's `on(Reset) => mark_pending_reset` handler. Records
    /// that a Reset arrived during a long TX op. The chart cannot
    /// honor Reset immediately (would drop the cancel-unsafe lora.cad
    /// or lora.tx future). route_tx_outcome (TxFanout entry) checks
    /// this flag AFTER natural TX completion and emits RadioInput::Reset
    /// then — Operational's `on(Reset) => reset_to_unconfigured`
    /// handles it normally with the chip in a clean state.
    async fn mark_pending_reset(&mut self) {
        self.pending_reset = true;
        info!("Reset received during TX; deferred until current TX completes");
    }

    /// TxSession's `on(CmdTx)` handler. A second CmdTx arrived while
    /// the first is still in flight. Reject the new one with a clean
    /// TxDone{Failed} so the host gets a tag-correlated reply.
    async fn reject_tx_busy(&mut self) {
        if let Some(pt) = self.pending_tx.take() {
            // No "TxDone with failure" code in the protocol; use
            // Err(ERadio) which the host already handles as
            // "TX did not complete".
            self.send(pt.tag, DeviceMessage::Err(ErrorCode::ERadio))
                .await;
        }
    }

    /// TxCadPrep entry — clears overlong/abort flags so the new CAD
    /// attempt starts with a clean slate. Chart timers in TxCadWait
    /// will set them again if needed.
    async fn clear_cad_overlong(&mut self) {
        self.cad_attempt_overlong = false;
        self.cad_must_abort = false;
    }

    /// TxCadPrep entry. Either skips CAD (skip_cad flag) and emits
    /// TxCadSkipped → TxPrep, or calls lora.prepare_for_cad and emits
    /// CadPrepOk → TxCadWait. SPI failure escalates via take_wedge_flag.
    async fn prep_for_cad_act(&mut self) {
        let skip = self.pending_tx.as_ref().is_some_and(|pt| pt.skip_cad);
        if skip {
            let _ = self.emit(RadioInput::TxCadSkipped);
            return;
        }
        let result = {
            let ctx: &mut RadioContext = &mut *self;
            match (ctx.lora.as_mut(), ctx.mdltn.as_ref()) {
                (Some(lora), Some(mdltn)) => {
                    lora_setup("prep_for_cad_act", lora.prepare_for_cad(mdltn)).await
                }
                _ => Err(RadioError::Busy),
            }
        };
        match result {
            Ok(()) => {
                let _ = self.emit(RadioInput::CadPrepOk);
            }
            Err(_) => {
                if take_wedge_flag() {
                    let _ = self.emit(RadioInput::WedgeDetected);
                } else {
                    let _ = self.emit(RadioInput::TransmitFailed);
                }
            }
        }
    }

    /// Stages the TX outcome as Failed AND sends the Err(ERadio) reply
    /// to the host. After this fires, last_tx_tag is None so
    /// TxSession's exit:respond_tx_failed safety net is a no-op.
    async fn stage_tx_failed(&mut self) {
        self.tx_outcome = Some(TxOutcomeKind::Failed);
        if let Some(tag) = self.last_tx_tag.take() {
            self.send(tag.get(), DeviceMessage::Err(ErrorCode::ERadio))
                .await;
        }
    }

    /// Stages the TX outcome as ChannelBusy AND sends the
    /// TxDone{ChannelBusy} reply. Same tag-consume + safety-net pattern
    /// as stage_tx_failed.
    async fn stage_tx_busy(&mut self) {
        self.tx_outcome = Some(TxOutcomeKind::ChannelBusy);
        if let Some(tag) = self.last_tx_tag.take() {
            let td = TxDonePayload {
                result: TxResult::ChannelBusy,
                airtime_us: 0,
            };
            self.send(tag.get(), DeviceMessage::TxDone(td)).await;
        }
    }

    /// Stages the TX outcome as Transmitted. The TxOnAirOk handler
    /// chain calls respond_tx_done BEFORE this fires, so last_tx_tag
    /// is already None — this just records the outcome for fanout
    /// routing.
    async fn stage_tx_transmitted(&mut self) {
        self.tx_outcome = Some(TxOutcomeKind::Transmitted);
    }

    /// TxCadWait timer (CAD_OVERLONG_MS) action. Soft warning that
    /// the in-flight CAD has been running longer than the typical
    /// per-attempt budget. Sets context flag — NO transition. The
    /// during keeps running. Routing actions check this flag at the
    /// during's natural completion to decide whether to escalate.
    async fn mark_attempt_overlong(&mut self) {
        if !self.cad_attempt_overlong {
            warn!(
                "TxCadWait: CAD attempt running >{=u64} ms (chip may be wedged)",
                CAD_OVERLONG_MS
            );
            self.cad_attempt_overlong = true;
        }
    }

    /// TxCadWait timer (CAD_MUST_ABORT_MS) action. Hard signal that
    /// the in-flight CAD has been running unacceptably long. Sets
    /// cad_must_abort which `cad_one_step` checks at its NEXT
    /// cancel-safe wakeup; on seeing the flag it walks the chip back
    /// to standby (cancel-safely) and returns TransmitFailed. The
    /// chart THEN routes to Recovering.
    async fn mark_cad_must_abort(&mut self) {
        warn!("TxCadWait: CAD attempt exceeded {=u64} ms — must-abort signaled", CAD_MUST_ABORT_MS);
        self.cad_must_abort = true;
    }

    /// TxOnAir timer (TX_MUST_ABORT_MS) action. Same pattern as
    /// mark_cad_must_abort but for the TX phase.
    async fn mark_tx_must_abort(&mut self) {
        warn!("TxOnAir: TX exceeded {=u64} ms — must-abort signaled", TX_MUST_ABORT_MS);
        self.tx_must_abort = true;
    }

    /// TxCadWait CadIdle handler. Channel was clear → proceed to TxPrep.
    /// Overlong flag doesn't matter here: we got a real answer in time
    /// for CAD even if it was slow.
    async fn check_overlong_route_idle(&mut self) {
        let _ = self.emit(RadioInput::TxCadGoToPrep);
    }

    /// TxCadWait CadBusy handler. Channel saw activity → back off and
    /// retry, OR fanout with ChannelBusy if max attempts reached.
    /// `bump_cad_attempt` (TxBackoff entry) does the cad_attempts_used
    /// bump and emits CadBackoffMaxed if exhausted.
    async fn check_overlong_route_busy(&mut self) {
        let _ = self.emit(RadioInput::TxCadGoToBackoff);
    }

    /// TxCadWait TransmitFailed handler. CAD itself errored. If the
    /// overlong flag is set, escalate to WedgeDetected (the chip is
    /// almost certainly stuck). Otherwise just fail this TX with a
    /// clean error reply.
    async fn check_overlong_route_fail(&mut self) {
        if self.cad_attempt_overlong {
            let _ = self.emit(RadioInput::WedgeDetected);
        } else {
            let _ = self.emit(RadioInput::TxCadGoToFanoutFailed);
        }
    }

    /// TxBackoff entry. Increments cad_attempts_used; if exhausted,
    /// emits CadBackoffMaxed which routes to TxFanout via stage_tx_busy.
    /// Otherwise the during (jittered_sleep_during) runs and emits
    /// CadBackoffDone → next CAD attempt.
    async fn bump_cad_attempt(&mut self) {
        self.cad_attempts_used = self.cad_attempts_used.saturating_add(1);
        if self.cad_attempts_used >= self.cad_attempts_max {
            let _ = self.emit(RadioInput::CadBackoffMaxed);
        }
    }

    /// TxPrep entry. Calls lora.prepare_for_tx (writes payload, sets
    /// TX power, etc.). On success → TxOnAir. On failure → TxFanout
    /// via stage_tx_failed.
    async fn prep_for_tx_act(&mut self) {
        let result = {
            let ctx: &mut RadioContext = &mut *self;
            // Need lora (mut), mdltn (ref), pending_tx (ref) — all
            // disjoint fields of ctx so split-borrow works.
            match (ctx.lora.as_mut(), ctx.mdltn.as_ref(), ctx.pending_tx.as_ref()) {
                (Some(lora), Some(mdltn), Some(pt)) => {
                    let cfg = pt.config;
                    match lora.create_tx_packet_params(
                        cfg.preamble_len,
                        matches!(cfg.header_mode, LoRaHeaderMode::Implicit),
                        cfg.payload_crc,
                        cfg.iq_invert,
                        mdltn,
                    ) {
                        Ok(mut tx_pkt) => {
                            lora_setup(
                                "prep_for_tx_act",
                                lora.prepare_for_tx(
                                    mdltn,
                                    &mut tx_pkt,
                                    cfg.tx_power_dbm as i32,
                                    &pt.data,
                                ),
                            )
                            .await
                        }
                        Err(e) => Err(e),
                    }
                }
                _ => Err(RadioError::Busy),
            }
        };
        match result {
            Ok(()) => {
                let _ = self.emit(RadioInput::TxPrepOk);
            }
            Err(_) => {
                if take_wedge_flag() {
                    let _ = self.emit(RadioInput::WedgeDetected);
                } else {
                    let _ = self.emit(RadioInput::TransmitFailed);
                }
            }
        }
    }

    /// TxFanout entry. Reads ctx.from_rx_session and ctx.pending_reset
    /// to decide where to go after TX completion (success, busy, or
    /// failed — all paths converge here). Emits one of three routing
    /// events the chart consumes via TxFanout's transition arms.
    async fn route_tx_outcome(&mut self) {
        // If a Reset was deferred during the in-flight TX, honor it
        // now — the chip is in a clean post-TX state, so reset_to_
        // unconfigured can run without the cancel-safety hazard.
        if self.pending_reset {
            self.pending_reset = false;
            let _ = self.emit(RadioInput::Reset);
            return;
        }
        if self.from_rx_session {
            let _ = self.emit(RadioInput::TxFanoutToRx);
        } else {
            let _ = self.emit(RadioInput::TxFanoutToIdle);
        }
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

async fn reconfigure_radio(lora: &mut LoRaDriver, cfg: &LoRaConfig) -> Result<(), RadioError> {
    lora_setup("reconfigure_radio.enter_standby", lora.enter_standby()).await?;
    // Apply the host-supplied LoRa sync word. The protocol field is a
    // u16 holding the SX126x register pair in big-endian order
    // (RadioLib convention: 0x1424 == byte 0x12 == LoRa "private" /
    // MeshCore; 0x3444 == byte 0x34 == LoRaWAN "public"). lora-phy's
    // set_lora_sync_word takes the underlying single-byte form, so
    // undo the nibble encoding: convert_sync_word(byte) puts the
    // chip's mandatory 0x4 in the low nibble of each register byte,
    // and the high nibble of each register byte holds the original
    // byte's high/low nibble respectively.
    let [hi, lo] = cfg.sync_word.to_be_bytes();
    let sync_byte = (hi & 0xF0) | (lo >> 4);
    lora_setup(
        "reconfigure_radio.set_lora_sync_word",
        lora.set_lora_sync_word(sync_byte),
    )
    .await?;
    // Calibrate image etc. via a throwaway create_modulation_params call
    // — inside the SET_CONFIG OK window so first TX/RX is fast.
    let _ = to_lora_mod_params(lora, cfg)?;
    Ok(())
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
        mdltn: None,
        rx_pkt: None,
        cad_attempts_used: 0,
        cad_attempts_max: 0,
        cad_attempt_overlong: false,
        cad_must_abort: false,
        tx_must_abort: false,
        from_rx_session: false,
        tx_outcome: None,
        pending_reset: false,
        rx_drain_dest: None,
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
