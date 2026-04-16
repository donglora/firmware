//! LoRa radio task: SX1262 statechart driven by host commands.
//!
//! ```text
//! Radio
//! ├── Unconfigured   (default)   no config cached → no RF ops possible
//! └── Configured
//!     ├── Idle          (default)  ready, waiting
//!     ├── StartingRx               brief: HW init in response to host
//!     │                            CmdStartRx; sends Ok/Error then →
//!     │                            Receiving (Ok) or Idle (Err)
//!     ├── Receiving                continuous RX
//!     ├── ResumingRx               brief: post-TX HW re-arm; silently
//!     │                            → Receiving (Ok) or Idle (Err)
//!     └── Transmitting
//!         ├── FromIdle  (default)  TxDone/TxFailed → Idle
//!         └── FromRx                TxDone → ResumingRx, TxFailed → Idle
//! ```
//!
//! The task body is a single `machine.run().await`. All I/O lives inside
//! state-scoped `during:` activities:
//!
//! * Root-level `during: next_command(commands)` continuously reads the
//!   host CommandChannel and converts each command into a `RadioInput`
//!   event.
//! * `Receiving.during: next_packet(lora, rx_buf)` runs the SX1262 RX
//!   and yields `PacketReceived` events.
//! * `TransmittingFromIdle/FromRx.during: perform_tx(lora, pending_tx)`
//!   drives `do_tx` and yields `TransmitDone`/`TransmitFailed`.

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Delay;
use hsmc::statechart;
use lora_phy::mod_params::RadioError;
use lora_phy::{LoRa, RxMode};

use crate::board::{Board, LoRaBoard, RadioDriver, RadioParts};
use crate::channel::{CommandChannel, RadioEvent, RadioEventChannel, ResponseChannel};
use crate::protocol::{self, Command, ErrorCode, RadioConfig, Response};

const MAX_PAYLOAD: usize = protocol::MAX_PAYLOAD;

type LoRaDriver = LoRa<RadioDriver, Delay>;

// ── Fixed LoRa radio parameters ─────────────────────────────────────
const IMPLICIT_HEADER: bool = false;
const CRC_ON: bool = true;
const IQ_INVERTED: bool = false;

/// A pending TX request staged by an incoming `Transmit` command, consumed
/// by the `perform_tx` during activity when the machine enters a
/// `Transmitting.*` state. `config` is `None` when the command asked to
/// reuse the cached config; the `stage_tx` handler resolves the effective
/// config.
#[derive(Debug, Clone)]
pub struct PendingTx {
    pub config: Option<RadioConfig>,
    pub payload: heapless::Vec<u8, MAX_PAYLOAD>,
}

/// Events driving the radio statechart. Host commands arrive as `Cmd*`
/// variants via the `next_command` during; low-level I/O results arrive
/// as internal variants via the `next_packet` / `perform_tx` durings.
#[derive(Debug, Clone)]
pub enum RadioInput {
    // Commands flowing in from the host via CommandChannel:
    CmdPing,
    CmdGetConfig,
    CmdSetConfig(RadioConfig),
    CmdStartRx,
    CmdStopRx,
    CmdTransmit(PendingTx),

    // Internal events from the RX during activity:
    PacketReceived {
        rssi: i16,
        snr: i16,
        len: u8,
        data: heapless::Vec<u8, MAX_PAYLOAD>,
    },
    RxFailed(&'static str),

    // Internal events from `try_start_rx_hw` (the entry action of
    // StartingRx and ResumingRx). Two distinct states consume these,
    // and their differing transitions are how the chart encodes whether
    // the original trigger was a host CmdStartRx (must respond) or a
    // post-TX RX re-arm (must NOT respond).
    StartRxOk,
    StartRxFailed(&'static str),

    // Internal events from the TX during activity:
    TransmitDone,
    TransmitFailed(&'static str),
}

/// Root context. Owns the LoRa driver, the RX scratch buffer, the cached
/// configuration, the outbound event + response channels, and a staging
/// slot for pending TX requests.
pub struct RadioContext {
    pub commands: &'static CommandChannel,
    pub responses: &'static ResponseChannel,
    pub events: &'static RadioEventChannel,
    pub lora: LoRaDriver,
    pub rx_buf: [u8; MAX_PAYLOAD],
    pub config: Option<RadioConfig>,
    pub pending_tx: Option<PendingTx>,
}

impl RadioContext {
    fn announce(&self, ev: RadioEvent) {
        if self.events.try_send(ev).is_err() {
            warn!("radio event dropped: downstream queue full");
        }
    }
}

statechart! {
    Radio {
        context: RadioContext;
        events: RadioInput;
        default(Unconfigured);

        // Commands handled the same way in every state. The actual command
        // read — `during: next_command(commands)` — is placed only on states
        // where commands are safe to interleave (NOT on Transmitting.*:
        // canceling perform_tx mid-flight would abort a LoRa TX).
        on(CmdPing) => respond_pong;
        on(CmdGetConfig) => respond_get_config;

        state Unconfigured {
            during: next_command(commands);
            on(CmdSetConfig(cfg: RadioConfig)) => cache_config;
            on(CmdSetConfig) => Configured;
            // Pre-configuration, only Ping/GetConfig make sense.
            on(CmdStartRx) => respond_not_configured;
            on(CmdStopRx) => respond_not_configured;
            on(CmdTransmit) => respond_not_configured;
        }

        state Configured {
            on(CmdSetConfig(cfg: RadioConfig)) => cache_config;
            default(Idle);

            state Idle {
                entry: announce_idle;
                during: next_command(commands);
                on(CmdStartRx) => StartingRx;
                on(CmdTransmit(p: PendingTx)) => stage_tx;
                on(CmdTransmit) => TransmittingFromIdle;
                on(CmdStopRx) => respond_ok;
            }

            // Brief intermediate state: a host `StartRx` is in flight,
            // hardware is being configured. `try_start_rx_hw` (entry)
            // emits `StartRxOk` or `StartRxFailed`; the transitions from
            // here are what tell the host whether RX actually started.
            // No `during: next_command` — this state is self-driven and
            // we don't want to interleave other commands with the ack.
            state StartingRx {
                entry: try_start_rx_hw;
                on(StartRxOk) => respond_ok, Receiving;
                on(StartRxFailed(err: &'static str)) => respond_start_rx_error;
                on(StartRxFailed) => Idle;
            }

            state Receiving {
                entry: announce_entered_rx;
                during: next_command(commands);
                during: next_packet(lora, rx_buf, config);
                // Idempotent ack: a redundant StartRx (same client re-issuing,
                // or another client's request that mux still chose to forward)
                // gets `Ok` without re-touching hardware.
                on(CmdStartRx) => respond_ok;
                on(PacketReceived {
                    rssi: i16,
                    snr: i16,
                    len: u8,
                    data: heapless::Vec<u8, MAX_PAYLOAD>,
                }) => record_packet;
                on(RxFailed(err: &'static str)) => announce_rx_failed;
                on(RxFailed) => Idle;
                on(CmdStopRx) => stop_rx_hw, respond_ok;
                on(CmdStopRx) => Idle;
                on(CmdTransmit(p: PendingTx)) => stage_tx;
                on(CmdTransmit) => TransmittingFromRx;
            }

            // Brief intermediate state: a TX from `Receiving` just
            // finished and the radio needs to be put back into RX mode.
            // Same `try_start_rx_hw` entry as `StartingRx`, but the
            // transitions DO NOT send a response — `respond_tx_done` was
            // already sent by the TransmittingFromRx → here transition.
            state ResumingRx {
                entry: try_start_rx_hw;
                on(StartRxOk) => Receiving;
                on(StartRxFailed) => Idle;
            }

            // Transmitting.* deliberately has no `next_command` during: the
            // TX operation inside `perform_tx` cannot be safely cancelled
            // by an incoming host command. Commands arriving during TX
            // queue on the CommandChannel and are processed by the next
            // state that reads them (Idle or Receiving).
            state Transmitting {
                entry: announce_entered_tx;
                exit: announce_packet_tx;
                default(TransmittingFromIdle);

                state TransmittingFromIdle {
                    during: perform_tx(lora, pending_tx);
                    on(TransmitDone) => respond_tx_done;
                    on(TransmitDone) => Idle;
                    on(TransmitFailed(err: &'static str)) => respond_tx_failed;
                    on(TransmitFailed) => Idle;
                }

                state TransmittingFromRx {
                    during: perform_tx(lora, pending_tx);
                    on(TransmitDone) => respond_tx_done;
                    on(TransmitDone) => ResumingRx;
                    on(TransmitFailed(err: &'static str)) => respond_tx_failed;
                    on(TransmitFailed) => Idle;
                }
            }
        }
    }
}

// ── during: activities ──────────────────────────────────────────────

/// Bridges the host's `CommandChannel` (typed `Command`) into the
/// machine's event stream. Each awaited command becomes one `RadioInput`.
async fn next_command(commands: &mut &'static CommandChannel) -> RadioInput {
    let cmd = commands.receive().await;
    match cmd {
        Command::Ping => RadioInput::CmdPing,
        Command::GetConfig => RadioInput::CmdGetConfig,
        Command::SetConfig(cfg) => RadioInput::CmdSetConfig(cfg),
        Command::StartRx => RadioInput::CmdStartRx,
        Command::StopRx => RadioInput::CmdStopRx,
        Command::Transmit { config, payload } => {
            // Mark the moment we learned about the Transmit command.
            // This runs before the latch engages, so it's visible as a
            // distinct LED value (18) when the firmware is sitting in
            // Receiving (5) waiting for something to process — if LED
            // stays on 5, the command never arrived at the firmware
            // (mux / UART delay); if LED flashes through 18 then deeper,
            // the command arrived normally.
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(18);
            // The `stage_tx` handler resolves the effective config from
            // either the inline `config` or the cached one.
            RadioInput::CmdTransmit(PendingTx { config, payload })
        }
        Command::DisplayOn | Command::DisplayOff | Command::GetMac => {
            // These are dispatched elsewhere in the host; if they land
            // here it's a routing bug. Map to Ping → Pong for minimum
            // harm (better: surface an error; there's no "unknown
            // command" variant today).
            RadioInput::CmdPing
        }
    }
}

/// Runs one iteration of continuous RX. Produces a `PacketReceived` event
/// on success or `RxFailed` on error. The radio was configured for
/// continuous RX by `try_start_rx_hw` (entry action of `StartingRx` /
/// `ResumingRx`) before the chart entered `Receiving`; each iteration
/// of this during awaits the next packet.
async fn next_packet(
    lora: &mut LoRaDriver,
    rx_buf: &mut [u8; MAX_PAYLOAD],
    config: &mut Option<RadioConfig>,
) -> RadioInput {
    let Some(cfg) = config.as_ref() else {
        return RadioInput::RxFailed("no cached config");
    };
    let mdltn = match modulation_params(lora, cfg) {
        Ok(m) => m,
        Err(_) => return RadioInput::RxFailed("mdltn"),
    };
    let pkt = match lora.create_rx_packet_params(
        cfg.preamble_len,
        IMPLICIT_HEADER,
        MAX_PAYLOAD as u8,
        CRC_ON,
        IQ_INVERTED,
        &mdltn,
    ) {
        Ok(p) => p,
        Err(_) => return RadioInput::RxFailed("pkt_params"),
    };
    match lora.rx(&pkt, rx_buf).await {
        Ok((len, status)) => {
            let copy_len = (len as usize).min(MAX_PAYLOAD);
            let mut data: heapless::Vec<u8, MAX_PAYLOAD> = heapless::Vec::new();
            let _ = data.extend_from_slice(&rx_buf[..copy_len]);
            RadioInput::PacketReceived {
                rssi: status.rssi,
                snr: status.snr,
                len,
                data,
            }
        }
        Err(_e) => RadioInput::RxFailed("rx"),
    }
}

/// Drives one TX operation using a staged `PendingTx`. Produces
/// `TransmitDone` on success or `TransmitFailed` on error.
async fn perform_tx(lora: &mut LoRaDriver, pending: &mut Option<PendingTx>) -> RadioInput {
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(6);
    let Some(tx) = pending.take() else {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(15);
        return RadioInput::TransmitFailed("no pending tx");
    };
    let Some(cfg) = tx.config.as_ref() else {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(15);
        return RadioInput::TransmitFailed("no effective config");
    };
    match do_tx(lora, cfg, &tx.payload).await {
        Ok(()) => {
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(10);
            RadioInput::TransmitDone
        }
        Err(_) => {
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(15);
            RadioInput::TransmitFailed("tx")
        }
    }
}

// ── Actions ─────────────────────────────────────────────────────────

impl RadioActions for RadioActionContext<'_> {
    // ── responses ────────────────────────────────────────────────────
    async fn respond_pong(&mut self) {
        self.responses.send(Response::Pong).await;
    }

    async fn respond_get_config(&mut self) {
        match self.config {
            Some(cfg) => self.responses.send(Response::Config(cfg)).await,
            None => {
                self.responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
            }
        }
    }

    async fn respond_ok(&mut self) {
        // try_send: StopRx after RX with full response channel is
        // tolerable — host is disconnected.
        let _ = self.responses.try_send(Response::Ok);
    }

    async fn respond_not_configured(&mut self) {
        self.responses
            .send(Response::Error(ErrorCode::NotConfigured))
            .await;
    }

    async fn respond_tx_done(&mut self) {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(11);
        self.responses.send(Response::TxDone).await;
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(12);
    }

    async fn respond_tx_failed(&mut self, _err: &'static str) {
        self.responses
            .send(Response::Error(ErrorCode::TxTimeout))
            .await;
    }

    // ── state bookkeeping ─────────────────────────────────────────────
    async fn cache_config(&mut self, cfg: RadioConfig) {
        if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
            warn!("SetConfig rejected: {}", reason);
            self.responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
            return;
        }
        let resolved = cfg.resolve(Board::TX_POWER_RANGE);
        self.config = Some(resolved);
        self.announce(RadioEvent::ConfigChanged(resolved));
        self.responses.send(Response::Ok).await;
    }

    async fn stage_tx(&mut self, p: PendingTx) {
        let effective = p.config.or(self.config);
        let Some(cfg) = effective else {
            self.responses
                .send(Response::Error(ErrorCode::NotConfigured))
                .await;
            return;
        };
        if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
            warn!("TX config rejected: {}", reason);
            self.responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
            return;
        }
        let resolved = cfg.resolve(Board::TX_POWER_RANGE);
        self.pending_tx = Some(PendingTx {
            config: Some(resolved),
            payload: p.payload,
        });
    }

    // ── RX hardware ──────────────────────────────────────────────────

    /// Entry action for `StartingRx` and `ResumingRx`. Performs HW init
    /// and emits `StartRxOk` on success or `StartRxFailed` on failure.
    /// Whether the host receives a Response::Ok / Response::Error is
    /// decided by the parent state (StartingRx responds; ResumingRx is
    /// silent — `respond_tx_done` already went out).
    async fn try_start_rx_hw(&mut self) {
        let Some(cfg) = self.config else {
            error!("try_start_rx_hw: no config cached — state invariant broken");
            let _ = self.emit(RadioInput::StartRxFailed("no config"));
            return;
        };
        match start_rx(&mut self.lora, &cfg).await {
            Ok(()) => {
                let _ = self.emit(RadioInput::StartRxOk);
            }
            Err(e) => {
                warn!("start_rx failed: {}", e);
                #[cfg(feature = "debug-checkpoint")]
                crate::debug_blink::set(17);
                let _ = self.emit(RadioInput::StartRxFailed("start_rx"));
            }
        }
    }

    async fn respond_start_rx_error(&mut self, _err: &'static str) {
        self.responses
            .send(Response::Error(ErrorCode::InvalidConfig))
            .await;
    }

    async fn stop_rx_hw(&mut self) {
        let _ = self.lora.enter_standby().await;
    }

    // ── announcements ────────────────────────────────────────────────
    async fn announce_idle(&mut self) {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(4);
        self.announce(RadioEvent::Idle);
    }
    async fn announce_entered_rx(&mut self) {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(5);
        self.announce(RadioEvent::EnteredRx);
    }
    async fn announce_entered_tx(&mut self) {
        self.announce(RadioEvent::EnteredTx);
    }
    async fn announce_packet_tx(&mut self) {
        self.announce(RadioEvent::PacketTx);
    }
    async fn announce_rx_failed(&mut self, _err: &'static str) {
        self.announce(RadioEvent::Idle);
    }

    // ── packet bookkeeping ────────────────────────────────────────────
    async fn record_packet(
        &mut self,
        rssi: i16,
        snr: i16,
        _len: u8,
        data: heapless::Vec<u8, MAX_PAYLOAD>,
    ) {
        self.announce(RadioEvent::PacketRx {
            rssi,
            snr: Some(snr),
        });
        let response = Response::RxPacket {
            rssi,
            snr,
            payload: data,
        };
        if self.responses.try_send(response).is_err() {
            warn!("RX packet dropped: response channel full");
        }
    }
}

// ── Debug-checkpoint mapping (see `crate::debug_blink`) ─────────────
//
// Boot / steady states (latch is INACTIVE — values overwrite normally):
//   1  boot: radio_task entered
//   2  boot: LoRa::new ok (driver is live)
//   3  boot: machine constructed, about to enter run()
//   4  in Idle state (announce_idle ran)
//   5  in Receiving state (announce_entered_rx ran — HW is in RX)
//
// TX phases 6–15 (linear; latch ENGAGES only on `set(6)`; only deeper
// TX-phase writes can update; 14/15 are terminal and freeze the latch):
//   6  TX phase 1: entered perform_tx
//   7  TX phase 2: in CAD (prepare_for_cad / cad loop)
//   8  TX phase 3: in prepare_for_tx (SPI config write)
//   9  TX phase 4: awaiting tx() completion (on-air transmit)
//  10  TX phase 5: do_tx returned Ok (back in perform_tx)
//  11  TX phase 6: respond_tx_done handler entered
//  12  TX phase 7: responses.send(TxDone) completed (radio side done)
//  13  TX phase 8: host_task received the response from channel (set in host/uart.rs)
//  14  TX phase 9: host_task wrote response to UART — TERMINAL success
//  15  TX errored somewhere — TERMINAL error
//
// Boot-time errors (can only be seen before any TX engages the latch):
//  16  LoRa::new failed at boot (degraded_loop branch)
//  17  try_start_rx_hw failed (radio HW init returned Err)

#[task]
pub async fn radio_task(
    parts: RadioParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    events: &'static RadioEventChannel,
) {
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(1);

    // Init the radio BEFORE the machine exists in any meaningful way —
    // a failure here never touches the statechart; we just run a
    // Ping-only loop forever.
    let lora = match LoRa::new(parts.driver, false, parts.delay).await {
        Ok(l) => {
            info!("radio initialized");
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(2);
            l
        }
        Err(e) => {
            error!("radio init failed: {}", e);
            #[cfg(feature = "debug-checkpoint")]
            crate::debug_blink::set(16);
            responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
            degraded_loop(commands, responses).await;
            return;
        }
    };

    let ctx = RadioContext {
        commands,
        responses,
        events,
        lora,
        rx_buf: [0u8; MAX_PAYLOAD],
        config: None,
        pending_tx: None,
    };

    // `run()` under the embassy feature requires a bound channel even when
    // no external task injects events. Radio inputs come from the root
    // `during: next_command(commands)` activity and from internal durings;
    // this static is kept solely to satisfy the constructor.
    static RADIO_INTERNAL_CH: Channel<CriticalSectionRawMutex, RadioInput, 8> = Channel::new();
    let mut machine = Radio::new(ctx, &RADIO_INTERNAL_CH);
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(3);
    let _ = machine.run().await;
}

/// Init-failure fallback. Answers Ping; everything else errors.
async fn degraded_loop(commands: &CommandChannel, responses: &ResponseChannel) {
    loop {
        let cmd = commands.receive().await;
        if let Command::Ping = cmd {
            responses.send(Response::Pong).await;
        } else {
            responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
        }
    }
}

// ── LoRa helpers ────────────────────────────────────────────────────

fn to_bw(bw: protocol::Bandwidth) -> lora_phy::mod_params::Bandwidth {
    use lora_phy::mod_params::Bandwidth::*;
    match bw {
        protocol::Bandwidth::Khz7 => _7KHz,
        protocol::Bandwidth::Khz10 => _10KHz,
        protocol::Bandwidth::Khz15 => _15KHz,
        protocol::Bandwidth::Khz20 => _20KHz,
        protocol::Bandwidth::Khz31 => _31KHz,
        protocol::Bandwidth::Khz41 => _41KHz,
        protocol::Bandwidth::Khz62 => _62KHz,
        protocol::Bandwidth::Khz125 => _125KHz,
        protocol::Bandwidth::Khz250 => _250KHz,
        protocol::Bandwidth::Khz500 => _500KHz,
    }
}

fn to_sf(sf: u8) -> lora_phy::mod_params::SpreadingFactor {
    use lora_phy::mod_params::SpreadingFactor::*;
    match sf {
        5 => _5,
        6 => _6,
        7 => _7,
        8 => _8,
        9 => _9,
        10 => _10,
        11 => _11,
        12 => _12,
        _ => {
            warn!("BUG: invalid SF {} passed validation", sf);
            _7
        }
    }
}

fn to_cr(cr: u8) -> lora_phy::mod_params::CodingRate {
    use lora_phy::mod_params::CodingRate::*;
    match cr {
        5 => _4_5,
        6 => _4_6,
        7 => _4_7,
        8 => _4_8,
        _ => {
            warn!("BUG: invalid CR {} passed validation", cr);
            _4_5
        }
    }
}

fn modulation_params(
    lora: &mut LoRaDriver,
    cfg: &RadioConfig,
) -> Result<lora_phy::mod_params::ModulationParams, RadioError> {
    lora.create_modulation_params(to_sf(cfg.sf), to_bw(cfg.bw), to_cr(cfg.cr), cfg.freq_hz)
}

async fn start_rx(lora: &mut LoRaDriver, cfg: &RadioConfig) -> Result<(), RadioError> {
    let mdltn = modulation_params(lora, cfg)?;
    let pkt = lora.create_rx_packet_params(
        cfg.preamble_len,
        IMPLICIT_HEADER,
        MAX_PAYLOAD as u8,
        CRC_ON,
        IQ_INVERTED,
        &mdltn,
    )?;
    lora.prepare_for_rx(RxMode::Continuous, &mdltn, &pkt).await
}

const CAD_MAX_RETRIES: u8 = 10;

async fn do_tx(lora: &mut LoRaDriver, cfg: &RadioConfig, payload: &[u8]) -> Result<(), RadioError> {
    let mdltn = modulation_params(lora, cfg)?;

    if cfg.cad_enabled() {
        #[cfg(feature = "debug-checkpoint")]
        crate::debug_blink::set(7);
        lora.prepare_for_cad(&mdltn).await?;
        for _ in 0..CAD_MAX_RETRIES {
            if !lora.cad(&mdltn).await? {
                break;
            }
            embassy_time::Timer::after_millis(10).await;
            lora.prepare_for_cad(&mdltn).await?;
        }
    }

    let mut tx_pkt = lora.create_tx_packet_params(
        cfg.preamble_len,
        IMPLICIT_HEADER,
        CRC_ON,
        IQ_INVERTED,
        &mdltn,
    )?;
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(8);
    lora.prepare_for_tx(&mdltn, &mut tx_pkt, cfg.tx_power_dbm as i32, payload)
        .await?;
    #[cfg(feature = "debug-checkpoint")]
    crate::debug_blink::set(9);
    lora.tx().await
}
