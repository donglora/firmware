//! LoRa radio task: SX1262 statechart driven by host commands.
//!
//! ```text
//! Radio
//! ├── Unconfigured   (default)   no config cached → no RF ops possible
//! └── Configured
//!     ├── Idle          (default)  ready, waiting
//!     ├── Receiving                continuous RX
//!     └── Transmitting
//!         ├── FromIdle  (default)  TransmitDone → Idle
//!         └── FromRx                TransmitDone → Receiving
//! ```
//!
//! The machine owns two pieces of radio state: which mode we're in and
//! the cached [`RadioConfig`]. Entry/exit actions broadcast typed
//! [`RadioEvent`]s to the display. RF I/O (`rx_once`, `do_tx`,
//! `enter_standby`) can't live in actions — `rx_once` blocks
//! indefinitely — so the task's main loop stays imperative and drives
//! the machine via `machine.send` + `drain`, branching on
//! `machine.current_state()`.
//!
//! The `Transmitting` substates encode "where to return after TX" as
//! state identity; no `was_receiving` bookkeeping. Radio init happens
//! **before** the machine ever steps — success falls through into the
//! default `Unconfigured`, failure early-returns into a small
//! Ping-only degraded loop that doesn't involve the machine at all.

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_futures::select::{select, Either};
use embassy_time::Delay;
use hsmc::{statechart, Duration};
use lora_phy::mod_params::{PacketStatus, RadioError};
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

/// Internal events that drive `Radio` state transitions. Kept distinct
/// from [`crate::channel::RadioEvent`], which is the display-facing
/// broadcast bus.
#[derive(Debug, Clone)]
pub enum RadioInput {
    /// Host issued `SetConfig`; driver already validated + resolved.
    ConfigSet(RadioConfig),
    /// `start_rx` returned `Ok(())` — radio is now in RX mode.
    StartRxOk,
    /// `enter_standby` completed.
    StopRxOk,
    /// About to call `do_tx` — parks the machine in `Transmitting.*`.
    TransmitBegin,
    /// `do_tx` returned (success or failure). Either way, leave Transmitting.
    TransmitDone,
    /// `start_rx` failed mid-Receiving; fall back to Idle.
    RxRestartFailed,
    /// A packet was just received off-air with these link metrics.
    PacketReceived { rssi: i16, snr: i16 },
}

/// Machine context — config cache + the outbound event channel. The
/// `LoRa` driver, RX buffer, and command-validation plumbing stay local
/// to `radio_task` because their async operations can't live inside
/// statechart actions.
pub struct RadioContext {
    events: &'static RadioEventChannel,
    config: Option<RadioConfig>,
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

        state Unconfigured {
            // Two declarations for the same event: the first catches the
            // payload to cache it, the second transitions. `ConfigSet`
            // from this state always means "first config ever".
            on(ConfigSet(cfg: RadioConfig)) => cache_config;
            on(ConfigSet) => Configured;
        }

        state Configured {
            // Re-configure from any Configured substate (driver rejects
            // mid-TX, so this never fires during Transmitting).
            on(ConfigSet(cfg: RadioConfig)) => cache_config;
            default(Idle);

            state Idle {
                entry: announce_idle;
                on(StartRxOk) => Receiving;
                on(TransmitBegin) => TransmittingFromIdle;
            }

            state Receiving {
                entry: announce_entered_rx;
                on(PacketReceived { rssi: i16, snr: i16 }) => announce_packet_rx;
                on(StopRxOk) => Idle;
                on(TransmitBegin) => TransmittingFromRx;
                on(RxRestartFailed) => Idle;
            }

            state Transmitting {
                entry: announce_entered_tx;
                exit: announce_packet_tx;
                default(TransmittingFromIdle);

                state TransmittingFromIdle {
                    on(TransmitDone) => Idle;
                }

                state TransmittingFromRx {
                    on(TransmitDone) => Receiving;
                    on(RxRestartFailed) => Idle;
                }
            }
        }
    }
}

impl RadioActions for RadioActionContext<'_> {
    async fn cache_config(&mut self, cfg: RadioConfig) {
        self.config = Some(cfg);
        self.announce(RadioEvent::ConfigChanged(cfg));
    }

    async fn announce_idle(&mut self) {
        self.announce(RadioEvent::Idle);
    }

    async fn announce_entered_rx(&mut self) {
        self.announce(RadioEvent::EnteredRx);
    }

    async fn announce_entered_tx(&mut self) {
        self.announce(RadioEvent::EnteredTx);
    }

    async fn announce_packet_rx(&mut self, rssi: i16, snr: i16) {
        self.announce(RadioEvent::PacketRx {
            rssi,
            snr: Some(snr),
        });
    }

    async fn announce_packet_tx(&mut self) {
        self.announce(RadioEvent::PacketTx);
    }
}

/// Drain the machine's internal queue. Call after every `push`.
async fn drain(m: &mut Radio) {
    while m.has_pending_events() {
        m.step(Duration::ZERO).await;
    }
}

/// Push an event into the machine's internal queue and immediately drain
/// so `current_state()` reflects the transition before we return.
/// Overflow here is a real bug — the queue is capacity 8 and we never
/// batch — so we panic rather than silently drop.
async fn push(m: &mut Radio, ev: RadioInput) {
    m.send(ev).expect("radio machine queue overflow");
    drain(m).await;
}

/// Post-TX invariant: if the machine transitioned back into `Receiving`,
/// the hardware is still in standby from `do_tx`; put it back in RX. On
/// failure, tell the machine to fall through to `Idle`.
async fn restore_rx_if_needed(machine: &mut Radio, lora: &mut LoRaDriver) {
    if machine.current_state() != RadioState::Receiving {
        return;
    }
    let Some(cfg) = machine.context().config else {
        // Can't happen: Receiving implies config is cached.
        return;
    };
    if let Err(e) = start_rx(lora, &cfg).await {
        warn!("post-TX RX restart failed: {}", e);
        push(machine, RadioInput::RxRestartFailed).await;
    }
}

#[task]
pub async fn radio_task(
    parts: RadioParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    events: &'static RadioEventChannel,
) {
    // Init the radio BEFORE the machine exists in any meaningful way —
    // a failure here never touches the statechart; we just run a
    // Ping-only loop forever.
    let mut lora = match LoRa::new(parts.driver, false, parts.delay).await {
        Ok(l) => {
            info!("radio initialized");
            l
        }
        Err(e) => {
            error!("radio init failed: {}", e);
            responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
            degraded_loop(commands, responses).await;
            return;
        }
    };

    let mut machine = Radio::new_local(RadioContext {
        events,
        config: None,
    });
    // First step enters the default path: Unconfigured.
    machine.step(Duration::ZERO).await;

    let mut rx_buf = [0u8; MAX_PAYLOAD];

    loop {
        match machine.current_state() {
            RadioState::Unconfigured | RadioState::Idle => {
                let cmd = commands.receive().await;
                handle_cmd(cmd, &mut machine, &mut lora, responses).await;
            }
            RadioState::Receiving => {
                // Config presence is a state invariant: we can't be in
                // Receiving without having passed through ConfigSet.
                let cfg = machine
                    .context()
                    .config
                    .expect("Receiving requires cached config");
                match select(
                    rx_once(&mut lora, &cfg, &mut rx_buf),
                    commands.receive(),
                )
                .await
                {
                    Either::First(rx_result) => {
                        handle_rx_result(
                            rx_result,
                            &mut machine,
                            &mut lora,
                            &cfg,
                            &mut rx_buf,
                            responses,
                        )
                        .await;
                    }
                    Either::Second(cmd) => {
                        handle_cmd(cmd, &mut machine, &mut lora, responses).await;
                    }
                }
            }
            // `Transmitting.*` is transient — entered and exited inside
            // `handle_cmd`'s Transmit branch. `Configured` and
            // `Transmitting` are composite states that `current_state()`
            // never returns once `descend_defaults` has completed. If
            // any of these land here it's a bug; log + yield so we
            // don't tight-spin.
            RadioState::TransmittingFromIdle
            | RadioState::TransmittingFromRx
            | RadioState::Configured
            | RadioState::Transmitting => {
                warn!(
                    "radio main loop reached transient/composite state; yielding"
                );
                embassy_time::Timer::after_millis(10).await;
            }
        }
    }
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

async fn handle_cmd(
    cmd: Command,
    machine: &mut Radio,
    lora: &mut LoRaDriver,
    responses: &ResponseChannel,
) {
    match cmd {
        Command::Ping => {
            responses.send(Response::Pong).await;
        }
        Command::GetConfig => match machine.context().config {
            Some(cfg) => responses.send(Response::Config(cfg)).await,
            None => {
                responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
            }
        },
        Command::SetConfig(cfg) => {
            if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
                warn!("SetConfig rejected: {}", reason);
                responses
                    .send(Response::Error(ErrorCode::InvalidConfig))
                    .await;
            } else {
                let resolved = cfg.resolve(Board::TX_POWER_RANGE);
                push(machine, RadioInput::ConfigSet(resolved)).await;
                responses.send(Response::Ok).await;
            }
        }
        Command::StartRx => {
            let Some(cfg) = machine.context().config else {
                responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
                return;
            };
            match start_rx(lora, &cfg).await {
                Ok(()) => {
                    push(machine, RadioInput::StartRxOk).await;
                    responses.send(Response::Ok).await;
                }
                Err(e) => {
                    warn!("StartRx failed: {}", e);
                    responses
                        .send(Response::Error(ErrorCode::InvalidConfig))
                        .await;
                }
            }
        }
        Command::StopRx => {
            let _ = lora.enter_standby().await;
            push(machine, RadioInput::StopRxOk).await;
            // try_send: StopRx may be sent internally on USB disconnect
            // when the host isn't draining responses.
            let _ = responses.try_send(Response::Ok);
        }
        Command::DisplayOn | Command::DisplayOff | Command::GetMac => {}
        Command::Transmit {
            config: tx_cfg,
            payload,
        } => execute_transmit(tx_cfg, &payload, machine, lora, responses).await,
    }
}

/// Validate the effective TX config, enter `Transmitting.*`, run
/// `do_tx`, and leave — restoring RX if the machine returned to
/// `Receiving`. Isolated from `handle_cmd` because it's the only
/// command that crosses the statechart in two hops and has a
/// non-trivial post-condition (RX hardware re-arm).
async fn execute_transmit(
    tx_cfg: Option<RadioConfig>,
    payload: &[u8],
    machine: &mut Radio,
    lora: &mut LoRaDriver,
    responses: &ResponseChannel,
) {
    let eff_cfg = tx_cfg
        .map(|c| c.resolve(Board::TX_POWER_RANGE))
        .or(machine.context().config);
    let Some(cfg) = eff_cfg else {
        responses
            .send(Response::Error(ErrorCode::NotConfigured))
            .await;
        return;
    };
    if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
        warn!("TX config rejected: {}", reason);
        responses
            .send(Response::Error(ErrorCode::InvalidConfig))
            .await;
        return;
    }

    // Per-state handlers pick TransmittingFromIdle vs TransmittingFromRx.
    push(machine, RadioInput::TransmitBegin).await;

    let tx_result = do_tx(lora, &cfg, payload).await;

    // Fire TransmitDone regardless — we must leave Transmitting. The
    // exit action announces PacketTx to the display.
    push(machine, RadioInput::TransmitDone).await;

    match tx_result {
        Ok(()) => responses.send(Response::TxDone).await,
        Err(e) => {
            warn!("TX failed: {}", e);
            responses
                .send(Response::Error(ErrorCode::TxTimeout))
                .await;
        }
    }

    restore_rx_if_needed(machine, lora).await;
}

async fn handle_rx_result(
    rx_result: Result<(u8, PacketStatus), RadioError>,
    machine: &mut Radio,
    lora: &mut LoRaDriver,
    cfg: &RadioConfig,
    rx_buf: &mut [u8; MAX_PAYLOAD],
    responses: &ResponseChannel,
) {
    match rx_result {
        Ok((len, pkt_status)) => {
            push(
                machine,
                RadioInput::PacketReceived {
                    rssi: pkt_status.rssi,
                    snr: pkt_status.snr,
                },
            )
            .await;

            let copy_len = (len as usize).min(MAX_PAYLOAD);
            if (len as usize) > copy_len {
                warn!("RX payload truncated: {} > {}", len, MAX_PAYLOAD);
            }
            let mut payload = heapless::Vec::new();
            let _ = payload.extend_from_slice(&rx_buf[..copy_len]);

            if responses
                .try_send(Response::RxPacket {
                    rssi: pkt_status.rssi,
                    snr: pkt_status.snr,
                    payload,
                })
                .is_err()
            {
                warn!("RX packet dropped: response channel full");
            }

            if let Err(e) = start_rx(lora, cfg).await {
                warn!("restart RX failed: {}", e);
                push(machine, RadioInput::RxRestartFailed).await;
                let _ = responses.try_send(Response::Error(ErrorCode::RadioBusy));
            }
        }
        Err(e) => {
            warn!("RX error: {}", e);
            if start_rx(lora, cfg).await.is_err() {
                push(machine, RadioInput::RxRestartFailed).await;
                let _ = responses.try_send(Response::Error(ErrorCode::RadioBusy));
            }
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

async fn rx_once(
    lora: &mut LoRaDriver,
    cfg: &RadioConfig,
    buf: &mut [u8],
) -> Result<(u8, PacketStatus), RadioError> {
    let mdltn = modulation_params(lora, cfg)?;
    let pkt = lora.create_rx_packet_params(
        cfg.preamble_len,
        IMPLICIT_HEADER,
        MAX_PAYLOAD as u8,
        CRC_ON,
        IQ_INVERTED,
        &mdltn,
    )?;
    lora.rx(&pkt, buf).await
}

const CAD_MAX_RETRIES: u8 = 10;

async fn do_tx(lora: &mut LoRaDriver, cfg: &RadioConfig, payload: &[u8]) -> Result<(), RadioError> {
    let mdltn = modulation_params(lora, cfg)?;

    if cfg.cad_enabled() {
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
    lora.prepare_for_tx(&mdltn, &mut tx_pkt, cfg.tx_power_dbm as i32, payload)
        .await?;
    lora.tx().await
}
