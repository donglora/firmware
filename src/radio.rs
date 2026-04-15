//! LoRa radio task: SX1262 state machine driven by host commands.
//!
//! Owns the radio peripheral exclusively. Receives [`Command`]s from the
//! host task, drives the SX1262 via [`lora_phy`], and sends [`Response`]s
//! back. Publishes typed [`RadioEvent`]s to the display task at every state
//! transition and RF event — no shared status snapshot, no diffing consumer.
//!
//! # State machine
//!
//! ```text
//! Idle ──StartRx──► Receiving ──StopRx──► Idle
//!   │                    │
//!   └──Transmit──► Transmitting ──TxDone──► (previous state)
//! ```

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_futures::select::{select, Either};
use embassy_time::Delay;
use lora_phy::mod_params::RadioError;
use lora_phy::{LoRa, RxMode};

use crate::board::{Board, LoRaBoard, RadioDriver, RadioParts};
use crate::channel::{CommandChannel, RadioEvent, RadioEventChannel, ResponseChannel};
use crate::protocol::{self, Command, ErrorCode, RadioConfig, Response};

const MAX_PAYLOAD: usize = protocol::MAX_PAYLOAD;

type Radio = LoRa<RadioDriver, Delay>;

// ── Fixed LoRa radio parameters ─────────────────────────────────────
const IMPLICIT_HEADER: bool = false;
const CRC_ON: bool = true;
const IQ_INVERTED: bool = false;

/// Internal mode, just for tracking what the radio is doing right now.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Idle,
    Receiving,
    Transmitting,
}

/// Runtime state not exposed outside this module.
struct RadioState {
    mode: Mode,
    config: Option<RadioConfig>,
}

impl RadioState {
    fn new() -> Self {
        Self {
            mode: Mode::Idle,
            config: None,
        }
    }
}

/// Emit a radio event to the display subscriber. Best-effort: if the
/// downstream queue is full we'd rather drop a UI update than stall the
/// radio task.
fn emit(events: &RadioEventChannel, ev: RadioEvent) {
    if events.try_send(ev).is_err() {
        warn!("radio event dropped: downstream queue full");
    }
}

#[task]
pub async fn radio_task(
    parts: RadioParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    events: &'static RadioEventChannel,
) {
    let mut state = RadioState::new();

    let mut lora = match LoRa::new(parts.driver, false, parts.delay).await {
        Ok(l) => l,
        Err(e) => {
            error!("radio init failed: {}", e);
            responses
                .send(Response::Error(ErrorCode::InvalidConfig))
                .await;
            // Radio is non-functional — respond to commands but can't do RF.
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
    };

    info!("radio initialized");
    let mut rx_buf = [0u8; MAX_PAYLOAD];

    loop {
        if state.mode == Mode::Receiving {
            // `Mode::Receiving` is only set after `start_rx` succeeded with
            // a validated config; we rely on that here.
            let cfg = match state.config {
                Some(c) => c,
                None => {
                    warn!("BUG: receiving without config, returning to idle");
                    state.mode = Mode::Idle;
                    emit(events, RadioEvent::Idle);
                    continue;
                }
            };

            match select(rx_once(&mut lora, &cfg, &mut rx_buf), commands.receive()).await {
                Either::First(rx_result) => match rx_result {
                    Ok((len, pkt_status)) => {
                        emit(
                            events,
                            RadioEvent::PacketRx {
                                rssi: pkt_status.rssi,
                                snr: Some(pkt_status.snr),
                            },
                        );

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

                        if let Err(e) = start_rx(&mut lora, &cfg).await {
                            warn!("restart RX failed: {}", e);
                            state.mode = Mode::Idle;
                            emit(events, RadioEvent::Idle);
                            let _ = responses.try_send(Response::Error(ErrorCode::RadioBusy));
                        }
                    }
                    Err(e) => {
                        warn!("RX error: {}", e);
                        if start_rx(&mut lora, &cfg).await.is_err() {
                            state.mode = Mode::Idle;
                            emit(events, RadioEvent::Idle);
                            let _ = responses.try_send(Response::Error(ErrorCode::RadioBusy));
                        }
                    }
                },
                Either::Second(cmd) => {
                    handle_cmd(cmd, &mut lora, &mut state, responses, events).await;
                }
            }
        } else {
            let cmd = commands.receive().await;
            handle_cmd(cmd, &mut lora, &mut state, responses, events).await;
        }
    }
}



async fn handle_cmd(
    cmd: Command,
    lora: &mut Radio,
    state: &mut RadioState,
    responses: &ResponseChannel,
    events: &RadioEventChannel,
) {
    match cmd {
        Command::Ping => {
            responses.send(Response::Pong).await;
        }
        Command::GetConfig => {
            if let Some(cfg) = state.config {
                responses.send(Response::Config(cfg)).await;
            } else {
                responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
            }
        }
        Command::SetConfig(cfg) => {
            if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
                warn!("SetConfig rejected: {}", reason);
                responses
                    .send(Response::Error(ErrorCode::InvalidConfig))
                    .await;
            } else {
                let resolved = cfg.resolve(Board::TX_POWER_RANGE);
                state.config = Some(resolved);
                emit(events, RadioEvent::ConfigChanged(resolved));
                responses.send(Response::Ok).await;
            }
        }
        Command::StartRx => {
            if let Some(cfg) = state.config {
                match start_rx(lora, &cfg).await {
                    Ok(()) => {
                        state.mode = Mode::Receiving;
                        emit(events, RadioEvent::EnteredRx);
                        responses.send(Response::Ok).await;
                    }
                    Err(e) => {
                        warn!("StartRx failed: {}", e);
                        responses
                            .send(Response::Error(ErrorCode::InvalidConfig))
                            .await;
                    }
                }
            } else {
                responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
            }
        }
        Command::StopRx => {
            let _ = lora.enter_standby().await;
            if state.mode != Mode::Idle {
                state.mode = Mode::Idle;
                emit(events, RadioEvent::Idle);
            }
            let _ = responses.try_send(Response::Ok);
        }
        Command::DisplayOn | Command::DisplayOff | Command::GetMac => {}
        Command::Transmit { config, payload } => {
            let tx_config = config
                .map(|c| c.resolve(Board::TX_POWER_RANGE))
                .or(state.config);
            let Some(cfg) = tx_config else {
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

            let was_receiving = state.mode == Mode::Receiving;
            state.mode = Mode::Transmitting;
            emit(events, RadioEvent::EnteredTx);

            let tx_result = do_tx(lora, &cfg, &payload).await;
            match tx_result {
                Ok(()) => {
                    emit(events, RadioEvent::PacketTx);
                    responses.send(Response::TxDone).await;
                }
                Err(e) => {
                    warn!("TX failed: {}", e);
                    responses.send(Response::Error(ErrorCode::TxTimeout)).await;
                }
            }

            // Restore previous mode — and announce the transition explicitly
            // so downstream consumers don't have to infer it.
            if was_receiving {
                let rx_cfg = state.config.unwrap_or(cfg);
                match start_rx(lora, &rx_cfg).await {
                    Ok(()) => {
                        state.mode = Mode::Receiving;
                        emit(events, RadioEvent::EnteredRx);
                    }
                    Err(e) => {
                        warn!("post-TX RX restart failed: {}", e);
                        state.mode = Mode::Idle;
                        emit(events, RadioEvent::Idle);
                    }
                }
            } else {
                state.mode = Mode::Idle;
                emit(events, RadioEvent::Idle);
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
    lora: &mut Radio,
    cfg: &RadioConfig,
) -> Result<lora_phy::mod_params::ModulationParams, RadioError> {
    lora.create_modulation_params(to_sf(cfg.sf), to_bw(cfg.bw), to_cr(cfg.cr), cfg.freq_hz)
}

async fn start_rx(lora: &mut Radio, cfg: &RadioConfig) -> Result<(), RadioError> {
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
    lora: &mut Radio,
    cfg: &RadioConfig,
    buf: &mut [u8],
) -> Result<(u8, lora_phy::mod_params::PacketStatus), RadioError> {
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

async fn do_tx(lora: &mut Radio, cfg: &RadioConfig, payload: &[u8]) -> Result<(), RadioError> {
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
