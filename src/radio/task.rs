//! LoRa radio task: SX1262 state machine driven by host commands.
//!
//! Owns the radio peripheral exclusively. Receives [`Command`]s from the USB
//! task, drives the SX1262 via [`lora_phy`], and sends [`Response`]s back.
//! Publishes [`RadioStatus`] to the display task via a watch channel.
//!
//! # State machine
//!
//! ```text
//! Idle ──StartRx──► Receiving ──StopRx──► Idle
//!   │                    │
//!   └──Transmit──► Transmitting ──TxDone──► (previous state)
//! ```
//!
//! # Invariants
//!
//! - This task never panics. All errors are reported to the host or logged.
//! - Config is validated before use (see [`RadioConfig::validate`]).

use defmt::{error, info, warn};
use embassy_executor::task;
use embassy_futures::select::{select, Either};
use embassy_time::Delay;
use lora_phy::mod_params::RadioError;
use lora_phy::{LoRa, RxMode};

use crate::board::{Board, LoRaBoard, RadioDriver, RadioParts};
use crate::channel::{CommandChannel, RadioState, RadioStatus, ResponseChannel, StatusWatch};
use crate::protocol::{self, Command, ErrorCode, RadioConfig, Response};

const MAX_PAYLOAD: usize = protocol::MAX_PAYLOAD;
// TX power range comes from the board trait: Board::TX_POWER_RANGE

type Radio = LoRa<RadioDriver, Delay>;

// ── Fixed LoRa radio parameters ─────────────────────────────────────
//
// These are intentionally not host-configurable. They are sensible
// defaults for a general-purpose LoRa pipe. Making them configurable
// would add protocol complexity for marginal benefit.

/// Explicit header mode (variable-length packets).
const IMPLICIT_HEADER: bool = false;

/// Enable CRC on received/transmitted packets.
const CRC_ON: bool = true;

/// Standard IQ polarity (not inverted).
const IQ_INVERTED: bool = false;

// ── Task entry point ────────────────────────────────────────────────

#[task]
pub async fn radio_task(
    parts: RadioParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    status: &'static StatusWatch,
) {
    let mut state = RadioStatus::default();
    status.sender().send(state.clone());

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
        if state.state == RadioState::Receiving {
            // Defensive: config must be Some when Receiving. If not, recover.
            let cfg = match state.config {
                Some(c) => c,
                None => {
                    warn!("BUG: receiving without config, returning to idle");
                    state.state = RadioState::Idle;
                    status.sender().send(state.clone());
                    continue;
                }
            };

            match select(rx_once(&mut lora, &cfg, &mut rx_buf), commands.receive()).await {
                Either::First(rx_result) => match rx_result {
                    Ok((len, pkt_status)) => {
                        state.rx_count = state.rx_count.wrapping_add(1);
                        state.last_rssi = Some(pkt_status.rssi);
                        state.last_snr = Some(pkt_status.snr);
                        status.sender().send(state.clone());

                        let copy_len = (len as usize).min(MAX_PAYLOAD);
                        if (len as usize) > copy_len {
                            warn!("RX payload truncated: {} > {}", len, MAX_PAYLOAD);
                        }
                        let mut payload = heapless::Vec::new();
                        // copy_len <= MAX_PAYLOAD == Vec capacity, so this cannot fail.
                        let _ = payload.extend_from_slice(&rx_buf[..copy_len]);

                        responses
                            .send(Response::RxPacket {
                                rssi: pkt_status.rssi,
                                snr: pkt_status.snr,
                                payload,
                            })
                            .await;

                        if let Err(e) = start_rx(&mut lora, &cfg).await {
                            warn!("restart RX failed: {}", e);
                            state.state = RadioState::Idle;
                            status.sender().send(state.clone());
                            responses.send(Response::Error(ErrorCode::RadioBusy)).await;
                        }
                    }
                    Err(e) => {
                        warn!("RX error: {}", e);
                        if start_rx(&mut lora, &cfg).await.is_err() {
                            state.state = RadioState::Idle;
                            status.sender().send(state.clone());
                            responses.send(Response::Error(ErrorCode::RadioBusy)).await;
                        }
                    }
                },
                Either::Second(cmd) => {
                    handle_cmd(cmd, &mut lora, &mut state, responses, status).await;
                }
            }
        } else {
            let cmd = commands.receive().await;
            handle_cmd(cmd, &mut lora, &mut state, responses, status).await;
        }
    }
}

// ── Command handler ─────────────────────────────────────────────────

async fn handle_cmd(
    cmd: Command,
    lora: &mut Radio,
    state: &mut RadioStatus,
    responses: &ResponseChannel,
    status: &StatusWatch,
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
                // Resolve TX_POWER_MAX sentinel to board's actual max
                state.config = Some(cfg.resolve(Board::TX_POWER_RANGE));
                status.sender().send(state.clone());
                responses.send(Response::Ok).await;
            }
        }
        Command::StartRx => {
            if let Some(cfg) = state.config {
                match start_rx(lora, &cfg).await {
                    Ok(()) => {
                        state.state = RadioState::Receiving;
                        status.sender().send(state.clone());
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
            // Best-effort: if standby fails, radio is in unknown state
            // but there's nothing useful we can do except continue.
            let _ = lora.enter_standby().await;
            state.state = RadioState::Idle;
            status.sender().send(state.clone());
            responses.send(Response::Ok).await;
        }
        Command::DisplayOn | Command::DisplayOff | Command::GetMac => {}
        Command::Transmit { config, payload } => {
            let tx_config = config
                .map(|c| c.resolve(Board::TX_POWER_RANGE))
                .or(state.config);
            if let Some(cfg) = tx_config {
                if let Err(reason) = cfg.validate(Board::TX_POWER_RANGE) {
                    warn!("TX config rejected: {}", reason);
                    responses
                        .send(Response::Error(ErrorCode::InvalidConfig))
                        .await;
                    return;
                }

                let was_receiving = state.state == RadioState::Receiving;
                state.state = RadioState::Transmitting;
                status.sender().send(state.clone());

                match do_tx(lora, &cfg, &payload).await {
                    Ok(()) => {
                        state.tx_count = state.tx_count.wrapping_add(1);
                        responses.send(Response::TxDone).await;
                    }
                    Err(e) => {
                        warn!("TX failed: {}", e);
                        responses.send(Response::Error(ErrorCode::TxTimeout)).await;
                    }
                }

                // Restore previous state. If we were receiving, restart RX
                // using the stored config — NOT the per-packet TX override.
                state.state = if was_receiving {
                    // state.config is always Some when was_receiving (invariant).
                    let rx_cfg = state.config.unwrap_or(cfg);
                    match start_rx(lora, &rx_cfg).await {
                        Ok(()) => RadioState::Receiving,
                        Err(e) => {
                            warn!("post-TX RX restart failed: {}", e);
                            RadioState::Idle
                        }
                    }
                } else {
                    RadioState::Idle
                };
                status.sender().send(state.clone());
            } else {
                responses
                    .send(Response::Error(ErrorCode::NotConfigured))
                    .await;
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

/// Create modulation parameters from a validated config.
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

/// Maximum CAD attempts before transmitting anyway.
const CAD_MAX_RETRIES: u8 = 10;

async fn do_tx(lora: &mut Radio, cfg: &RadioConfig, payload: &[u8]) -> Result<(), RadioError> {
    let mdltn = modulation_params(lora, cfg)?;

    // Listen-before-talk: check channel is clear before transmitting.
    if cfg.cad_enabled() {
        lora.prepare_for_cad(&mdltn).await?;
        for _ in 0..CAD_MAX_RETRIES {
            if !lora.cad(&mdltn).await? {
                break; // Channel is clear
            }
            // Channel busy — wait briefly and retry
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
