//! LoRa radio task: SX1262 statechart driven by DongLoRa Protocol v2 host commands.
//!
//! ```text
//! Radio
//! ├── Unconfigured   (default)   no config cached → RF ops rejected
//! └── Configured
//!     ├── Idle          (default)  ready, waiting
//!     ├── StartingRx              brief: HW init in response to host
//!     │                           RxStart; responds Ok/Err then →
//!     │                           Receiving (Ok) or Idle (Err)
//!     ├── Receiving               continuous RX
//!     ├── ResumingRx              brief: post-TX HW re-arm; silent →
//!     │                           Receiving (Ok) or Idle (Err)
//!     └── Transmitting
//!         ├── FromIdle (default)  TxDone → Idle
//!         └── FromRx              TxDone → ResumingRx
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

const CAD_MAX_RETRIES: u8 = 10;

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
}

// ── Context ─────────────────────────────────────────────────────────

pub struct RadioContext {
    pub commands: &'static CommandChannel,
    pub outbound: &'static OutboundChannel,
    pub events: &'static RadioEventChannel,
    pub info: &'static Info,
    pub lora: LoRaDriver,
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
    pub incoming_tag: u16,
    /// Tag of the TX currently on the wire. Captured on entry to
    /// `Transmitting`; consumed by `respond_tx_*` handlers.
    pub last_tx_tag: Option<u16>,
    /// Tag of a pending `RX_START`, pulled by `respond_ok_start_rx` /
    /// `respond_err_start_rx` once the chip reports back.
    pub pending_rx_start_tag: Option<u16>,
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
        default(Unconfigured);

        // Globals: PING always answers; GET_INFO always answers; Reset
        // forces Unconfigured.
        on(CmdPing) => respond_pong;
        on(CmdGetInfo) => respond_info;
        on(Reset) => reset_to_unconfigured, Unconfigured;

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
            // Re-configure from any substate (Receiving, Transmitting, …)
            // drops back to Idle. Spec §3.5: "Any continuous RX is
            // stopped. RX ring is cleared." Only successful applies
            // emit ConfigApplied, so validation failures keep state.
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
                on(StartRxOk) => respond_ok_start_rx, Receiving;
                on(StartRxFailed) => respond_err_start_rx, Idle;
            }

            state Receiving {
                entry: announce_entered_rx;
                during: next_command(commands, incoming_tag, pending_tx);
                during: next_packet(lora, rx_buf, config);
                on(CmdRxStart) => respond_ok;
                on(PacketReceived(rx: RxPayload)) => record_packet;
                on(RxFailed) => announce_rx_failed, Idle;
                on(CmdRxStop) => stop_rx_hw, respond_ok;
                on(CmdRxStop) => Idle;
                on(CmdTx) => TransmittingFromRx;
            }

            state ResumingRx {
                entry: try_start_rx_hw;
                on(StartRxOk) => Receiving;
                on(StartRxFailed) => Idle;
            }

            state Transmitting {
                entry: announce_entered_tx;
                exit: announce_packet_tx;
                default(TransmittingFromIdle);

                state TransmittingFromIdle {
                    during: perform_tx(lora, pending_tx);
                    on(TransmitDone { airtime_us: u32 }) => respond_tx_done;
                    on(TransmitDone) => Idle;
                    on(ChannelBusy) => respond_tx_channel_busy, Idle;
                    on(TransmitFailed) => respond_tx_failed, Idle;
                }

                state TransmittingFromRx {
                    during: perform_tx(lora, pending_tx);
                    on(TransmitDone { airtime_us: u32 }) => respond_tx_done;
                    on(TransmitDone) => ResumingRx;
                    on(ChannelBusy) => respond_tx_channel_busy, ResumingRx;
                    on(TransmitFailed) => respond_tx_failed, Idle;
                }
            }
        }
    }
}

// ── during: activities ──────────────────────────────────────────────

/// Read the next host command, stash its tag into context, and (for TX)
/// stage a placeholder pending-TX struct before emitting the corresponding
/// event. The `apply_set_config` / `try_start_rx_hw` / TX-staging actions
/// patch in the real config from ctx when they run.
async fn next_command(
    commands: &mut &'static CommandChannel,
    incoming_tag: &mut u16,
    pending_tx: &mut Option<PendingTx>,
) -> RadioInput {
    match commands.receive().await {
        InboundEvent::Reset => RadioInput::Reset,
        InboundEvent::Command { tag, cmd } => {
            *incoming_tag = tag;
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

async fn next_packet(
    lora: &mut LoRaDriver,
    rx_buf: &mut [u8; MAX_OTA_PAYLOAD],
    config: &mut Option<LoRaConfig>,
) -> RadioInput {
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
        Err(_) => RadioInput::RxFailed,
    }
}

async fn perform_tx(lora: &mut LoRaDriver, pending: &mut Option<PendingTx>) -> RadioInput {
    let Some(tx) = pending.take() else {
        return RadioInput::TransmitFailed;
    };
    let start = Instant::now();
    match do_tx(lora, &tx).await {
        Ok(TxOutcome::Transmitted) => RadioInput::TransmitDone {
            airtime_us: Instant::now().duration_since(start).as_micros() as u32,
        },
        Ok(TxOutcome::ChannelBusy) => RadioInput::ChannelBusy,
        Err(_) => RadioInput::TransmitFailed,
    }
}

// ── Actions ─────────────────────────────────────────────────────────

impl RadioActions for RadioActionContext<'_> {
    // ── PING / GET_INFO ───────────────────────────────────────────────

    async fn respond_pong(&mut self) {
        let tag = self.incoming_tag;
        self.send(tag, DeviceMessage::Ok(OkPayload::Empty)).await;
    }

    async fn respond_info(&mut self) {
        let tag = self.incoming_tag;
        let info = *self.info;
        self.send(tag, DeviceMessage::Ok(OkPayload::Info(info)))
            .await;
    }

    // ── SET_CONFIG ────────────────────────────────────────────────────

    async fn apply_set_config(&mut self, m: Modulation) {
        let tag = self.incoming_tag;
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
                if let Err(_e) = reconfigure_radio(&mut self.lora, &cfg).await {
                    // Hardware failure: drop to Unconfigured (spec §6.3).
                    self.config = None;
                    self.send(tag, DeviceMessage::Err(ErrorCode::ERadio)).await;
                    let _ = self.emit(RadioInput::ConfigFailedRadio);
                    return;
                }
                self.config = Some(cfg);
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
            self.send(tag, DeviceMessage::TxDone(td)).await;
        }
    }

    async fn respond_tx_channel_busy(&mut self) {
        if let Some(tag) = self.last_tx_tag.take() {
            let td = TxDonePayload {
                result: TxResult::ChannelBusy,
                airtime_us: 0,
            };
            self.send(tag, DeviceMessage::TxDone(td)).await;
        }
    }

    async fn respond_tx_failed(&mut self) {
        if let Some(tag) = self.last_tx_tag.take() {
            self.send(tag, DeviceMessage::Err(ErrorCode::ERadio)).await;
        }
    }

    // ── RX_START / RX_STOP ────────────────────────────────────────────

    async fn stage_rx_start(&mut self) {
        self.pending_rx_start_tag = Some(self.incoming_tag);
    }

    async fn try_start_rx_hw(&mut self) {
        let Some(cfg) = self.config else {
            error!("try_start_rx_hw: no config cached — state invariant broken");
            let _ = self.emit(RadioInput::StartRxFailed);
            return;
        };
        match start_rx(&mut self.lora, &cfg).await {
            Ok(()) => {
                let _ = self.emit(RadioInput::StartRxOk);
            }
            Err(e) => {
                warn!("start_rx failed: {}", e);
                let _ = self.emit(RadioInput::StartRxFailed);
            }
        }
    }

    async fn respond_ok(&mut self) {
        let tag = self.incoming_tag;
        self.send(tag, DeviceMessage::Ok(OkPayload::Empty)).await;
    }

    async fn respond_ok_start_rx(&mut self) {
        if let Some(tag) = self.pending_rx_start_tag.take() {
            self.send(tag, DeviceMessage::Ok(OkPayload::Empty)).await;
        }
    }

    async fn respond_err_start_rx(&mut self) {
        if let Some(tag) = self.pending_rx_start_tag.take() {
            self.send(tag, DeviceMessage::Err(ErrorCode::ERadio)).await;
        }
    }

    async fn reject_not_configured(&mut self) {
        let tag = self.incoming_tag;
        self.send(tag, DeviceMessage::Err(ErrorCode::ENotConfigured))
            .await;
    }

    async fn stop_rx_hw(&mut self) {
        let _ = self.lora.enter_standby().await;
    }

    // ── Reset (disconnect / inactivity timeout) ──────────────────────

    async fn reset_to_unconfigured(&mut self) {
        self.pending_tx = None;
        self.last_tx_tag = None;
        self.pending_rx_start_tag = None;
        self.config = None;
        self.packets_dropped = 0;
        let _ = self.lora.enter_standby().await;
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
            self.last_tx_tag = Some(pt.tag);
        }
        self.announce(RadioEvent::EnteredTx);
    }

    async fn announce_packet_tx(&mut self) {
        self.announce(RadioEvent::PacketTx);
    }

    async fn announce_rx_failed(&mut self) {
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
    lora.prepare_for_rx(RxMode::Continuous, &mdltn, &pkt).await
}

async fn reconfigure_radio(lora: &mut LoRaDriver, cfg: &LoRaConfig) -> Result<(), RadioError> {
    lora.enter_standby().await?;
    // Calibrate image etc. via a throwaway create_modulation_params call
    // — inside the SET_CONFIG OK window so first TX/RX is fast.
    let _ = to_lora_mod_params(lora, cfg)?;
    Ok(())
}

async fn do_tx(lora: &mut LoRaDriver, tx: &PendingTx) -> Result<TxOutcome, RadioError> {
    let mdltn = to_lora_mod_params(lora, &tx.config)?;

    if !tx.skip_cad {
        lora.prepare_for_cad(&mdltn).await?;
        let mut saw_activity = false;
        for _ in 0..CAD_MAX_RETRIES {
            if !lora.cad(&mdltn).await? {
                saw_activity = false;
                break;
            }
            saw_activity = true;
            embassy_time::Timer::after_millis(10).await;
            lora.prepare_for_cad(&mdltn).await?;
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
    lora.prepare_for_tx(&mdltn, &mut tx_pkt, tx.config.tx_power_dbm as i32, &tx.data)
        .await?;
    lora.tx().await?;
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
    let lora = match LoRa::new(parts.driver, false, parts.delay).await {
        Ok(l) => {
            info!("radio initialized");
            l
        }
        Err(e) => {
            error!("radio init failed: {}", e);
            return;
        }
    };

    let ctx = RadioContext {
        commands,
        outbound,
        events,
        info,
        lora,
        rx_buf: [0u8; MAX_OTA_PAYLOAD],
        config: None,
        pending_tx: None,
        packets_dropped: 0,
        incoming_tag: 0,
        last_tx_tag: None,
        pending_rx_start_tag: None,
    };

    static RADIO_INTERNAL_CH: embassy_sync::channel::Channel<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        RadioInput,
        8,
    > = embassy_sync::channel::Channel::new();
    let mut machine = Radio::new(ctx, &RADIO_INTERNAL_CH);
    let _ = machine.run().await;
}
