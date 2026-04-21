//! USB CDC-ACM host task for DongLoRa Protocol v2.
//!
//! ```text
//! Host
//! ├── Disconnected   (default)   no DTR, or inactivity timer fired
//! └── Connected                  host attached, liveness OK
//!     entry: wake_display, start_liveness_timer
//!     exit:  reset_radio, reset_display, stop_liveness_timer
//! ```
//!
//! The statechart tracks one state bit: "is a host present?". DTR drives
//! connect edges; the 1-second inactivity watchdog (spec §3.4) drives
//! the disconnect edge when DTR is held high but no bytes arrive.
//!
//! The data plane (frame decode, command dispatch, frame encode) stays
//! imperative in `protocol_loop` for the same reason the radio's
//! `rx_once` stayed out of its chart: `read_packet` blocks.

use defmt::warn;
use embassy_executor::task;
use embassy_futures::join::join;
use embassy_time::{Duration, Instant};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::Builder;
use hsmc::{statechart, Duration as HsmcDuration};
use static_cell::StaticCell;

use super::framing::{dispatch_frame, emit_async_err, encode_outbound};
use crate::channel::{
    CommandChannel, DisplayCommand, DisplayCommandChannel, InboundEvent, OutboundChannel,
};
use crate::protocol::{ErrorCode, FrameDecoder, FrameResult, MAX_WIRE_FRAME};

const MAX_USB_PACKET: usize = 64;

/// Spec §3.4: inactivity timer period. The device must see at least one
/// frame (good or bad) every 1000 ms or the session is considered lost.
const INACTIVITY_MS: u64 = 1000;

/// Poll interval for the DTR edge detector / inactivity check.
const POLL_INTERVAL_MS: u64 = 100;

#[derive(Debug, Clone)]
pub enum HostInput {
    DtrRaised,
    DtrDropped,
    /// Host went silent for >1 s; kick the radio back to Unconfigured.
    InactivityTimeout,
}

pub struct HostContext {
    commands: &'static CommandChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
}

impl HostContext {
    async fn wake_display_if_present(&self) {
        if self.has_display {
            self.display_commands.send(DisplayCommand::On).await;
        }
    }

    async fn reset_display_if_present(&self) {
        if self.has_display {
            self.display_commands.send(DisplayCommand::Reset).await;
        }
    }

    fn send_radio_reset(&self) {
        if self.commands.try_send(InboundEvent::Reset).is_err() {
            warn!("radio Reset inject: command queue full");
        }
    }
}

statechart! {
    Host {
        context: HostContext;
        events: HostInput;
        default(Disconnected);

        state Disconnected {
            on(DtrRaised) => Connected;
        }

        state Connected {
            entry: wake_display;
            exit: reset_radio, reset_display;
            on(DtrDropped) => Disconnected;
            on(InactivityTimeout) => Disconnected;
        }
    }
}

impl HostActions for HostActionContext<'_> {
    async fn wake_display(&mut self) {
        self.wake_display_if_present().await;
    }

    async fn reset_radio(&mut self) {
        self.send_radio_reset();
    }

    async fn reset_display(&mut self) {
        self.reset_display_if_present().await;
    }
}

#[task]
pub async fn host_task(
    parts: crate::board::UsbParts,
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
    mac: [u8; 6],
) {
    let mut config = embassy_usb::Config::new(0x1209, 0x5741);
    config.manufacturer = Some("DongLoRa");
    config.product = Some("DongLoRa LoRa Radio");
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    static SERIAL_BUF: StaticCell<[u8; 12]> = StaticCell::new();
    let serial_buf = SERIAL_BUF.init([0u8; 12]);
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    for (i, &byte) in mac.iter().enumerate() {
        serial_buf[i * 2] = HEX[(byte >> 4) as usize];
        serial_buf[i * 2 + 1] = HEX[(byte & 0x0F) as usize];
    }
    config.serial_number = Some(core::str::from_utf8(serial_buf).expect("MAC hex is valid UTF-8"));

    static DESC_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static CONF_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static CTRL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let desc_buf = DESC_BUF.init([0; 256]);
    let conf_buf = CONF_BUF.init([0; 256]);
    let bos_buf = BOS_BUF.init([0; 256]);
    let ctrl_buf = CTRL_BUF.init([0; 64]);
    let cdc_state = CDC_STATE.init(State::new());

    let mut builder = Builder::new(parts.driver, config, desc_buf, conf_buf, bos_buf, ctrl_buf);

    let class = CdcAcmClass::new(&mut builder, cdc_state, MAX_USB_PACKET as u16);
    let mut usb_dev = builder.build();
    let (sender, receiver) = class.split();

    join(
        usb_dev.run(),
        protocol_loop(
            sender,
            receiver,
            commands,
            outbound,
            display_commands,
            has_display,
        ),
    )
    .await;
}

async fn protocol_loop<'d, D: embassy_usb_driver::Driver<'d>>(
    mut sender: Sender<'d, D>,
    mut receiver: Receiver<'d, D>,
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
) {
    use embassy_futures::select::{select3, Either3};

    let mut machine = Host::new_local(HostContext {
        commands,
        display_commands,
        has_display,
    });
    machine.step(HsmcDuration::ZERO).await;

    let mut read_buf = [0u8; MAX_USB_PACKET];
    let mut write_buf: [u8; MAX_WIRE_FRAME] = [0u8; MAX_WIRE_FRAME];
    let mut decoder = FrameDecoder::new();

    let mut last_frame_at = Instant::now();

    loop {
        match select3(
            receiver.read_packet(&mut read_buf),
            outbound.receive(),
            embassy_time::Timer::after(Duration::from_millis(POLL_INTERVAL_MS)),
        )
        .await
        {
            Either3::First(Ok(0)) => {}
            Either3::First(Ok(n)) => {
                last_frame_at = Instant::now();
                dispatch_bytes(&mut decoder, &read_buf[..n], commands, outbound).await;
            }
            Either3::First(Err(_)) => {
                decoder.reset();
                embassy_time::Timer::after_millis(100).await;
            }
            Either3::Second(frame) => {
                if let Some(n) = encode_outbound(&frame, &mut write_buf) {
                    for chunk in write_buf[..n].chunks(MAX_USB_PACKET) {
                        if sender.write_packet(chunk).await.is_err() {
                            warn!("USB write failed, frame dropped");
                            break;
                        }
                    }
                }
            }
            Either3::Third(()) => {}
        }

        // DTR edge detection — drives Connected / Disconnected transitions.
        let connected = receiver.dtr();
        let was_connected = matches!(machine.current_state(), HostState::Connected);
        match (was_connected, connected) {
            (false, true) => {
                let _ = machine.dispatch(HostInput::DtrRaised).await;
                last_frame_at = Instant::now();
            }
            (true, false) => {
                decoder.reset();
                let _ = machine.dispatch(HostInput::DtrDropped).await;
            }
            _ => {}
        }

        // Inactivity watchdog (spec §3.4): while Connected, if no frame
        // arrived in the last INACTIVITY_MS, drive the session back to
        // Disconnected.
        if matches!(machine.current_state(), HostState::Connected)
            && last_frame_at.elapsed() >= Duration::from_millis(INACTIVITY_MS)
        {
            warn!("host inactivity timeout fired");
            let _ = machine.dispatch(HostInput::InactivityTimeout).await;
        }
    }
}

/// Feed raw bytes into the frame decoder. For each complete frame:
/// - Ok → route into the command channel (or emit sync ERR on parse fail)
/// - Err(Crc | Cobs | TooShort) → emit async ERR(EFRAME)
///
/// Borrow-scoped per the `FrameDecoder::feed` callback contract: the
/// payload slice is valid only for the duration of the callback.
async fn dispatch_bytes(
    decoder: &mut FrameDecoder,
    bytes: &[u8],
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
) {
    // `FrameDecoder::feed` hands payloads by reference; we must synchronously
    // copy any data we need before the callback returns. To avoid fighting
    // lifetimes we accumulate per-frame work as an enum.
    let mut work: heapless::Vec<FrameWork, 4> = heapless::Vec::new();
    decoder.feed(bytes, |res| match res {
        FrameResult::Ok {
            type_id,
            tag,
            payload,
        } => {
            let mut p: heapless::Vec<u8, { crate::protocol::MAX_PAYLOAD_FIELD }> =
                heapless::Vec::new();
            let _ = p.extend_from_slice(payload);
            let _ = work.push(FrameWork::Ok {
                type_id,
                tag,
                payload: p,
            });
        }
        FrameResult::Err(_e) => {
            let _ = work.push(FrameWork::Frame);
        }
    });
    for w in work {
        match w {
            FrameWork::Ok {
                type_id,
                tag,
                payload,
            } => {
                dispatch_frame(type_id, tag, &payload, commands, outbound).await;
            }
            FrameWork::Frame => {
                emit_async_err(outbound, ErrorCode::EFrame).await;
            }
        }
    }
}

#[allow(clippy::large_enum_variant)]
enum FrameWork {
    Ok {
        type_id: u8,
        tag: u16,
        payload: heapless::Vec<u8, { crate::protocol::MAX_PAYLOAD_FIELD }>,
    },
    Frame,
}
