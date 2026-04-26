//! USB CDC-ACM host task for DongLoRa Protocol v2.
//!
//! ```text
//! Host
//! ├── Disconnected             (default)   no DTR
//! └── Connected                            host attached
//!     ├── WaitingForFirstFrame (default)   DTR up, no traffic yet
//!     │                                    NO inactivity timer (boot quiet
//!     │                                    is normal — chart structurally
//!     │                                    blocks the spurious 1s fire).
//!     └── Active                           ≥1 frame seen since connect
//!                                          inactivity timer (1s) armed and
//!                                          restarted by every frame via
//!                                          self-transition.
//! ```
//!
//! Two chart-level invariants made structurally impossible by the shape:
//! 1. **No spurious inactivity at boot.** The 1-second timer only exists
//!    in `Active`, which requires a `FrameReceived` event to enter.
//!    A freshly-connected host that holds DTR high but sends nothing
//!    sits in `WaitingForFirstFrame` indefinitely — no timer, no warn,
//!    no Reset cascade.
//! 2. **Chart-owned timer scope.** Per hsmc README rule §7 (timers
//!    armed by state lifecycle), the inactivity timer is cancelled on
//!    every exit of `Active`, restarted on every entry. No global
//!    `last_frame_at` instant, no manual elapsed check in the loop.
//!
//! The data plane (frame decode, command dispatch, frame encode) stays
//! imperative in `protocol_loop` because `read_packet` blocks.

use embassy_executor::task;
use embassy_futures::join::join;
use embassy_time::Duration;
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
/// Declared in the chart on `Active` — chart owns timer lifecycle.
const INACTIVITY_MS: u64 = 1000;

/// Poll interval for the DTR edge detector. Chart owns the inactivity
/// timer; this is only the cadence at which protocol_loop wakes when
/// no other event is pending, to re-check DTR.
const POLL_INTERVAL_MS: u64 = 100;

#[derive(Debug, Clone)]
pub enum HostInput {
    DtrRaised,
    DtrDropped,
    /// A USB read returned ≥1 byte. Drives `WaitingForFirstFrame →
    /// Active` (first frame after connect) and the `Active → Active`
    /// self-transition that restarts the inactivity timer per
    /// hsmc README rule §4 (self-transition exits + re-enters target,
    /// re-arming its timer).
    FrameReceived,
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
        use portable_atomic::Ordering;
        // Dedupe: if a Reset is already queued (and not yet consumed by
        // the radio task), don't queue another. Without this, a slow or
        // wedged radio + repeated chart exits (DTR drop, inactivity,
        // both transports) fill the commands channel and produce the
        // "queue full" warn flood. The radio chart's reset_to_unconfigured
        // clears the flag once it processes the queued Reset.
        if crate::channel::RESET_PENDING.swap(true, Ordering::AcqRel) {
            return;
        }
        if self.commands.try_send(InboundEvent::Reset).is_err() {
            // Channel genuinely full of non-Reset commands; this is the
            // edge case worth knowing about. Clear the flag so future
            // Reset attempts can re-try once the channel drains.
            defmt::warn!("radio Reset inject: command queue full");
            crate::channel::RESET_PENDING.store(false, Ordering::Release);
        }
    }
}

statechart! {
    Host {
        context: HostContext;
        events: HostInput;
        // Auto-trace — backend selected by `hsmc/trace-*` cargo feature.
        trace;
        default(Disconnected);

        state Disconnected {
            on(DtrRaised) => Connected;
        }

        state Connected {
            entry: wake_display;
            exit: reset_radio, reset_display;
            on(DtrDropped) => Disconnected;
            default(WaitingForFirstFrame);

            // Default child of Connected. No inactivity timer yet —
            // a freshly-connected host that hasn't sent its first byte
            // is normal (mux-rs holds DTR high before any client
            // attaches). Stays here indefinitely until the first
            // frame arrives.
            state WaitingForFirstFrame {
                on(FrameReceived) => Active;
            }

            // Inactivity timer is chart-scoped: armed on every entry to
            // Active (whether from WaitingForFirstFrame or self-loop
            // after a frame), cancelled on every exit (per hsmc README
            // rule §7). The self-transition `on(FrameReceived) =>
            // Active` re-enters Active per rule §4, re-arming the
            // timer — equivalent to the "reset last_frame_at on each
            // byte" pattern but expressed in chart shape.
            //
            // hsmc's `after` triggers want `hsmc::Duration` (re-export
            // of `core::time::Duration`), not `embassy_time::Duration`.
            state Active {
                on(after HsmcDuration::from_millis(INACTIVITY_MS)) => Disconnected;
                on(FrameReceived) => Active;
            }
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

    // CTRL_BUF was 64 bytes; bumped to 128 because the previous size
    // matches the panic site "control buffer too small" in
    // embassy_usb/src/lib.rs:758. CdcAcmClass only requires >= 7 for
    // SET_LINE_CODING, but composite descriptor handlers and string
    // descriptor responses may overrun a 64 B buffer for some host
    // enumerations. 128 is safely above the largest CDC-ACM control
    // request and well within RAM budget.
    static DESC_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static CONF_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_BUF: StaticCell<[u8; 256]> = StaticCell::new();
    static CTRL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    static CDC_STATE: StaticCell<State> = StaticCell::new();

    let desc_buf = DESC_BUF.init([0; 256]);
    let conf_buf = CONF_BUF.init([0; 256]);
    let bos_buf = BOS_BUF.init([0; 256]);
    let ctrl_buf = CTRL_BUF.init([0; 128]);
    let cdc_state = CDC_STATE.init(State::new());

    let mut builder = Builder::new(parts.driver, config, desc_buf, conf_buf, bos_buf, ctrl_buf);

    // Note: when this device enumerates on Linux, modem-manager's
    // CDC-ACM probe issues SET_INTERFACE with alt settings beyond what
    // we declare, triggering a "SET_INTERFACE: alt setting out of
    // range" WARN from embassy_usb. The probe is normal Linux
    // behavior; embassy_usb's STALL response is correct USB negotiation.
    // The WARN is benign noise — do NOT add dummy alt settings just to
    // silence it (risks breaking Windows/macOS enumeration).
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

    loop {
        match select3(
            receiver.read_packet(&mut read_buf),
            outbound.receive(),
            embassy_time::Timer::after(Duration::from_millis(POLL_INTERVAL_MS)),
        )
        .await
        {
            Either3::First(Ok(0)) => {
                defmt::info!("host_task: read_packet returned 0 bytes");
            }
            Either3::First(Ok(n)) => {
                defmt::info!(
                    "host_task: read_packet got {=usize}B (DTR={=bool})",
                    n,
                    receiver.dtr(),
                );
                // Drives WaitingForFirstFrame → Active on first byte
                // and Active → Active (timer-restart) on every byte
                // after. The chart owns inactivity from here.
                let _ = machine.dispatch(HostInput::FrameReceived).await;
                dispatch_bytes(&mut decoder, &read_buf[..n], commands, outbound).await;
            }
            Either3::First(Err(e)) => {
                // Per embassy_usb_driver::EndpointError, `read_packet`
                // returns one of two errors:
                //   - `Disabled`:       OUT endpoint not enabled by the host
                //                       (USB enumeration in progress, host
                //                       suspended, or interface not yet
                //                       configured). Completely normal during
                //                       bring-up + every host disconnect.
                //   - `BufferOverflow`: incoming packet larger than read_buf.
                //                       read_buf is sized to MAX_USB_PACKET
                //                       (64) which matches the CDC-ACM EP
                //                       max packet size, so this should
                //                       never happen in practice.
                //
                // `Disabled` floods at boot before USB enumeration finishes
                // (the OS hasn't sent SET_CONFIGURATION yet) — was previously
                // logging dozens of "read_packet error" lines per second.
                // Demote to debug-level since it's expected; reserve WARN
                // for BufferOverflow which IS a real bug.
                use embassy_usb_driver::EndpointError;
                match e {
                    EndpointError::Disabled => {
                        defmt::debug!(
                            "host_task: read_packet returned Disabled (USB endpoint not yet enabled by host — normal during enumeration)"
                        );
                    }
                    EndpointError::BufferOverflow => {
                        defmt::warn!(
                            "host_task: read_packet BufferOverflow — host sent a packet larger than {=usize}B (max EP packet size); decoder reset",
                            MAX_USB_PACKET,
                        );
                    }
                }
                decoder.reset();
                embassy_time::Timer::after_millis(100).await;
            }
            Either3::Second(frame) => {
                let frame_tag = frame.tag;
                if let Some(n) = encode_outbound(&frame, &mut write_buf) {
                    defmt::info!(
                        "host_task: sending {=usize}B frame tag={=u16}",
                        n,
                        frame_tag,
                    );
                    // Per embassy_usb 0.6: write_packet only returns
                    // EndpointError::Disabled when the IN endpoint is
                    // dead (host gone or USB suspended). If a chunk
                    // mid-frame fails, the partial bytes already sent
                    // are unrecoverable — host's framing layer will see
                    // corruption until a clean frame arrives. Bail on
                    // the rest of THIS frame and set RADIO_THROTTLED so
                    // the radio task stops generating async events
                    // until the host comes back. Without the throttle,
                    // every subsequent frame would also fail mid-write
                    // and produce more wire corruption.
                    let mut frame_failed = false;
                    for (chunk_idx, chunk) in
                        write_buf[..n].chunks(MAX_USB_PACKET).enumerate()
                    {
                        defmt::info!(
                            "host_task: write_packet chunk {=usize} ({=usize}B) tag={=u16}",
                            chunk_idx,
                            chunk.len(),
                            frame_tag,
                        );
                        if sender.write_packet(chunk).await.is_err() {
                            defmt::warn!(
                                "USB write failed (host stalled or gone); frame dropped tag={=u16}, throttling radio",
                                frame_tag,
                            );
                            crate::channel::RADIO_THROTTLED
                                .store(true, portable_atomic::Ordering::Release);
                            frame_failed = true;
                            break;
                        }
                        defmt::info!(
                            "host_task: write_packet OK chunk {=usize} tag={=u16}",
                            chunk_idx,
                            frame_tag,
                        );
                    }
                    if !frame_failed
                        && crate::channel::RADIO_THROTTLED
                            .load(portable_atomic::Ordering::Acquire)
                    {
                        defmt::info!(
                            "USB write recovered (tag={=u16}); clearing radio throttle",
                            frame_tag,
                        );
                        crate::channel::RADIO_THROTTLED
                            .store(false, portable_atomic::Ordering::Release);
                    }
                    defmt::info!("host_task: frame done tag={=u16}", frame_tag);
                }
            }
            Either3::Third(()) => {
                // Heartbeat tick — drives DTR edge polling + inactivity
                // watchdog below. No log: it would fire every
                // POLL_INTERVAL_MS and bury the interesting traffic.
            }
        }

        // DTR edge detection — drives Connected / Disconnected
        // transitions. The chart's `current_state()` may be
        // Disconnected, Connected/WaitingForFirstFrame, or
        // Connected/Active per the new hierarchy. We're connected if
        // current state is anywhere inside Connected.
        let dtr_high = receiver.dtr();
        let was_connected = !matches!(
            machine.current_state(),
            HostState::Disconnected
        );
        match (was_connected, dtr_high) {
            (false, true) => {
                defmt::info!("host_task: DTR raised");
                let _ = machine.dispatch(HostInput::DtrRaised).await;
            }
            (true, false) => {
                defmt::info!("host_task: DTR dropped");
                decoder.reset();
                let _ = machine.dispatch(HostInput::DtrDropped).await;
            }
            _ => {}
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
