//! UART host task for DongLoRa Protocol v2.
//!
//! UART has no DTR line, so we assume a host is always present and
//! rely entirely on the spec §3.4 inactivity watchdog (1 s of silence
//! → Disconnected → drop radio to Unconfigured).

use defmt::{info, warn};
use embassy_executor::task;
use embassy_time::{Duration, Instant};
use embedded_io_async::Write;

use super::framing::{dispatch_frame, emit_async_err, encode_outbound};
use crate::channel::{
    CommandChannel, DisplayCommand, DisplayCommandChannel, InboundEvent, OutboundChannel,
};
use crate::protocol::{ErrorCode, FrameDecoder, FrameResult, MAX_PAYLOAD_FIELD, MAX_WIRE_FRAME};

const READ_BUF: usize = 64;
const INACTIVITY_MS: u64 = 1000;

#[task]
pub async fn host_task(
    parts: crate::board::UartParts,
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
    _mac: [u8; 6],
) {
    let (mut rx, mut tx) = parts.driver.split();

    // UART has no DTR — wake display immediately and assume a host.
    if has_display {
        display_commands.send(DisplayCommand::On).await;
    }

    // Two sub-loops joined in the same task. Splitting read and write
    // is what stops a long-running tx.write_all (~25 ms for a max-size
    // frame at 115_200 baud, multiplied across a saturated outbound
    // queue) from starving the inbound poll. While write_all sits in
    // its FIFO-drain await, join re-polls the read future on the next
    // wake, so last_frame_at keeps refreshing and the §3.4 watchdog
    // does not spuriously fire under sustained bidirectional load.
    // Same task = single executor + disjoint borrows on the split
    // rx/tx halves, so no Mutex is needed.
    let read_loop = async {
        let mut read_buf = [0u8; READ_BUF];
        let mut decoder = FrameDecoder::new();
        let mut last_frame_at = Instant::now();
        let mut connected = true;

        loop {
            use embassy_futures::select::{select, Either};

            match select(
                embedded_io_async::Read::read(&mut rx, &mut read_buf),
                embassy_time::Timer::after(Duration::from_millis(100)),
            )
            .await
            {
                Either::First(Ok(0)) => {}
                Either::First(Ok(n)) => {
                    last_frame_at = Instant::now();
                    if !connected {
                        if has_display {
                            let _ = display_commands.try_send(DisplayCommand::On);
                        }
                        connected = true;
                    }
                    dispatch_bytes(&mut decoder, &read_buf[..n], commands, outbound).await;
                }
                Either::First(Err(_)) => {
                    // Byte-level error (FIFO overrun, UART framing,
                    // parity). Per PROTOCOL.md §3.4: only "byte stream
                    // activity that results in a complete frame
                    // arriving" resets the inactivity timer, so we do
                    // NOT touch last_frame_at here. The 100 ms sleep
                    // is anti-spin: a stuck driver pumping continuous
                    // Errs would otherwise burn the executor.
                    decoder.reset();
                    embassy_time::Timer::after_millis(100).await;
                }
                Either::Second(()) => {}
            }

            // Inactivity watchdog (PROTOCOL.md §3.4, 1000 ms).
            if connected && last_frame_at.elapsed() >= Duration::from_millis(INACTIVITY_MS) {
                warn!("UART host inactivity timeout fired");
                let _ = commands.try_send(InboundEvent::Reset);
                if has_display {
                    let _ = display_commands.try_send(DisplayCommand::Reset);
                }
                // Drop in-flight frame state. When the host resumes it
                // may pick up mid-frame; without resetting the decoder
                // we'd chew through garbage until the next 0x00
                // sentinel. USB gets the equivalent for free via
                // DTR-drop reset (host/usb.rs Disconnected entry); UART
                // has no DTR.
                decoder.reset();
                connected = false;
            }
        }
    };

    let write_loop = async {
        let mut write_buf: [u8; MAX_WIRE_FRAME] = [0u8; MAX_WIRE_FRAME];

        loop {
            let frame = outbound.receive().await;
            let Some(n) = encode_outbound(&frame, &mut write_buf) else {
                continue;
            };
            // Mirror src/host/usb.rs:349-374. The throttle gate at
            // src/radio.rs:2017 silently drops async RX into the
            // outbound queue while RADIO_THROTTLED is true; clear it
            // on every successful write and re-arm it on failure,
            // same contract as USB.
            match tx.write_all(&write_buf[..n]).await {
                Ok(()) => {
                    if crate::channel::RADIO_THROTTLED
                        .load(portable_atomic::Ordering::Acquire)
                    {
                        info!("UART write recovered; clearing radio throttle");
                        crate::channel::RADIO_THROTTLED
                            .store(false, portable_atomic::Ordering::Release);
                    }
                }
                Err(_) => {
                    warn!("UART write failed, frame dropped, throttling radio");
                    crate::channel::RADIO_THROTTLED
                        .store(true, portable_atomic::Ordering::Release);
                }
            }
        }
    };

    embassy_futures::join::join(read_loop, write_loop).await;
}

async fn dispatch_bytes(
    decoder: &mut FrameDecoder,
    bytes: &[u8],
    commands: &'static CommandChannel,
    outbound: &'static OutboundChannel,
) {
    let mut work: heapless::Vec<FrameWork, 4> = heapless::Vec::new();
    decoder.feed(bytes, |res| match res {
        FrameResult::Ok {
            type_id,
            tag,
            payload,
        } => {
            let mut p: heapless::Vec<u8, MAX_PAYLOAD_FIELD> = heapless::Vec::new();
            let _ = p.extend_from_slice(payload);
            let _ = work.push(FrameWork::Ok {
                type_id,
                tag,
                payload: p,
            });
        }
        FrameResult::Err(_) => {
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
        payload: heapless::Vec<u8, MAX_PAYLOAD_FIELD>,
    },
    Frame,
}
