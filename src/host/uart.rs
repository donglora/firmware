//! UART host task for DongLoRa Protocol v2.
//!
//! UART has no DTR line, so we assume a host is always present and
//! rely entirely on the spec §3.4 inactivity watchdog (1 s of silence
//! → Disconnected → drop radio to Unconfigured).

use defmt::warn;
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

    let mut read_buf = [0u8; READ_BUF];
    let mut write_buf: [u8; MAX_WIRE_FRAME] = [0u8; MAX_WIRE_FRAME];
    let mut decoder = FrameDecoder::new();
    let mut last_frame_at = Instant::now();
    let mut connected = true;

    loop {
        use embassy_futures::select::{select3, Either3};

        match select3(
            embedded_io_async::Read::read(&mut rx, &mut read_buf),
            outbound.receive(),
            embassy_time::Timer::after(Duration::from_millis(100)),
        )
        .await
        {
            Either3::First(result) => match result {
                Ok(0) => continue,
                Ok(n) => {
                    last_frame_at = Instant::now();
                    if !connected {
                        if has_display {
                            let _ = display_commands.try_send(DisplayCommand::On);
                        }
                        connected = true;
                    }
                    dispatch_bytes(&mut decoder, &read_buf[..n], commands, outbound).await;
                }
                Err(_) => {
                    decoder.reset();
                    embassy_time::Timer::after_millis(100).await;
                }
            },
            Either3::Second(frame) => {
                if let Some(n) = encode_outbound(&frame, &mut write_buf) {
                    if tx.write_all(&write_buf[..n]).await.is_err() {
                        warn!("UART write failed, frame dropped");
                    }
                }
            }
            Either3::Third(()) => {}
        }

        // Inactivity watchdog.
        if connected && last_frame_at.elapsed() >= Duration::from_millis(INACTIVITY_MS) {
            warn!("UART host inactivity timeout fired");
            let _ = commands.try_send(InboundEvent::Reset);
            if has_display {
                let _ = display_commands.try_send(DisplayCommand::Reset);
            }
            connected = false;
        }
    }
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
