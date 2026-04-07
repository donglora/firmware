//! UART task: COBS-framed fixed-size LE command/response protocol.
//!
//! Reads COBS frames from UART, decodes them, parses commands, routes them
//! to the radio or display task, and sends COBS-framed responses back.
//! Used for boards with USB-UART bridge chips (e.g. CP2102 on some Heltec V3).

use defmt::warn;
use embassy_executor::task;
use embedded_io_async::Write;

use crate::channel::{CommandChannel, DisplayCommandChannel, ResponseChannel};
use crate::protocol_io::{self, CobsDecoder, MAX_FRAME};

#[task]
pub async fn uart_task(
    parts: crate::board::UartParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
    mac: [u8; 6],
) {
    let (mut rx, mut tx) = parts.driver.split();

    let mut read_buf = [0u8; 64];
    let mut write_buf = [0u8; MAX_FRAME];
    let mut cobs_encode_buf = [0u8; MAX_FRAME];
    let mut decoder = CobsDecoder::new();

    loop {
        use embassy_futures::select::select;
        use embassy_futures::select::Either;

        match select(
            embedded_io_async::Read::read(&mut rx, &mut read_buf),
            responses.receive(),
        )
        .await
        {
            Either::First(result) => {
                let n = match result {
                    Ok(0) => continue,
                    Ok(n) => n,
                    Err(_) => {
                        decoder.reset();
                        embassy_time::Timer::after_millis(100).await;
                        continue;
                    }
                };

                let mut cmds = heapless::Vec::<_, 4>::new();
                decoder.feed(&read_buf[..n], |cmd| {
                    let _ = cmds.push(cmd);
                });
                for cmd in cmds {
                    protocol_io::route_command(
                        cmd, commands, responses, display_commands, has_display, mac,
                    )
                    .await;
                }
            }
            Either::Second(response) => {
                if let Some(frame) =
                    protocol_io::cobs_encode_response(response, &mut write_buf, &mut cobs_encode_buf)
                {
                    if tx.write_all(frame).await.is_err() {
                        warn!("UART write failed, response dropped");
                    }
                }
            }
        }
    }
}
