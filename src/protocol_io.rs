//! Shared COBS protocol helpers for USB and UART transports.
//!
//! Both `usb_task` and `uart_task` use these helpers for frame
//! accumulation, response encoding, and command routing.

use defmt::warn;

use crate::channel::{CommandChannel, DisplayCommand, DisplayCommandChannel, ResponseChannel};
use crate::protocol::{Command, ErrorCode, Response};

/// Maximum COBS frame size (bytes).
pub const MAX_FRAME: usize = 512;

// Worst case: tag(1) + rssi(2) + snr(2) + len(2) + payload(256) = 263 bytes.
// COBS adds ceil(263/254)+1 = 3 bytes. 266 < 512. Plenty of room.
const _: () = assert!(
    MAX_FRAME >= crate::protocol::MAX_PAYLOAD + 64,
    "MAX_FRAME too small for max payload + COBS overhead"
);

/// Decodes COBS-framed commands from a byte stream.
pub struct CobsDecoder {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl CobsDecoder {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME],
            len: 0,
        }
    }

    /// Reset the accumulator (discard partial frame).
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Feed a chunk of bytes, calling `on_command` for each decoded command.
    pub fn feed(&mut self, data: &[u8], mut on_command: impl FnMut(Command)) {
        for &byte in data {
            if byte == 0x00 {
                // End of COBS frame — decode and dispatch
                if self.len > 0 {
                    let mut decode_buf = [0u8; MAX_FRAME];
                    if let Some(decoded_len) =
                        ucobs::decode(&self.buf[..self.len], &mut decode_buf)
                    {
                        if let Some(cmd) = Command::from_bytes(&decode_buf[..decoded_len]) {
                            on_command(cmd);
                        }
                    }
                }
                self.len = 0;
            } else if self.len < MAX_FRAME {
                self.buf[self.len] = byte;
                self.len += 1;
            } else {
                // Frame too large — discard
                self.len = 0;
            }
        }
    }
}

/// COBS-encode a response into `encode_buf` with trailing 0x00 sentinel.
///
/// Returns the slice to send (encoded frame + sentinel), or `None` on overflow.
pub fn cobs_encode_response<'a>(
    response: Response,
    write_buf: &mut [u8; MAX_FRAME],
    encode_buf: &'a mut [u8; MAX_FRAME],
) -> Option<&'a [u8]> {
    let raw_len = response.write_to(write_buf);
    let encoded_len = ucobs::encode(&write_buf[..raw_len], encode_buf).unwrap_or(0);
    if encoded_len < encode_buf.len() {
        encode_buf[encoded_len] = 0x00;
        Some(&encode_buf[..encoded_len + 1])
    } else {
        warn!("COBS encode buffer overflow");
        None
    }
}

/// Route a parsed command to the appropriate handler.
///
/// Display and MAC commands are handled locally (response sent immediately).
/// All other commands are forwarded to the radio task (response sent later).
/// Both paths feed the same `ResponseChannel`, so the one-outstanding-command
/// rule in PROTOCOL.md is required to keep solicited responses ordered.
pub async fn route_command(
    cmd: Command,
    commands: &CommandChannel,
    responses: &ResponseChannel,
    display_commands: &DisplayCommandChannel,
    has_display: bool,
    mac: [u8; 6],
) {
    match cmd {
        Command::DisplayOn => {
            if has_display {
                display_commands.send(DisplayCommand::On).await;
                responses.send(Response::Ok).await;
            } else {
                responses.send(Response::Error(ErrorCode::NoDisplay)).await;
            }
        }
        Command::DisplayOff => {
            if has_display {
                display_commands.send(DisplayCommand::Off).await;
                responses.send(Response::Ok).await;
            } else {
                responses.send(Response::Error(ErrorCode::NoDisplay)).await;
            }
        }
        Command::GetMac => {
            responses.send(Response::MacAddress(mac)).await;
        }
        other => {
            commands.send(other).await;
        }
    }
}
