//! Shared COBS protocol helpers for USB and UART transports.
//!
//! Both `usb_task` and `uart_task` use these helpers for frame
//! accumulation, response encoding, and command routing.
//!
//! The actual codec lives in `donglora-protocol` so host crates can
//! reuse it; this module re-exports the codec and owns the
//! firmware-only [`route_command`] (which touches embassy channels).

pub use donglora_protocol::framing::{cobs_encode_response, CobsDecoder, MAX_FRAME};

use crate::channel::{CommandChannel, DisplayCommand, DisplayCommandChannel, ResponseChannel};
use crate::protocol::{Command, ErrorCode, Response};

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
