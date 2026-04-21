//! Shared framing helpers for USB and UART host transports.
//!
//! Ingress path: raw bytes → `FrameDecoder` → `(type_id, tag, payload)`
//! → `Command::parse` → forwarded to radio task along with the tag. On
//! parse failure we echo `ERR(ELENGTH/EPARAM/EMODULATION/EUNKNOWN_CMD)`
//! with the received tag. On CRC/COBS failure we emit async
//! `ERR(EFRAME)` with tag `0x0000` (`PROTOCOL.md §2.5`).
//!
//! Egress path: `OutboundFrame` (tag + DeviceMessage) → `encode_frame`
//! → bytes ready for transport-level write.

use defmt::warn;

use crate::channel::{CommandChannel, InboundEvent, OutboundChannel, OutboundFrame};
use crate::protocol::{
    encode_frame, Command, CommandParseError, DeviceMessage, ErrorCode, FrameEncodeError,
    MAX_PAYLOAD_FIELD, MAX_WIRE_FRAME,
};

/// Dispatch a frame-level decode into the appropriate downstream queue.
///
/// - `type_id`, `tag`, and `payload` come from `FrameDecoder`.
/// - Known commands are forwarded to the radio task via `commands`.
/// - Parse failures produce a synchronous `ERR` with the echoed tag on
///   `outbound`, so the host sees a concrete answer instead of timing out.
/// - The caller is responsible for rejecting `tag == 0` before calling
///   this; the spec says hosts MUST NOT send tag 0, and we classify such
///   frames as framing errors upstream.
pub async fn dispatch_frame(
    type_id: u8,
    tag: u16,
    payload: &[u8],
    commands: &CommandChannel,
    outbound: &OutboundChannel,
) {
    if tag == 0 {
        // Hosts MUST NOT use tag 0 on outbound commands. Treat as
        // framing error; do not route to radio. (Spec §2.2)
        emit_async_err(outbound, ErrorCode::EFrame).await;
        return;
    }
    match Command::parse(type_id, payload) {
        Ok(cmd) => {
            let envelope = InboundEvent::Command { tag, cmd };
            if commands.try_send(envelope).is_err() {
                emit_sync_err(outbound, tag, ErrorCode::EBusy).await;
            }
        }
        Err(e) => {
            let code = parse_err_to_wire_code(e);
            emit_sync_err(outbound, tag, code).await;
        }
    }
}

/// Emit an async framing error (tag = 0x0000) per spec §2.5.
pub async fn emit_async_err(outbound: &OutboundChannel, code: ErrorCode) {
    let frame = OutboundFrame {
        tag: 0,
        msg: DeviceMessage::Err(code),
    };
    if outbound.try_send(frame).is_err() {
        warn!("async ERR dropped: outbound queue full");
    }
}

/// Emit a synchronous error response echoing the originating tag.
async fn emit_sync_err(outbound: &OutboundChannel, tag: u16, code: ErrorCode) {
    let frame = OutboundFrame {
        tag,
        msg: DeviceMessage::Err(code),
    };
    if outbound.try_send(frame).is_err() {
        warn!("sync ERR dropped: outbound queue full");
    }
}

fn parse_err_to_wire_code(e: CommandParseError) -> ErrorCode {
    match e {
        CommandParseError::UnknownType => ErrorCode::EUnknownCmd,
        CommandParseError::WrongLength => ErrorCode::ELength,
        CommandParseError::InvalidField => ErrorCode::EParam,
        CommandParseError::ReservedBitSet => ErrorCode::EParam,
        CommandParseError::UnknownModulation => ErrorCode::EModulation,
    }
}

/// Encode an outbound frame into a single wire-ready byte buffer.
///
/// Returns the number of bytes written to `out`, or `None` on buffer
/// overflow / encode failure (caller may log and drop).
pub fn encode_outbound(frame: &OutboundFrame, out: &mut [u8; MAX_WIRE_FRAME]) -> Option<usize> {
    let mut payload_buf = [0u8; MAX_PAYLOAD_FIELD];
    let payload_len = match frame.msg.encode_payload(&mut payload_buf) {
        Ok(n) => n,
        Err(e) => {
            warn!("encode_payload failed: {:?}", defmt::Debug2Format(&e));
            return None;
        }
    };
    match encode_frame(
        frame.msg.type_id(),
        frame.tag,
        &payload_buf[..payload_len],
        out.as_mut_slice(),
    ) {
        Ok(n) => Some(n),
        Err(FrameEncodeError::BufferTooSmall) => {
            warn!("encode_frame: output buffer too small");
            None
        }
        Err(FrameEncodeError::PayloadTooLarge) => {
            warn!("encode_frame: payload exceeds MAX_PAYLOAD_FIELD");
            None
        }
        Err(FrameEncodeError::CobsEncode) => {
            warn!("encode_frame: COBS encode refused");
            None
        }
    }
}
