//! USB CDC-ACM task: COBS-framed fixed-size LE command/response protocol.
//!
//! Reads COBS frames from USB, decodes them, parses commands, routes them
//! to the radio or display task, and sends COBS-framed responses back.

use defmt::warn;
use embassy_executor::task;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::Builder;
use static_cell::StaticCell;

use crate::channel::{CommandChannel, DisplayCommand, DisplayCommandChannel, ResponseChannel};
use crate::protocol::{Command, ErrorCode, Response};

const MAX_FRAME: usize = 512;
const MAX_PACKET: usize = 64;

// Worst case: tag(1) + rssi(2) + snr(2) + len(2) + payload(256) = 263 bytes.
// COBS adds ceil(263/254)+1 = 3 bytes. 266 < 512. Plenty of room.
const _: () = assert!(
    MAX_FRAME >= crate::protocol::MAX_PAYLOAD + 64,
    "MAX_FRAME too small for max payload + COBS overhead"
);

#[task]
pub async fn usb_task(
    parts: crate::board::UsbParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
    mac: [u8; 6],
) {
    // ── USB device configuration ────────────────────────────────────
    // VID 0x1209: pid.codes open-source USB vendor ID
    // PID 0x5741: DongLoRa product ID
    // Class 0xEF/0x02/0x01: Miscellaneous / Interface Association Descriptor
    let mut config = embassy_usb::Config::new(0x1209, 0x5741);
    config.manufacturer = Some("DongLoRa");
    config.product = Some("DongLoRa LoRa Radio");
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Use MAC address as USB serial number for unique device identification.
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

    let class = CdcAcmClass::new(&mut builder, cdc_state, MAX_PACKET as u16);
    let mut usb_dev = builder.build();
    let (sender, receiver) = class.split();

    // ── Run USB device + protocol loop concurrently ────────────────
    join(
        usb_dev.run(),
        protocol_loop(
            sender,
            receiver,
            commands,
            responses,
            display_commands,
            has_display,
            mac,
        ),
    )
    .await;
}

async fn protocol_loop<'d, D: embassy_usb_driver::Driver<'d>>(
    mut sender: Sender<'d, D>,
    mut receiver: Receiver<'d, D>,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
    display_commands: &'static DisplayCommandChannel,
    has_display: bool,
    mac: [u8; 6],
) {
    use embassy_futures::select::select3;
    use embassy_futures::select::Either3;

    let mut read_buf = [0u8; MAX_PACKET];
    let mut write_buf = [0u8; MAX_FRAME];
    let mut cobs_encode_buf = [0u8; MAX_FRAME];

    // COBS accumulator: collect bytes until 0x00 sentinel
    let mut frame_buf = [0u8; MAX_FRAME];
    let mut frame_len: usize = 0;

    let mut was_connected = false;

    loop {
        match select3(
            receiver.read_packet(&mut read_buf),
            responses.receive(),
            embassy_time::Timer::after_millis(250), // DTR poll interval for disconnect detection
        )
        .await
        {
            Either3::First(result) => {
                let n = match result {
                    Ok(0) => continue,
                    Ok(n) => n,
                    Err(_) => {
                        frame_len = 0; // reset accumulator
                        embassy_time::Timer::after_millis(100).await;
                        continue;
                    }
                };

                // Feed bytes into the COBS accumulator
                for &byte in &read_buf[..n] {
                    if byte == 0x00 {
                        // End of COBS frame — decode and parse
                        if frame_len > 0 {
                            let mut decode_buf = [0u8; MAX_FRAME];
                            if let Some(decoded_len) =
                                ucobs::decode(&frame_buf[..frame_len], &mut decode_buf)
                            {
                                if let Some(cmd) = Command::from_bytes(&decode_buf[..decoded_len]) {
                                    route_command(
                                        cmd,
                                        commands,
                                        responses,
                                        display_commands,
                                        has_display,
                                        mac,
                                    )
                                    .await;
                                }
                            }
                        }
                        frame_len = 0;
                    } else if frame_len < MAX_FRAME {
                        frame_buf[frame_len] = byte;
                        frame_len += 1;
                    } else {
                        // Frame too large — discard
                        frame_len = 0;
                    }
                }
            }
            Either3::Second(response) => {
                // Serialize response to fixed-size LE bytes, then COBS encode
                let raw_len = response.write_to(&mut write_buf);
                let encoded_len =
                    ucobs::encode(&write_buf[..raw_len], &mut cobs_encode_buf).unwrap_or(0);
                // Append 0x00 sentinel
                if encoded_len < cobs_encode_buf.len() {
                    cobs_encode_buf[encoded_len] = 0x00;
                    let frame = &cobs_encode_buf[..encoded_len + 1];
                    for chunk in frame.chunks(MAX_PACKET) {
                        if sender.write_packet(chunk).await.is_err() {
                            warn!("USB write failed, response dropped");
                            break;
                        }
                    }
                } else {
                    warn!("COBS encode buffer overflow");
                }
            }
            Either3::Third(()) => {
                // Poll DTR every 250ms to detect host disconnect
            }
        }

        // Check DTR after every select wake
        let connected = receiver.dtr();
        if was_connected && !connected && has_display {
            display_commands.send(DisplayCommand::Reset).await;
        }
        if !was_connected && connected && has_display {
            display_commands.send(DisplayCommand::On).await;
        }
        was_connected = connected;
    }
}

/// Route a parsed command to the appropriate handler.
///
/// Display and MAC commands are handled locally (response sent immediately).
/// All other commands are forwarded to the radio task (response sent later).
/// Both paths feed the same `ResponseChannel`, so the one-outstanding-command
/// rule in PROTOCOL.md is required to keep solicited responses ordered.
async fn route_command(
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
