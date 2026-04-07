//! USB CDC-ACM host task: COBS-framed fixed-size LE command/response protocol.

use defmt::warn;
use embassy_executor::task;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::Builder;
use static_cell::StaticCell;

use crate::channel::{CommandChannel, DisplayCommand, DisplayCommandChannel, ResponseChannel};
use super::framing::{self, CobsDecoder, MAX_FRAME};

const MAX_PACKET: usize = 64;

#[task]
pub async fn host_task(
    parts: crate::board::UsbParts,
    commands: &'static CommandChannel,
    responses: &'static ResponseChannel,
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

    let class = CdcAcmClass::new(&mut builder, cdc_state, MAX_PACKET as u16);
    let mut usb_dev = builder.build();
    let (sender, receiver) = class.split();

    join(
        usb_dev.run(),
        protocol_loop(sender, receiver, commands, responses, display_commands, has_display, mac),
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
    use embassy_futures::select::{select3, Either3};

    let mut read_buf = [0u8; MAX_PACKET];
    let mut write_buf = [0u8; MAX_FRAME];
    let mut cobs_encode_buf = [0u8; MAX_FRAME];
    let mut decoder = CobsDecoder::new();
    let mut was_connected = false;

    loop {
        match select3(
            receiver.read_packet(&mut read_buf),
            responses.receive(),
            embassy_time::Timer::after_millis(250),
        )
        .await
        {
            Either3::First(result) => {
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
                    framing::route_command(
                        cmd, commands, responses, display_commands, has_display, mac,
                    )
                    .await;
                }
            }
            Either3::Second(response) => {
                if let Some(frame) =
                    framing::cobs_encode_response(response, &mut write_buf, &mut cobs_encode_buf)
                {
                    for chunk in frame.chunks(MAX_PACKET) {
                        if sender.write_packet(chunk).await.is_err() {
                            warn!("USB write failed, response dropped");
                            break;
                        }
                    }
                }
            }
            Either3::Third(()) => {}
        }

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
