//! USB CDC-ACM host task: COBS-framed fixed-size LE command/response protocol.
//!
//! ```text
//! Host
//! ├── Disconnected   (default)   no DTR
//! └── Connected                  host attached
//!     entry: wake_display
//!     exit:  stop_radio, reset_display
//! ```
//!
//! The statechart tracks exactly one thing: DTR. Entry/exit actions
//! replace the old `was_connected` edge-detector and emit the display
//! wake / radio-stop / display-reset side-effects that used to be open
//! coded around the `select3` loop.
//!
//! The data plane (COBS decoding, command routing, response encoding)
//! stays imperative in `protocol_loop` for the same reason the radio's
//! `rx_once` stayed out of its chart: `read_packet` blocks and can't
//! live inside a statechart action. Host-contract lifecycle (one
//! outstanding command, unsolicited `RxPacket`, synchronous local
//! responses) is deliberately *not* modeled — any finer chart here
//! would desynchronise from the concurrent `select3` reality and lie
//! about what the firmware is actually doing.

use defmt::warn;
use embassy_executor::task;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::Builder;
use hsmc::{statechart, Duration};
use static_cell::StaticCell;

use super::framing::{self, CobsDecoder, MAX_FRAME};
use crate::channel::{CommandChannel, DisplayCommand, DisplayCommandChannel, ResponseChannel};
use crate::protocol::Command;

const MAX_PACKET: usize = 64;

/// Internal events driving `Host` state transitions.
#[derive(Debug, Clone)]
pub enum HostInput {
    /// DTR rose: host opened the serial port.
    DtrRaised,
    /// DTR fell: host closed or disconnected.
    DtrDropped,
}

/// Machine context — channel handles the actions need to broadcast
/// DTR-edge side-effects. The COBS decoder, read/write scratch, and
/// USB endpoints stay local to `protocol_loop` because their async
/// operations can't live inside statechart actions.
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

    fn stop_radio_best_effort(&self) {
        if self.commands.try_send(Command::StopRx).is_err() {
            warn!("host DTR-drop: StopRx dropped (command channel full)");
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
            exit: stop_radio, reset_display;
            on(DtrDropped) => Disconnected;
        }
    }
}

impl HostActions for HostActionContext<'_> {
    async fn wake_display(&mut self) {
        self.wake_display_if_present().await;
    }

    async fn stop_radio(&mut self) {
        self.stop_radio_best_effort();
    }

    async fn reset_display(&mut self) {
        self.reset_display_if_present().await;
    }
}

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
    use embassy_futures::select::{select3, Either3};

    let mut machine = Host::new_local(HostContext {
        commands,
        display_commands,
        has_display,
    });
    // Enter the default: Disconnected.
    machine.step(Duration::ZERO).await;

    let mut read_buf = [0u8; MAX_PACKET];
    let mut write_buf = [0u8; MAX_FRAME];
    let mut cobs_encode_buf = [0u8; MAX_FRAME];
    let mut decoder = CobsDecoder::new();

    loop {
        match select3(
            receiver.read_packet(&mut read_buf),
            responses.receive(),
            embassy_time::Timer::after_millis(250),
        )
        .await
        {
            Either3::First(Ok(0)) => {}
            Either3::First(Ok(n)) => {
                let mut cmds = heapless::Vec::<_, 4>::new();
                decoder.feed(&read_buf[..n], |cmd| {
                    let _ = cmds.push(cmd);
                });
                for cmd in cmds {
                    framing::route_command(
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
            Either3::First(Err(_)) => {
                // Read errored — usually a USB disconnect. Drop decoder
                // scratch and back off briefly so we don't spin at I/O
                // rate. Do NOT `continue`: the DTR check below must
                // still run so the machine's exit action fires.
                decoder.reset();
                embassy_time::Timer::after_millis(100).await;
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

        // Edge-detect DTR against the machine's current state. The
        // machine's entry/exit actions own the side-effects. dispatch()
        // pushes the event AND drains the internal queue, so
        // current_state() reflects the post-dispatch state before we
        // loop back around.
        let connected = receiver.dtr();
        let was_connected = matches!(machine.current_state(), HostState::Connected);
        match (was_connected, connected) {
            (false, true) => {
                let _ = machine.dispatch(HostInput::DtrRaised).await;
            }
            (true, false) => {
                // Belt-and-braces: reset decoder here too. The error
                // arm above typically fires on disconnect and already
                // resets it, but this makes intent explicit and
                // survives embassy-usb behavior changes where a
                // disconnect doesn't surface as a read error.
                decoder.reset();
                let _ = machine.dispatch(HostInput::DtrDropped).await;
            }
            _ => {}
        }
    }
}
