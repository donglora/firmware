//! Wire protocol types for DongLoRa Protocol v2.
//!
//! Wire definitions live in the `donglora-protocol` crate so host-side
//! Rust projects can share them with the firmware without duplicating
//! wire layouts. This module re-exports them unchanged so firmware
//! modules can `use crate::protocol::*;`.

#[allow(unused_imports)]
pub use donglora_protocol::{
    cap, commands, encode_frame, events, Command, CommandParseError, DeviceMessage,
    DeviceMessageParseError, ErrorCode, FlrcConfig, FrameDecoder, FrameEncodeError, FrameResult,
    FskConfig, Info, InfoParseError, LoRaBandwidth, LoRaCodingRate, LoRaConfig, LoRaHeaderMode,
    LrFhssConfig, Modulation, ModulationId, OkPayload, Owner, RadioChipId, RxOrigin, RxPayload,
    SetConfigResult, SetConfigResultCode, TxDonePayload, TxFlags, TxResult, MAX_MCU_UID_LEN,
    MAX_OK_PAYLOAD, MAX_OTA_PAYLOAD, MAX_PAYLOAD_FIELD, MAX_RADIO_UID_LEN, MAX_WIRE_FRAME,
};
