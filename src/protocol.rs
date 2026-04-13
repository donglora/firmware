//! Wire protocol types.
//!
//! The definitions live in the `donglora-protocol` crate so host-side
//! Rust projects (client-rs, mux-rs, bridge) can share them with the
//! firmware without duplicating wire layouts. This module re-exports
//! them unchanged for existing `crate::protocol::*` call sites.

#[allow(unused_imports)]
pub use donglora_protocol::{
    Bandwidth, Command, ErrorCode, RadioConfig, Response, MAX_PAYLOAD, PREAMBLE_DEFAULT,
    RADIO_CONFIG_SIZE, TX_POWER_MAX,
};
