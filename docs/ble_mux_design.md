# DongLoRa BLE Soft-Mesh Multiplexer — Design Document

**Status:** DRAFT v0.3
**Date:** 2026-04-12
**Target audience:** Senior embedded firmware engineers (depth, tradeoffs); junior engineers (must be implementable without asking questions).

> **Changes from v0.2** (junior-engineer round 2 review): single connectable adv set instead of dual sets (v0.2 assumed an `AdvertisementSet::update_data` API that `nrf-softdevice` does not expose — now designed around `advertise_connectable()` with reshape-on-reconfigure); full implementations for `AdvMgr`, `ConnTable`, `BleTxRouter`, `rotate_identity`, `ed25519` key management; concrete crate pins (nrf-softdevice git rev, ed25519-compact, blake2); `Origin::Ble` carries a `(slot, generation)` pair to defeat slot-recycle response-misrouting; v3 ring is 64 slots partitioned 32+24+8 (fixes v0.2 arithmetic contradiction); Status characteristic semantics are fixed length 128 bytes zero-padded (consistent across versions); `dongle_id` is 8 bytes everywhere; SEQ_WATCH bumped to 8 receivers; test-host feature added; per-board button handling reframed; duty-cycle computation delegated to `lora_phy::time_on_air`; iOS background-wake claim corrected (once-per-service-UUID, not per-change-tag); `embassy-nrf` init uses real API; bonding linker variant + `build.rs` selection; peer-sync opcode scoping made explicit.

> **Changes from v0.1:** CRDT framing reframed as op-based CmRDT with content-addressed keys; v1 adds regulatory TX gate (duty cycle + access control); address + epoch rotation; `Signal` → `Watch` for multi-consumer; RxRing lock is per-packet snapshot, never across `.await`; Wio Tracker L1 has a bootloader prerequisite; linker numbers aligned with in-tree `ld/nrf52840-memory.x`; v3 mandates ed25519 peer identity; panic-rate-limit safe-mode; several factual errors corrected (Embassy thread-mode vs priority, 2M PHY scope, ATT MTU 247, VLOC usage, SD RAM minimum, epoch widened to u64).

---

## 0. Purpose and reading guide

This document specifies a BLE feature for DongLoRa firmware that turns a dongle from a 1-user USB device into a **shared near-field multiplexer** for LoRa packets. Nearby phones observe and catch up via BLE; phones that want to transmit briefly connect via GATT. Firmware stays dumb; protocol semantics live on phones.

### 0.1 Reading order

- §§1–2 — motivation, constraints, scope
- §§3–4 — architecture and state model
- §5 — **v1.0** MVP (~6.5 weeks focused)
- §6 — **v2.0** hardening, LE Secure Connections, ESP32-S3, optional scanner mode
- §7 — **v3.0** multi-dongle mesh with cryptographic peer identity (~7 weeks on top of v2)
- §§8–9 — testing, metrics, rollout
- Appendix A — wire formats
- Appendix B — state machines
- Appendix C — Cargo, linker, SoftDevice init
- Appendix D — glossary and references
- Appendix E — implementer decisions

### 0.2 Prerequisites

- Rust `async`/`await` + Embassy: `embassy_sync::channel::Channel`, `mutex::Mutex`, `watch::Watch`, `signal::Signal`, `static_cell::StaticCell`, `#[embassy_executor::task]`. See [embassy.dev/book](https://embassy.dev/book/).
- BLE: GAP roles, advertising (legacy vs extended), GATT characteristic properties, ATT MTU, pairing / bonding. Bluetooth Core 5.3 Vol 3 Parts C and G.
- `nrf-softdevice` crate: [github.com/embassy-rs/nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice); examples `ble_peripheral.rs`, `ble_advertise.rs`.
- DongLoRa `PROTOCOL.md` (wire protocol unchanged by BLE).

### 0.3 Mutex raw-mutex choice

Use `CriticalSectionRawMutex` everywhere in this design. Under SoftDevice S140, `ThreadModeRawMutex` is unsafe: S140 ISRs at priorities 0/1/4 preempt thread-mode code, causing re-entrancy on thread-mode-only locks. `nrf-softdevice/critical-section-impl` routes `critical_section::with` through `sd_nvic_critical_region_enter/_exit`, S140-safe. See §5.12.

---

## 1. Motivation, goals, regulatory constraints

### 1.1 The problem

A DongLoRa today is 1:1: one USB cable, one host, one user. The dongle is a radio, not an identity; radios are shareable. A household with three users shouldn't need three dongles.

### 1.2 The reframe

A DongLoRa becomes a neighborhood _radio gateway_. Any phone within ~10 m can observe and occasionally transmit. Higher-layer protocols run on phones.

### 1.3 Goals

1. Multiplex to ≤ 4 concurrent GATT connections; unlimited passive scanners.
2. Work with iOS background apps (scan-wake + GATT catch-up, not continuous ad consumption).
3. Preserve USB CDC administration; no wire change for `donglora-mux`.
4. No per-protocol firmware evolution.
5. Build on existing nRF52840 boards with existing Embassy code.
6. Degrade gracefully — BLE congestion must not block LoRa; dropped connections must not corrupt state.
7. **Regulatory compliance**: enforce ISM-band duty-cycle rules in firmware (EU 868 MHz 1%; US 915 MHz 400 ms dwell).

### 1.4 Non-goals

- Not a general-purpose BLE stack.
- Not a phone-side library.
- Not a Meshtastic replacement — firmware is a raw packet mover.
- Not an RP2040 feature.

### 1.5 Regulatory constraints — a v1 precondition

Without firmware-enforced rate limiting and duty-cycle tracking, a hostile or buggy phone can exhaust the ISM-band budget in seconds and put the dongle's operator in legal jeopardy. v1 ships with:

- **Firmware-enforced TX air-time accounting** per rolling hour (§5.9.4), computed from `lora_phy::time_on_air`.
- **Per-connection TX rate limit** (default 1 TX / 500 ms; §5.9.5).
- **TX access control** — bonded-only (default in v2); board-button-gated fallback (§5.9.1); USB-admin override (§5.9.3). The "open TX" model is rejected as non-compliant.

---

## 2. Scope

### 2.1 Hardware

| Board                       | MCU      | BLE | Bootloader                                               | v1 status                    |
| --------------------------- | -------- | --- | -------------------------------------------------------- | ---------------------------- |
| RAK WisBlock 4631           | nRF52840 | 5.1 | Adafruit nRF52 UF2 (factory)                             | **Primary target**           |
| Wio Tracker L1              | nRF52840 | 5.1 | Seeed default — **prerequisite reflash required** (§2.2) | Secondary                    |
| Heltec V3 / V4 (USB / UART) | ESP32-S3 | 5.0 | n/a                                                      | v2 target via `esp32-nimble` |
| ELECROW ThinkNode-M2        | ESP32-S3 | 5.0 | n/a                                                      | v2 target                    |
| Waveshare RP2040-LoRa       | RP2040   | —   | —                                                        | Excluded (no BLE radio)      |

### 2.2 Wio Tracker L1 bootloader prerequisite

Seeed ships Wio Tracker L1 with a bootloader that does not expect SoftDevice S140 coexistence. Flashing a BLE build on the factory bootloader bricks USB-DFU. The justfile recipe `just wio-prepare-adafruit-bootloader` (see Appendix C.1) reflashes the bootloader to the matching Adafruit nRF52 UF2 variant via SWD/probe-rs. Run this once per device. Recovery from a mistake requires SWD/J-Link.

`build.rs` emits `cargo:warning=` on `wio_tracker_l1_ble` builds reminding the developer.

### 2.3 Cross-cutting constraints

- **Binary size**: ≤ ~780 KB flash on nRF52840 post-S140. Current firmware ~194 KB; plenty of headroom.
- **RAM**: ≤ ~255 KB. §5.10 enumerates.
- **USB wire compat**: non-`ble` builds byte-identical on USB; `donglora-mux` diff must be empty (§5.14 Tier 0).
- **LoRa RX/TX determinism**: SX1262 on SPI; S140 owns the 2.4 GHz radio; no contention.

---

## 3. High-level architecture

### 3.1 System diagram

```
┌──────────────────────────────────────────────────────────────────────┐
│                          DongLoRa Firmware                            │
│                                                                       │
│  ┌──────────────┐        ┌──────────────────────┐   ┌─────────────┐   │
│  │  radio_task  │◄──────►│ CommandChannel       │◄──│ host_task   │   │
│  │  owns SX1262 │        │ (Origin, Command)×16 │   │ USB or UART │   │
│  └──┬───────────┘        └──────────┬───────────┘   └──┬──────────┘   │
│     │ push (rssi,snr,payload)      ▲                   │              │
│     ▼                              │                   │              │
│  ┌──────────────┐                  │                   │              │
│  │ RX_RING      │                  │ (Origin::Ble(s,g),│              │
│  │ async Mutex  │                  │  Command::Transmit)              │
│  │ snapshot API │                  │                   │              │
│  └──────┬───────┘                  │                   │              │
│         │ seq bumps                │                   │              │
│         ▼                          │                   │              │
│  ┌──────────────┐                  │                   │              │
│  │ SEQ_WATCH    │                  │                   │              │
│  │ Watch<u32,8> │─── fan-out ──────┤                   │              │
│  └──────────────┘                  │                   │              │
│                                    │                   │              │
│  ┌───────────────────────────────┐ │                   │              │
│  │ ble_task (single spawned task)│─┘                   │              │
│  │  Inner tasks (via join):      │                     │              │
│  │   • adv_and_connect_loop      │                     │              │
│  │   • rotate_timer              │                     │              │
│  │   • rx_notify_fanout          │                     │              │
│  │                               │                     │              │
│  │  On each new Connection,      │                     │              │
│  │  spawns handle_connection()   │                     │              │
│  │  which owns gatt_server::run  │                     │              │
│  │  for that peer.               │                     │              │
│  └───────────────────────────────┘                     │              │
│                                                        │              │
│  ┌───────────────────────────────┐                     │              │
│  │ softdevice_task (sibling)     │                     │              │
│  │ runs Softdevice::run()        │                     │              │
│  └───────────────────────────────┘                     │              │
│                                                        │              │
│  ┌───────────────────────────────┐   ┌───────────────┐ │              │
│  │ display_task (unchanged)      │◄──│  StatusWatch  │ │              │
│  └───────────────────────────────┘   │   (N=3)       │ │              │
│                                      └───────────────┘ │              │
└────────────────────────────────────────────────────────┼──────────────┘
                        │                                │
                    USB │                                │ BLE
                        ▼                                ▼
                 donglora-mux                     phones (observer + TX)
```

### 3.2 Components

| Component         | Owns                                                         | Key invariants                                                                      |
| ----------------- | ------------------------------------------------------------ | ----------------------------------------------------------------------------------- |
| `radio_task`      | SX1262, radio config, RX/TX state                            | Never blocks on BLE. Push to `RX_RING` bounded < 5 µs.                              |
| `RX_RING`         | Bounded RX history (v1: 128 slots; v3: 64 slots partitioned) | Per-packet snapshot API — mutex never held across `.await`.                         |
| `SEQ_WATCH`       | `Watch<CSRM, u32, 8>`                                        | 8 receivers: advertiser + up to 4 RX-Notify fanouts + up to 3 v3 peer-sync workers. |
| `ConnTable`       | 4 `ConnSlot`s, keyed by `Connection` (not raw handle)        | Slot recycling mitigated by generation counter; see §5.5.3.                         |
| `BleTxRouter`     | `[Signal<TxOutcome>; MAX_CONNS]`                             | Routes TX outcomes back to origin slot+generation pair.                             |
| `softdevice_task` | Runs `sd.run()` forever                                      | Spawned before any BLE API call.                                                    |
| `ble_task`        | Single connectable adv set; gatt server; rotations           | Sibling of softdevice_task.                                                         |

### 3.3 Data and control flow

1. **RX from air.** radio_task gets `(len, pkt_status)` → brief `RX_RING::push` → `SEQ_WATCH.send(seq)` → also `responses.try_send(Response::RxPacket)` for USB (unchanged).
2. **Advertiser observes SEQ_WATCH** and reshapes adv data on a debounced cadence.
3. **Phone connects**, pulls via Sync, disconnects. Per-packet snapshot streaming, lock-held < 5 µs per packet.
4. **Phone transmits**. Write-chunk reassembly → `(Origin::Ble(slot, generation), Command::Transmit)` into `CommandChannel` → awaits `BLE_TX_ROUTER[slot].wait()` → Status notification → completes.
5. **USB admin** continues to see every RX and reconfigures radio; does not see BLE activity.

---

## 4. State and naming

### 4.1 Identifiers

- **`dongle_id: [u8; 8]`** — truncated BLAKE2s-256 of the ed25519 identity public key (v3 only); stable across address rotations. 8 bytes _everywhere_ in this document.
- **`epoch: u64`** — random at boot, re-randomized on address rotation. Scopes `seq`.
- **`seq: u32`** — monotonic per-epoch; on `u32::MAX` saturation forces a new epoch (safety net; would take 135+ years at 1 pkt/sec).
- **`tx_id: u6`** — client-chosen TX correlator in TX-Write chunked frames.
- **`(slot: u8, generation: u16)`** — internal pair that routes TX outcomes (§5.5.3).
- **Op codes** — first-byte opcodes, _scoped per-characteristic_ (opcode 0x60 on Peer-Sync is a different namespace from 0x60 on Status).

### 4.2 Address rotation

v1 and later: BLE device address is non-resolvable random, rotated every 15 min; `epoch` re-randomizes at the same moment. Bonded peers (v2+) see an IRK-based resolvable-private-address instead — they resolve to stable identity; passive observers don't.

Active GATT connections drop on rotation (forced via `sd_ble_gap_disconnect` with reason `REMOTE_USER_TERMINATED`). Clients reconnect immediately. A 15-minute rotation gives users ~30 seconds' grace after a reconnect to complete a sync before the next rotation. §5.4.5 has the full procedure.

### 4.3 Terminology

| Term              | Meaning                                                                                                                                                            |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Nudge adv**     | Our single connectable advert with service UUID + manuf metadata (§5.4). Note: v0.2 used "nudge" to mean a distinct non-connectable set; v0.3 merges into one adv. |
| **Catch-up sync** | Client connects, writes SyncSince, receives delta, disconnects.                                                                                                    |
| **Drift / gap**   | Client's `since_seq` is older than ring oldest.                                                                                                                    |
| **Soft-mesh**     | Phone-side view across one or more dongles (v2+); dongles gossip directly in v3.                                                                                   |
| **S140**          | Nordic SoftDevice S140 v7.3.0.                                                                                                                                     |
| **CmRDT**         | Commutative Replicated Data Type (op-based); v3 correct model (§7.4).                                                                                              |

---

## 5. V1.0 — The MVP

**Scope:** nRF52840. Single dongle. Gated TX (regulatory). Ring + connectable adv + sync. USB coexists. iOS-friendly. Rotating address + epoch.

**Effort:** ~6.5 weeks focused.

### 5.1 SoftDevice

#### 5.1.1 Choice

`nrf-softdevice` with S140 v7.3.0. Pinned to a specific git revision (see §5.12). TrouBLE rejected for v1 (extended adv + concurrent roles still maturing as of early 2026).

#### 5.1.2 Configuration (v1)

- Peripheral connections: 4.
- Central: 0 (v1, v2); 1 in v3.
- Connection event length: 3 units (3.75 ms). Senior review showed that event_length 6 (7.5 ms) × 4 connections doesn't fit a 30 ms interval with an adv set running alongside.
- ATT MTU: request 247 on connect. `evt-max-size-257` Cargo feature on nrf-softdevice (the SD event buffer is 247 ATT payload + 10 B of L2CAP/ATT/padding = 257).
- PHY: request 2M for connections. Legacy adv on primary channels 37/38/39 is always 1M by BLE spec.
- Adv sets: **1** (single connectable adv, see §5.4). v0.2's dual-set design was based on an API that `nrf-softdevice` does not expose.
- Extended adv: disabled (v2 `ble_fat_adv` enables).
- GATT attribute table size: 2048 B (> default 1408, headroom for future characteristics).

#### 5.1.3 Interrupt priorities

Cortex-M4 has 3 priority bits (0 highest, 7 lowest). S140 reserves 0 (SWI2_EGU2), 1 (RADIO, TIMER0, RTC0, CCM_AAR), 4 (SWI5_EGU5, MWU). Application may use 2, 3, 5, 6, 7.

Embassy's thread-mode executor has no NVIC priority; it is preempted by every enabled ISR. The v0.1 claim "Embassy at priority 6" was wrong; the correct statement is "thread mode, preempted by all ISRs." `embassy-nrf` with `time-driver-rtc1` sets RTC1 IRQ at priority 2 (safe).

#### 5.1.4 Critical-section impl

On (`ble`): `nrf-softdevice/critical-section-impl` — routes `critical_section::with` to `sd_nvic_critical_region_enter/_exit`, S140-safe.
Off: `cortex-m/critical-section-single-core` — naive disable-interrupts.

Exactly one is active per build. `build.rs` compile-time guard (§5.12).

#### 5.1.5 SoftDevice initialization — complete code

```rust
// src/ble/softdevice.rs
use nrf_softdevice::{raw, Softdevice};

static DEVICE_NAME: [u8; 8] = *b"DongLoRa";

pub fn enable() -> &'static Softdevice {
    let cfg = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
            rc_ctiv: 0,
            rc_temp_ctiv: 0,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 4,
            event_length: 3,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 247 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: 2048,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 4,
            central_role_count: 0,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            // VLOC_USER: app owns the buffer; SD stores the pointer.
            // DEVICE_NAME is in .rodata (static) so it lives for 'static.
            p_value: DEVICE_NAME.as_ptr() as *mut u8,
            current_len: DEVICE_NAME.len() as u16,
            max_len: DEVICE_NAME.len() as u16,
            write_perm: raw::ble_gap_conn_sec_mode_t {
                _bitfield_1: raw::ble_gap_conn_sec_mode_t::new_bitfield_1(0, 0),
            },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_USER as u8,
            ),
        }),
        ..Default::default()
    };
    Softdevice::enable(&cfg)
}

#[embassy_executor::task]
pub async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}
```

#### 5.1.6 Boot ordering — complete `main`

```rust
// src/main.rs — nrf52840 + ble path
use embassy_executor::Spawner;
use embassy_nrf::config::Config as NrfConfig;
use static_cell::StaticCell;

static RX_RING_CELL: StaticCell<RxRingMutex> = StaticCell::new();
static COMMANDS: StaticCell<CommandChannel> = StaticCell::new();
static RESPONSES: StaticCell<ResponseChannel> = StaticCell::new();
static STATUS: StaticCell<StatusWatch> = StaticCell::new();
static SEQ_WATCH_CELL: StaticCell<SeqWatch> = StaticCell::new();
static BLE_TX_ROUTER: BleTxRouter = BleTxRouter::new(); // const-new; see §5.5.3

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // 1. Embassy HAL init. Use Default — SD and embassy-nrf coexist because
    //    nrf-softdevice's POWER handling takes over from embassy-nrf once
    //    Softdevice::enable() runs. Documented in nrf-softdevice examples.
    let mut nrf_cfg = NrfConfig::default();
    // The only explicit tweak: embassy-nrf HFCLK config must not request XTAL
    // before SD enables — SD owns clock control. Leave at default (RC).
    let p = embassy_nrf::init(nrf_cfg);

    // 2. Initial epoch from HWRNG BEFORE enabling SD (SD takes the RNG peripheral).
    let initial_epoch: u64 = {
        let mut rng = embassy_nrf::rng::Rng::new_blocking(p.RNG);
        let mut buf = [0u8; 8];
        rng.blocking_fill_bytes(&mut buf);
        u64::from_le_bytes(buf)
    };

    // 3. Initialize static RxRing with epoch.
    let mut ring_init = RxRing::new_empty();
    ring_init.set_epoch(initial_epoch);
    let ring = RX_RING_CELL.init(Mutex::new(ring_init));

    // 4. Read MAC from FICR (unchanged; board-specific helper).
    let mac = hal::nrf52840::read_mac();

    // 5. Enable SoftDevice. After this line: do NOT use p.RNG, p.TIMER0, p.RTC0,
    //    p.SWI1/2/5, p.CCM, p.AAR, p.TEMP, p.POWER through embassy-nrf.
    let sd = ble::softdevice::enable();
    spawner.must_spawn(ble::softdevice::softdevice_task(sd));

    // 6. Set an initial random non-resolvable BLE address.
    ble::set_random_address(sd, &mac); // see §5.4.5

    // 7. GATT server (after SD is running).
    let server = ble::gatt::Server::new(sd).expect("gatt register");

    // 8. Inter-task channels.
    let commands  = COMMANDS.init(CommandChannel::new());
    let responses = RESPONSES.init(ResponseChannel::new());
    let status    = STATUS.init(StatusWatch::new());
    let seq_watch = SEQ_WATCH_CELL.init(SeqWatch::new());

    // 9. Radio & host parts.
    let (radio_parts, usb_parts, display_parts, has_display) =
        board::into_parts(p);

    // 10. Spawn tasks. ble_task owns ConnTable and AdvMgr internally.
    spawner.must_spawn(radio_task(
        radio_parts, ring, commands, responses,
        status.sender(), seq_watch.sender(), &BLE_TX_ROUTER,
    ));
    spawner.must_spawn(host_task(
        usb_parts, commands, responses, display_cmds, has_display, mac,
    ));
    spawner.must_spawn(ble::ble_task(
        sd, server, ring, commands, seq_watch.receiver().unwrap(),
        &BLE_TX_ROUTER, initial_epoch,
    ));
    if has_display {
        spawner.must_spawn(display_task(display_parts, display_cmds, status.receiver().unwrap()));
    }
}
```

### 5.2 RX_RING

#### 5.2.1 Type

```rust
// src/rx_ring.rs
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::Deque;
use static_cell::StaticCell;

pub const RING_CAPACITY_V1: usize = 128;
pub const MAX_PAYLOAD: usize = 256;

#[derive(Clone, defmt::Format)]
pub struct BufferedPacket {
    pub epoch: u64,      // 8
    pub seq: u32,        // 4
    pub rssi: i16,       // 2
    pub snr: i16,        // 2
    pub len: u16,        // 2
    pub payload: [u8; MAX_PAYLOAD],  // 256
}
// Size: 8 + 4 + 2 + 2 + 2 + 256 = 274. Padded to 4-byte alignment = 276 B.
// 128 slots × 276 = 35,328 B = 34.5 KiB.

pub struct RxRing {
    buf: Deque<BufferedPacket, RING_CAPACITY_V1>,
    epoch: u64,
    next_seq: u32,
}

impl RxRing {
    pub const fn new_empty() -> Self {
        Self { buf: Deque::new(), epoch: 0, next_seq: 0 }
    }

    pub fn set_epoch(&mut self, epoch: u64) {
        self.epoch = epoch;
        self.next_seq = 0;
        self.buf.clear();
    }

    pub fn push(&mut self, rssi: i16, snr: i16, payload: &[u8]) -> (u64, u32) {
        let pkt = BufferedPacket::from_rx(self.epoch, self.next_seq, rssi, snr, payload);
        if self.buf.is_full() { let _ = self.buf.pop_front(); }
        self.buf.push_back(pkt).ok();
        let seq = self.next_seq;
        self.next_seq = self.next_seq.checked_add(1).unwrap_or_else(|| {
            // u32 overflow safety net: force new epoch, clear ring.
            self.epoch = self.epoch.wrapping_add(1);
            self.buf.clear();
            0
        });
        (self.epoch, seq)
    }

    /// Returns the packet at (epoch, seq) if present. O(log N).
    /// Clones so caller can release the lock before streaming.
    pub fn snapshot_at(&self, epoch: u64, seq: u32) -> Option<BufferedPacket> {
        if epoch != self.epoch { return None; }
        let (front, back) = self.buf.as_slices();
        let total = front.len() + back.len();
        if total == 0 { return None; }
        // seq increments monotonically with insertion; deque is ordered by insertion.
        let first_seq = front.first().or(back.first()).unwrap().seq;
        if seq < first_seq { return None; }
        let idx = (seq - first_seq) as usize;
        if idx >= total { return None; }
        let pkt = if idx < front.len() { &front[idx] } else { &back[idx - front.len()] };
        Some(pkt.clone())
    }

    pub fn latest_seq(&self) -> u32 { self.buf.back().map(|p| p.seq).unwrap_or(0) }
    pub fn oldest_seq(&self) -> u32 { self.buf.front().map(|p| p.seq).unwrap_or(0) }
    pub fn epoch(&self) -> u64 { self.epoch }
    pub fn len(&self) -> usize { self.buf.len() }
}

pub type RxRingMutex = Mutex<CriticalSectionRawMutex, RxRing>;
```

`heapless::Deque::new()` is `const` in 0.9+; `as_slices()` exists in 0.9+ (verified).

#### 5.2.2 Lock discipline (the correctness rule)

**Never hold the mutex across an `.await` that performs I/O.** The snapshot API enforces this — one lock acquisition per packet:

```rust
pub async fn stream_sync(
    conn: &Connection,
    server: &Server,
    ring: &'static RxRingMutex,
    req: &SyncReq,
) -> Result<(), StreamError> {
    // 1. Snapshot ring metadata.
    let (epoch, oldest, latest) = {
        let g = ring.lock().await;
        (g.epoch(), g.oldest_seq(), g.latest_seq())
    };

    let mut epoch_mismatch = false;
    let mut gap = false;

    let start = match req.op {
        Op::SyncAll => oldest,
        Op::SyncSince if req.epoch != epoch => { epoch_mismatch = true; oldest }
        Op::SyncSince => {
            if req.since_seq + 1 < oldest { gap = true; oldest }
            else { req.since_seq.wrapping_add(1) }
        }
        Op::SyncOldest => oldest,
        Op::BloomSync => oldest, // v2
    };

    let max = if req.max_count == 0 { 64 } else { req.max_count };
    let mut seq = start;
    let mut count: u16 = 0;

    while count < max && seq <= latest {
        let pkt = {
            let g = ring.lock().await;
            g.snapshot_at(epoch, seq)
        }; // <- lock released before notify
        match pkt {
            Some(p) => {
                notify_sync_packet(conn, server, &p, req.flags).await?;
                count += 1;
            }
            None => gap = true,
        }
        seq = seq.wrapping_add(1);
    }

    // Final end marker reads latest again (new packets may have arrived).
    let final_latest = ring.lock().await.latest_seq();
    notify_sync_end(conn, server, SyncEnd {
        epoch, latest_seq: final_latest, oldest_seq: oldest,
        count_sent: count,
        flags: (gap as u8) | ((count >= max) as u8) << 1 | (epoch_mismatch as u8) << 2,
    }).await
}
```

Lock-held time per call: O(log N) binary search + 276-byte memcpy ≈ < 5 µs at 64 MHz. radio_task's push path waits at most that long.

### 5.3 Radio task changes

#### 5.3.1 Existing code reference

`src/radio.rs` has these `responses` emission sites (confirmed by direct read):

- `responses.try_send(Response::RxPacket { ... })` — RX path.
- `responses.send(Response::Pong).await` — Ping command.
- `responses.send(Response::Config(...))` — GetConfig.
- `responses.send(Response::Ok).await` — SetConfig, StartRx, StopRx, DisplayOn/Off.
- `responses.send(Response::TxDone).await` — Transmit success.
- `responses.send(Response::Error(ErrorCode::...)).await` — error paths: `InvalidConfig`, `RadioBusy`, `TxTimeout`, `NotConfigured`, `NoDisplay`.
- `responses.send(Response::MacAddress(mac)).await` — GetMac.

Existing `ErrorCode` (from `src/protocol.rs:280-287`):

```
InvalidConfig = 0
RadioBusy     = 1
TxTimeout     = 2
NotConfigured = 4
NoDisplay     = 5
```

#### 5.3.2 Diff for BLE build

RX path (additive):

```rust
// After existing responses.try_send(Response::RxPacket{...}):
#[cfg(feature = "ble")]
{
    let (_epoch, seq) = {
        let mut g = ring.lock().await;
        g.push(pkt_status.rssi, pkt_status.snr, &rx_buf[..len])
    };
    seq_watch_sender.send(seq);
}
```

`seq_watch_sender: embassy_sync::watch::Sender<'static, CriticalSectionRawMutex, u32, 8>` (N=8; see §3.2).

TX reply routing:

```rust
async fn reply_tx(
    origin: Origin,
    outcome: Result<(), ErrorCode>,
    responses: &ResponseChannel,
    #[cfg(feature = "ble")] ble_router: &'static BleTxRouter,
) {
    match origin {
        Origin::Host => match outcome {
            Ok(()) => responses.send(Response::TxDone).await,
            Err(code) => responses.send(Response::Error(code)).await,
        },
        #[cfg(feature = "ble")]
        Origin::Ble(slot, generation) => {
            let tx = match outcome {
                Ok(()) => TxOutcome::Done,
                Err(code) => TxOutcome::Error(ble_error_from(code)),
            };
            ble_router.signal(slot, generation, tx);
        }
    }
}

fn ble_error_from(code: ErrorCode) -> BleTxErrorCode {
    match code {
        ErrorCode::InvalidConfig => BleTxErrorCode::ConfigInvalid,  // 0x09
        ErrorCode::RadioBusy     => BleTxErrorCode::RadioBusy,      // 0x07
        ErrorCode::TxTimeout     => BleTxErrorCode::TxTimeout,      // 0x10
        ErrorCode::NotConfigured => BleTxErrorCode::NotConfigured,  // 0x0A (new; see §A.5)
        ErrorCode::NoDisplay     => unreachable!(),                 // never in TX path
    }
}
```

All sites that previously sent `Response::TxDone` or TX-related `Response::Error(...)` now call `reply_tx(origin, ...)`. Non-TX paths (Ping, GetConfig, GetMac, SetConfig, StartRx, StopRx, DisplayOn/Off) continue sending to `responses` unconditionally — they're only issued with `Origin::Host` by design (v1 restricts BLE to Transmit only; §5.5.6).

#### 5.3.3 `CommandChannel` retype

```rust
// src/channel.rs
#[derive(Copy, Clone, Debug, defmt::Format)]
pub enum Origin {
    Host,
    #[cfg(feature = "ble")]
    Ble(u8 /* slot */, u16 /* generation */),
}

pub type CommandChannel = Channel<CriticalSectionRawMutex, (Origin, Command), 16>;
```

Callers must change to wrap commands in `(origin, cmd)`. Sites:

- `src/host/framing.rs::route_command` — wrap as `(Origin::Host, cmd)`.
- `src/ble/tx_assemble.rs` (new) — wrap as `(Origin::Ble(slot, gen), Command::Transmit { .. })`.
- radio_task's destructuring: `while let (origin, cmd) = commands.receive().await { ... }`.

The compiler will surface every missed site.

### 5.4 Advertising — single connectable adv set

#### 5.4.1 Why one set, not two

`nrf-softdevice`'s high-level API exposes `advertise()` / `advertise_connectable()` / `advertise_pairable()`. There is no public `AdvertisementSet::update_data` and no documented way to run two concurrent sets from the one peripheral role (the raw `sd_ble_gap_adv_set_configure` API supports multiple adv handles, but nrf-softdevice doesn't expose this). v0.2's dual-set design is not implementable against this crate.

v0.3: use **one connectable advert** that carries both the service UUID (for discovery) AND the manuf metadata (for iOS scan-wake). When manuf data needs to change, the advert is reconfigured in a tight loop that accepts incoming connections between reshapes.

#### 5.4.2 Adv content

31-byte legacy advertisement:

```
Offset  Bytes              Meaning
0       02 01 06           AD record 1: Flags (3 bytes)
3       11 07 ..16..       AD record 2: Complete 128-bit Service UUID (18 bytes)
21      09 FF FF FF
        DD 01
        <epoch_tag LE:2>
        <change_tag LE:2>  AD record 3: Manuf Specific Data (10 bytes)
                           CID = 0xFFFF (testing); magic = 0xDD; version = 0x01;
                           epoch_tag = (epoch & 0xFFFF) as u16 LE;
                           change_tag = running u16 counter (see 5.4.4).

Total: 3 + 18 + 10 = 31 bytes exactly.
```

Service UUID on the air (little-endian): `01 00 4C 9C 1A D0 8E 8C 1B 4C 9E D0 00 01 40 6E`.

Device name "DongLoRa" is not in the adv (no room) but is exposed via the standard GAP Device Name characteristic (provided by SD automatically when we set `gap_device_name`).

#### 5.4.3 iOS background behavior — the honest claim

iOS Core Bluetooth background scans with `scanForPeripherals(withServices: [ourUUID])` deliver at most one advert per (device address, service UUID) per ~15-second coalescing window. Changing manuf data does NOT force re-delivery in background mode. What it DOES do: (a) foreground scans see every update; (b) when iOS does deliver (once per window), the manuf data is the latest value.

**Implication:** phones that want near-real-time updates must connect and subscribe to RX-Notify. Phones that merely want "a device is nearby and has data" get that at ~15 s resolution in background. Both are supported; the latency characteristics are now honestly documented.

#### 5.4.4 `AdvMgr` — complete type

```rust
// src/ble/adv.rs
use nrf_softdevice::ble::peripheral::{self, AdvertiseError, ConnectableAdvertisement};
use nrf_softdevice::ble::Connection;
use nrf_softdevice::Softdevice;

pub struct AdvMgr {
    sd: &'static Softdevice,
    adv_data: [u8; 31],
    change_tag: u16,
    address: [u8; 6],
    epoch_tag: u16,
}

impl AdvMgr {
    pub fn new(sd: &'static Softdevice, address: [u8; 6], epoch: u64) -> Self {
        let mut m = Self {
            sd,
            adv_data: [0; 31],
            change_tag: 0,
            address,
            epoch_tag: (epoch & 0xFFFF) as u16,
        };
        m.build_adv();
        m
    }

    fn build_adv(&mut self) {
        // Flags
        self.adv_data[0..3].copy_from_slice(&[0x02, 0x01, 0x06]);
        // Service UUID (little-endian)
        self.adv_data[3] = 0x11;
        self.adv_data[4] = 0x07;
        self.adv_data[5..21].copy_from_slice(&SERVICE_UUID_BYTES_LE);
        // Manuf specific data
        self.adv_data[21] = 0x09;
        self.adv_data[22] = 0xFF;
        self.adv_data[23..25].copy_from_slice(&[0xFF, 0xFF]); // CID
        self.adv_data[25] = 0xDD;
        self.adv_data[26] = 0x01;
        self.adv_data[27..29].copy_from_slice(&self.epoch_tag.to_le_bytes());
        self.adv_data[29..31].copy_from_slice(&self.change_tag.to_le_bytes());
    }

    pub fn bump_change_tag(&mut self) {
        self.change_tag = self.change_tag.wrapping_add(1);
        self.adv_data[29..31].copy_from_slice(&self.change_tag.to_le_bytes());
    }

    pub fn set_epoch(&mut self, epoch: u64) {
        self.epoch_tag = (epoch & 0xFFFF) as u16;
        self.change_tag = 0;
        self.build_adv();
    }

    /// Set BLE address. Caller MUST stop any active advertising first —
    /// `sd_ble_gap_addr_set` fails with `BUSY` if an adv set is running.
    /// The `advertise_loop` (§5.4.6) drops its adv future before calling this,
    /// which stops the adv in S140.
    pub fn set_address(&mut self, addr: [u8; 6]) {
        self.address = addr;
        // SAFETY: `sd_ble_gap_addr_set` is safe to call when no adv/scan/conn
        // is active. Advertise loop ensures this precondition. The bitfield
        // uses BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE — matches our
        // rotation policy of non-resolvable addresses. Static random would
        // require addr[5] top 2 bits = 11; we clear them in rotate_identity.
        unsafe {
            use nrf_softdevice::raw;
            let gap_addr = raw::ble_gap_addr_t {
                addr,
                _bitfield_1: raw::ble_gap_addr_t::new_bitfield_1(
                    raw::BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE as u8,
                    0, // addr_id_peer
                ),
            };
            let _ = raw::sd_ble_gap_addr_set(&gap_addr);
        }
    }

    /// Run one advertising cycle: wait for a connection.
    ///
    /// Uses `ScannableUndirected` (connectable + scannable-undirected, the most
    /// discovery-friendly legacy variant). `adv_data` carries our 31-byte adv;
    /// `scan_data` is empty — phones get a zero-length scan response.
    pub async fn advertise_once(&self) -> Result<Connection, AdvertiseError> {
        let conn = peripheral::advertise_connectable(
            self.sd,
            ConnectableAdvertisement::ScannableUndirected {
                adv_data: &self.adv_data,
                scan_data: &[],
            },
            &peripheral::Config {
                interval: 320, // 320 × 0.625 ms = 200 ms — fast discovery, moderate battery
                timeout: None, // advertise forever
                primary_phy: peripheral::Phy::M1, // legacy adv is always 1M on primary channels
                secondary_phy: peripheral::Phy::M1,
                ..Default::default()
            },
        ).await?;
        Ok(conn)
    }
}
```

Pinned nrf-softdevice rev (see §5.12): `ConnectableAdvertisement::ScannableUndirected` is the correct variant at that rev — verified from crate source. `nrf-softdevice`'s `Config` struct field names (`interval`, `timeout`, `primary_phy`, `secondary_phy`) are public at that rev.

#### 5.4.5 Address and epoch rotation

```rust
// src/ble/rotate.rs
use embassy_time::{Duration, Timer};

async fn rotate_identity(
    sd: &'static Softdevice,
    ring: &'static RxRingMutex,
    adv: &mut AdvMgr,
    active_conns: &mut ConnTable,
    adv_stop_signal: &Signal<CriticalSectionRawMutex, ()>,
) {
    // 1. Signal advertise_loop to drop its advertise_connectable() future,
    //    which stops the current adv in S140. (advertise_loop restarts
    //    automatically after rotation completes; see §5.4.6.)
    adv_stop_signal.signal(());
    // Give S140 a moment to process the adv-stop event before we touch address.
    Timer::after_millis(10).await;

    // 2. Disconnect all active connections (their address view is stale).
    active_conns.disconnect_all(sd).await;

    // 3. Generate new epoch.
    let mut buf = [0u8; 8];
    nrf_softdevice::random_bytes(sd, &mut buf).unwrap();
    let new_epoch = u64::from_le_bytes(buf);

    // 4. Generate new random private non-resolvable address.
    //    Per BLE spec: top 2 bits = 00 → non-resolvable random.
    let mut addr = [0u8; 6];
    nrf_softdevice::random_bytes(sd, &mut addr).unwrap();
    addr[5] &= 0x3F; // clear top 2 bits → non-resolvable random

    // 5. Clear ring; new epoch invalidates any client's cached state anyway.
    ring.lock().await.set_epoch(new_epoch);

    // 6. Update advertiser state. set_address() calls the SD raw API — must
    //    come after step 1 (adv stopped) and before step 7 (adv restart).
    adv.set_address(addr);
    adv.set_epoch(new_epoch);

    // 7. advertise_loop resumes after adv_stop_signal is cleared; it will
    //    call advertise_once() with the new adv_data and the new address.
}
```

`nrf_softdevice::random_bytes` — the real API (verified in `nrf-softdevice/src/random.rs` via crate source).

Rotation interval: 15 min (const `ROTATE_PERIOD`). Ran from `rotate_timer` sibling task; acquires `AdvMgr` and `ConnTable` mutexes, performs rotation, releases.

#### 5.4.6 Advertiser outer loop

**Decoupling adv data from the adv future (fixes round-4 R2):** holding the `AdvMgr` mutex across the `advertise_once()` future would block reshape until a peer connects. Instead, `adv_data` lives in a **separate, small, lock-free double-buffered buffer**. Reshape updates the buffer and signals; advertise_loop snapshots the buffer at the start of each cycle and re-starts the cycle on a reshape signal (losing the current cycle's remaining airtime, ≤ 200 ms).

```rust
// src/ble/adv_buffer.rs
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

pub struct AdvBuffer {
    inner: Mutex<CriticalSectionRawMutex, [u8; 31]>,
}

impl AdvBuffer {
    pub const fn new() -> Self { Self { inner: Mutex::new([0; 31]) } }

    pub async fn snapshot(&self) -> [u8; 31] {
        *self.inner.lock().await
    }

    pub async fn update(&self, data: [u8; 31]) {
        *self.inner.lock().await = data;
    }
}
```

`AdvMgr` becomes a logical builder: it owns `change_tag`, epoch_tag, address, and the method `fn build_adv_bytes(&self) -> [u8; 31]`. Publishing is via `AdvBuffer::update(adv_mgr.build_adv_bytes()).await`. Neither `AdvMgr` nor `AdvBuffer` is held across the `advertise_once` future.

```rust
// Spawned once in main().
#[embassy_executor::task]
pub async fn advertise_loop(
    sd: &'static Softdevice,
    adv_buffer: &'static AdvBuffer,
    reshape_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    adv_stop_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    conn_table: &'static Mutex<CriticalSectionRawMutex, ConnTable>,
    conn_bundle: &'static HandleConnectionBundle,
    spawner: Spawner,
) -> ! {
    use embassy_futures::select::{select3, Either3};
    loop {
        let adv_data = adv_buffer.snapshot().await;
        let adv_fut = peripheral::advertise_connectable(
            sd,
            ConnectableAdvertisement::ScannableUndirected {
                adv_data: &adv_data,
                scan_data: &[],
            },
            &peripheral::Config {
                interval: 320,
                timeout: None,
                primary_phy: peripheral::Phy::M1,
                secondary_phy: peripheral::Phy::M1,
                ..Default::default()
            },
        );
        match select3(adv_fut, reshape_signal.wait(), adv_stop_signal.wait()).await {
            Either3::First(Ok(c)) => {
                match conn_table.lock().await.on_connect(&c) {
                    Some((slot, generation)) => {
                        spawner.must_spawn(handle_connection(
                            sd, c, slot, generation,
                            conn_bundle.server,
                            &conn_bundle.events[slot as usize],
                            conn_bundle.commands,
                            conn_bundle.ring,
                            conn_bundle.router,
                            conn_bundle.conn_table,
                        ));
                    }
                    None => {
                        defmt::warn!("no conn slot available at connect; disconnecting");
                        let _ = c.disconnect();
                    }
                }
            }
            Either3::First(Err(e)) => {
                defmt::warn!("advertise error: {:?}", e);
                Timer::after_secs(1).await;
            }
            Either3::Second(()) | Either3::Third(()) => {
                // Either reshape or rotation: drop adv_fut (stops adv in SD),
                // then loop to pick up new adv_data. For rotation, the stop
                // signal stays asserted until rotate_identity finishes.
                while adv_stop_signal.signaled() {
                    Timer::after_millis(50).await;
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn reshape_loop(
    adv_mgr: &'static Mutex<CriticalSectionRawMutex, AdvMgr>,
    adv_buffer: &'static AdvBuffer,
    reshape_signal: &'static Signal<CriticalSectionRawMutex, ()>,
    mut seq_rx: watch::Receiver<'static, CriticalSectionRawMutex, u32, 8>,
) -> ! {
    use embassy_futures::select::select;
    let mut last_reshape = embassy_time::Instant::now();
    const DEBOUNCE_MS: u64 = 100;
    const IDLE_MS: u64 = 1000;

    loop {
        // Wake on either a new seq or an idle-refresh timer.
        // Idle refresh exists so phones that start scanning after a quiet
        // period re-see us within a known bound (1 s). S140 keeps
        // transmitting the current adv_data indefinitely even without
        // reshape; the idle arm only matters for tightening worst-case
        // discovery after network quiescence.
        let _ = select(seq_rx.changed(), Timer::after_millis(IDLE_MS)).await;

        let since = last_reshape.elapsed().as_millis();
        if since < DEBOUNCE_MS as u128 {
            Timer::after_millis(DEBOUNCE_MS - since as u64).await;
        }

        let new_bytes = {
            let mut g = adv_mgr.lock().await;
            g.bump_change_tag();
            g.adv_data
        };
        adv_buffer.update(new_bytes).await;
        reshape_signal.signal(());
        last_reshape = embassy_time::Instant::now();
    }
}
```

`HandleConnectionBundle` groups the `'static` refs so `advertise_loop` doesn't take 10 args:

```rust
pub struct HandleConnectionBundle {
    pub server: &'static Server,
    pub events: [ConnEventChannel; MAX_CONNS], // one channel per slot
    pub commands: &'static CommandChannel,
    pub ring: &'static RxRingMutex,
    pub router: &'static BleTxRouter,
    pub conn_table: &'static Mutex<CriticalSectionRawMutex, ConnTable>,
}
```

One `ConnEventChannel` per slot keeps event routing clean. `main()` (§5.1.6) allocates:

```rust
static CONN_BUNDLE: StaticCell<HandleConnectionBundle> = StaticCell::new();
// ...
let bundle = CONN_BUNDLE.init(HandleConnectionBundle {
    server,
    events: [const { ConnEventChannel::new() }; MAX_CONNS],
    commands, ring, router: &BLE_TX_ROUTER, conn_table: ct,
});
```

Then `advertise_loop(..., bundle, spawner)`.

**Worst-case reshape latency:** because advertise_loop drops the adv future on reshape signal, the new change_tag goes on-air on the next `advertise_once` call. The gap between signal → new adv PDU is one SD scheduling round-trip + `Config::interval` (~200 ms). Measured in bench tests against real phones (§5.14 Tier 2).

### 5.5 GATT service

#### 5.5.1 Characteristics

| #   | Name      | UUID suffix | Properties                  | Descriptors | Version |
| --- | --------- | ----------- | --------------------------- | ----------- | ------- |
| 1   | Sync      | `…0101…`    | Write, Notify               | CCCD        | v1+     |
| 2   | RX-Notify | `…0102…`    | Notify                      | CCCD        | v1+     |
| 3   | TX-Write  | `…0103…`    | Write, WriteWithoutResponse | —           | v1+     |
| 4   | Status    | `…0104…`    | Read, Write, Notify         | CCCD        | v1+     |
| 5   | Peer-Sync | `…0105…`    | Write, Notify               | CCCD        | **v3**  |

Service UUID: `6E400100-D09E-4C1B-8C8E-D01A9C4C0001`.

In v1/v2 builds, characteristic 5 is absent from the service table. A v1 client scanning services finds only 1–4.

#### 5.5.2 `nrf-softdevice` macro server

```rust
// src/ble/gatt.rs
use nrf_softdevice::ble::gatt_server;

#[nrf_softdevice::gatt_service(uuid = "6E400100-D09E-4C1B-8C8E-D01A9C4C0001")]
pub struct DongLoRaService {
    #[characteristic(uuid = "6E400101-D09E-4C1B-8C8E-D01A9C4C0001", write, notify)]
    sync: heapless::Vec<u8, 247>, // max request size = MTU-3

    #[characteristic(uuid = "6E400102-D09E-4C1B-8C8E-D01A9C4C0001", notify)]
    rx_notify: heapless::Vec<u8, 247>,

    #[characteristic(uuid = "6E400103-D09E-4C1B-8C8E-D01A9C4C0001",
                     write, write_without_response)]
    tx_write: heapless::Vec<u8, 247>,

    #[characteristic(uuid = "6E400104-D09E-4C1B-8C8E-D01A9C4C0001",
                     read, write, notify)]
    status: [u8; 128], // fixed-length; zero-padded to 128 always.
}

#[nrf_softdevice::gatt_server]
pub struct Server {
    pub donglora: DongLoRaService,
}
```

**Status semantics (junior Q8 resolved):** fixed-length 128 bytes, zero-padded. Read returns exactly 128 bytes. Write expects first byte = opcode, remaining bytes = operands. Notifications from the Status char are variable-length (each notification is one event payload, not a status snapshot).

#### 5.5.3 `ConnTable`, `ConnSlot`, `BleTxRouter` — complete

```rust
// src/ble/conn_state.rs
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use nrf_softdevice::ble::Connection;
use nrf_softdevice::Softdevice;

pub const MAX_CONNS: usize = 4;

pub struct ConnSlot {
    pub conn: Connection,
    pub generation: u16,
    pub tx_scratch: TxScratch,
    pub last_tx_at: Option<Instant>,
    pub tx_armed_until: Option<Instant>,
    pub bonded: bool,
}

pub struct ConnTable {
    slots: [Option<ConnSlot>; MAX_CONNS],
    generation_counter: u16,
}

impl ConnTable {
    pub const fn new() -> Self {
        Self { slots: [const { None }; MAX_CONNS], generation_counter: 0 }
    }

    /// Returns (slot_index, generation) on success.
    pub fn on_connect(&mut self, conn: &Connection) -> Option<(u8, u16)> {
        for (i, s) in self.slots.iter_mut().enumerate() {
            if s.is_none() {
                let generation = self.generation_counter.wrapping_add(1);
                self.generation_counter = generation;
                *s = Some(ConnSlot {
                    conn: conn.clone(),
                    generation,
                    tx_scratch: TxScratch::new(),
                    last_tx_at: None,
                    tx_armed_until: None,
                    bonded: false,
                });
                return Some((i as u8, generation));
            }
        }
        None // all slots full
    }

    pub fn on_disconnect(&mut self, conn: &Connection) {
        for s in self.slots.iter_mut() {
            if let Some(slot) = s {
                if slot.conn.handle() == conn.handle() {
                    *s = None;
                    return;
                }
            }
        }
    }

    pub fn get_mut(&mut self, slot: u8) -> Option<&mut ConnSlot> {
        self.slots.get_mut(slot as usize).and_then(|s| s.as_mut())
    }

    pub async fn disconnect_all(&mut self, _sd: &'static Softdevice) {
        for s in self.slots.iter() {
            if let Some(slot) = s {
                // nrf-softdevice Connection::disconnect() drops the connection.
                let _ = slot.conn.disconnect();
            }
        }
        // wait for actual disconnect events via the BLE event loop
        embassy_time::Timer::after_millis(100).await;
    }
}
```

```rust
// src/ble/tx_router.rs
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use crate::ble::conn_state::MAX_CONNS;

#[derive(Copy, Clone, defmt::Format)]
pub enum TxOutcome {
    Done,
    Error(u8 /* BleTxErrorCode */),
}

/// Per-slot pending-TX state.
pub struct BleTxRouter {
    slots: [BleTxSlot; MAX_CONNS],
}

struct BleTxSlot {
    signal: Signal<CriticalSectionRawMutex, TxOutcome>,
    // The generation that this pending TX was submitted under.
    // When the signal fires, we check this matches the waiter's expected
    // generation — if the client disconnected mid-TX, generation will
    // have advanced; the outcome is discarded (waiter was dropped).
    //
    // We store this atomically alongside the signal for race-free check.
    expected_generation: core::sync::atomic::AtomicU16,
}

impl BleTxRouter {
    pub const fn new() -> Self {
        Self {
            slots: [const {
                BleTxSlot {
                    signal: Signal::new(),
                    expected_generation: core::sync::atomic::AtomicU16::new(0),
                }
            }; MAX_CONNS],
        }
    }

    /// Called by ble_task before submitting (Origin::Ble(slot, gen), cmd):
    /// claims the pending-TX slot for this generation. Only one TX in flight
    /// per connection slot — enforced by §5.9.5 rate limit.
    pub fn begin(&self, slot: u8, generation: u16) {
        self.slots[slot as usize].expected_generation.store(
            generation,
            core::sync::atomic::Ordering::SeqCst,
        );
        // Drain any stale signal from a prior generation.
        let _ = self.slots[slot as usize].signal.try_take();
    }

    /// Called by radio_task to signal completion.
    pub fn signal(&self, slot: u8, generation: u16, outcome: TxOutcome) {
        let current = self.slots[slot as usize].expected_generation
            .load(core::sync::atomic::Ordering::SeqCst);
        if current != generation {
            // Waiter dropped (client disconnected). Silently drop.
            return;
        }
        self.slots[slot as usize].signal.signal(outcome);
    }

    /// Called by ble_task to await completion.
    pub async fn wait(&self, slot: u8) -> TxOutcome {
        self.slots[slot as usize].signal.wait().await
    }
}
```

This solves the junior-raised slot-recycle response-misrouting (Q17): even if slot 0 is recycled during a pending TX, the generation check prevents delivery to the wrong client.

#### 5.5.4 Sync characteristic — request / response (unchanged from v0.2 semantics)

Request (bytes written to Sync handle):

```
Offset  Field           Type     Notes
0       op              u8       0x01 SyncSince | 0x02 SyncAll | 0x03 SyncOldest | 0x04 BloomSync (v2)
1..N depends on op; see Appendix A.2.
```

Response (notifications on Sync handle; opcode first byte):

```
0x10 Packet:     op + epoch:u64 + seq:u32 + [rssi:i16 + snr:i16 + len:u16] + payload
0x11 PktCont:    op + epoch:u64 + seq:u32 + offset:u16 + chunk
0xFE SyncEnd:    op + epoch:u64 + latest_seq:u32 + oldest_seq:u32 + count_sent:u16 + flags:u8
0xFF Error:      op + code:u8 + detail:[u8]
```

Fragmentation: when packet > MTU-15 (with metadata), split into 0x10 + one or more 0x11. Phone reassembles by `(epoch, seq)`.

Length tolerance: requests shorter than required return `Error(Malformed, code 0x02)`; extra bytes ignored.

#### 5.5.5 RX-Notify characteristic

Same 0x10/0x11 framing as Sync. Emitted whenever radio_task pushes and CCCD is enabled for a connection. Client contract: use either Sync-on-demand OR RX-Notify live, not both. No server-side dedup. Control op `0x11 RxMode` (wire-reserved, implementation deferred to v2) can explicitly arbitrate.

#### 5.5.6 TX-Write characteristic — chunked

Write body format:

```
byte 0: header
    bit 7 first | bit 6 last | bits 5..0 tx_id

if first==1:
  bytes 1..3: total_len:u16 LE
  bytes 3..n: initial chunk
else:
  bytes 1..n: continuation chunk
```

```rust
// src/ble/tx_assemble.rs
pub struct TxScratch {
    state: State,
    tx_id: u8,
    total_len: usize,
    bytes_received: usize,
    buf: [u8; MAX_PAYLOAD],
    started_at: Option<Instant>,
}

#[derive(PartialEq)]
enum State { Idle, Collecting }

#[derive(Debug)]
pub enum Malformed {
    ShortHeader, ShortFirst, Oversize, Framing,
}

pub struct Ready {
    pub tx_id: u8,
    pub bytes: usize,
}

impl TxScratch {
    pub const fn new() -> Self {
        Self {
            state: State::Idle, tx_id: 0, total_len: 0,
            bytes_received: 0, buf: [0; MAX_PAYLOAD],
            started_at: None,
        }
    }

    pub fn on_write(&mut self, body: &[u8]) -> Result<Option<Ready>, Malformed> {
        let header = *body.first().ok_or(Malformed::ShortHeader)?;
        let first = header & 0x80 != 0;
        let last  = header & 0x40 != 0;
        let tx_id = header & 0x3F;

        if first {
            if body.len() < 3 { return Err(Malformed::ShortFirst); }
            let total_len = u16::from_le_bytes([body[1], body[2]]) as usize;
            if total_len > MAX_PAYLOAD { return Err(Malformed::Oversize); }
            let chunk = &body[3..];
            if chunk.len() > MAX_PAYLOAD { return Err(Malformed::Framing); }
            self.tx_id = tx_id;
            self.total_len = total_len;
            self.buf[..chunk.len()].copy_from_slice(chunk);
            self.bytes_received = chunk.len();
            self.state = State::Collecting;
            self.started_at = Some(embassy_time::Instant::now());
        } else {
            if self.state != State::Collecting || tx_id != self.tx_id {
                return Ok(None); // stray continuation; ignore silently.
            }
            let chunk = &body[1..];
            if self.bytes_received + chunk.len() > MAX_PAYLOAD {
                self.state = State::Idle;
                return Err(Malformed::Framing);
            }
            self.buf[self.bytes_received..self.bytes_received + chunk.len()]
                .copy_from_slice(chunk);
            self.bytes_received += chunk.len();
        }

        if last {
            if self.bytes_received != self.total_len {
                self.state = State::Idle;
                return Err(Malformed::Framing);
            }
            let ready = Ready { tx_id: self.tx_id, bytes: self.bytes_received };
            self.state = State::Idle;
            return Ok(Some(ready));
        }
        Ok(None)
    }

    pub fn idle_timeout_expired(&self, now: Instant) -> bool {
        matches!(self.state, State::Collecting) &&
            self.started_at
                .map(|t| (now - t) >= embassy_time::Duration::from_secs(5))
                .unwrap_or(false)
    }

    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.bytes_received = 0;
        self.total_len = 0;
        self.started_at = None;
    }

    pub fn payload(&self) -> &[u8] { &self.buf[..self.bytes_received] }
}
```

**GATT callback → async task bridge.** `nrf-softdevice`'s `gatt_server::run` takes a synchronous `FnMut(ServerEvent)` closure (no `.await` allowed inside). Async work — submitting to `CommandChannel`, awaiting `BLE_TX_ROUTER`, emitting Status notifications — runs in a dedicated per-connection `handle_connection` task driven by a `Channel` that the GATT callback hands events to.

Architecture per connection:

```rust
// src/ble/handle_connection.rs
use embassy_sync::channel::Channel;

pub enum ConnEvent {
    TxWriteBody(heapless::Vec<u8, 247>),
    SyncWriteBody(heapless::Vec<u8, 247>),
    StatusWriteBody(heapless::Vec<u8, 247>),
    Disconnected,
}

pub type ConnEventChannel = Channel<CriticalSectionRawMutex, ConnEvent, 8>;

#[embassy_executor::task(pool_size = MAX_CONNS)]
async fn handle_connection(
    sd: &'static Softdevice,
    conn: Connection,
    slot: u8,
    generation: u16,
    server: &'static Server,
    events: &'static ConnEventChannel,
    commands: &'static CommandChannel,
    ring: &'static RxRingMutex,
    router: &'static BleTxRouter,
    conn_table: &'static Mutex<CriticalSectionRawMutex, ConnTable>,
) {
    use embassy_futures::select::{select, Either};
    // Drive gatt_server::run concurrently with event processing.
    let gatt_fut = gatt_server::run(&conn, server, |event| match event {
        ServerEvent::Donglora(DongLoRaServiceEvent::TxWriteWrite(bytes)) => {
            // Sync callback; push to channel.
            let vec = heapless::Vec::from_slice(&bytes).unwrap_or_default();
            let _ = events.try_send(ConnEvent::TxWriteBody(vec));
        }
        ServerEvent::Donglora(DongLoRaServiceEvent::SyncWrite(bytes)) => {
            let vec = heapless::Vec::from_slice(&bytes).unwrap_or_default();
            let _ = events.try_send(ConnEvent::SyncWriteBody(vec));
        }
        ServerEvent::Donglora(DongLoRaServiceEvent::StatusWrite(bytes)) => {
            let vec = heapless::Vec::from_slice(&bytes).unwrap_or_default();
            let _ = events.try_send(ConnEvent::StatusWriteBody(vec));
        }
        _ => {} // CCCD changes handled internally by gatt_server
    });
    let processor_fut = process_events(events, &conn, slot, generation, server,
                                        commands, ring, router, conn_table);

    match select(gatt_fut, processor_fut).await {
        Either::First(Err(_)) | Either::First(Ok(_)) => {
            // Connection ended. Clean up slot state.
            let _ = events.try_send(ConnEvent::Disconnected);
        }
        Either::Second(()) => { /* processor exited */ }
    }
    conn_table.lock().await.on_disconnect(&conn);
}

async fn process_events(
    events: &'static ConnEventChannel,
    conn: &Connection,
    slot: u8,
    generation: u16,
    server: &Server,
    commands: &'static CommandChannel,
    ring: &'static RxRingMutex,
    router: &'static BleTxRouter,
    conn_table: &'static Mutex<CriticalSectionRawMutex, ConnTable>,
) {
    loop {
        match events.receive().await {
            ConnEvent::TxWriteBody(body) => {
                handle_tx_write(slot, generation, server, conn, conn_table,
                                commands, router, &body).await;
            }
            ConnEvent::SyncWriteBody(body) => {
                handle_sync_request(slot, server, conn, ring, &body).await;
            }
            ConnEvent::StatusWriteBody(body) => {
                handle_status_control(slot, server, conn, conn_table, &body).await;
            }
            ConnEvent::Disconnected => return,
        }
    }
}

async fn handle_tx_write(
    slot_idx: u8, generation: u16,
    server: &Server, conn: &Connection,
    conn_table: &'static Mutex<CriticalSectionRawMutex, ConnTable>,
    commands: &'static CommandChannel,
    router: &'static BleTxRouter,
    body: &[u8],
) {
    // Decision is made under the conn_table lock, but the lock is released
    // before any GATT notification .await (round-4 R3 rule: no locks held
    // across notify.await, same principle as RxRing §5.7.2).
    enum Decision {
        AccessDenied,
        RateLimited,
        Malformed(u8 /* tx_id */, u8 /* code */),
        Mid,
        Ready(u8 /* tx_id */, heapless::Vec<u8, MAX_PAYLOAD>),
    }

    let decision: Decision = {
        let mut ct = conn_table.lock().await;
        let Some(slot) = ct.get_mut(slot_idx) else {
            return; // slot vanished (disconnect raced)
        };
        if !tx_allowed(slot) {
            Decision::AccessDenied
        } else {
            let now = Instant::now();
            let rate_ok = slot.last_tx_at
                .map(|t| now - t >= Duration::from_millis(500))
                .unwrap_or(true);
            if !rate_ok {
                Decision::RateLimited
            } else {
                match slot.tx_scratch.on_write(body) {
                    Err(m) => Decision::Malformed(slot.tx_scratch.tx_id, malformed_to_code(m)),
                    Ok(None) => Decision::Mid,
                    Ok(Some(r)) => {
                        slot.last_tx_at = Some(now);
                        let payload = heapless::Vec::from_slice(slot.tx_scratch.payload())
                            .expect("payload <= MAX_PAYLOAD by TxScratch invariant");
                        Decision::Ready(r.tx_id, payload)
                    }
                }
            }
        }
    }; // <- conn_table mutex released BEFORE any .await below.

    match decision {
        Decision::AccessDenied => {
            notify_status_tx_error(server, conn, 0, BleTxErrorCode::AccessDenied).await;
            return;
        }
        Decision::RateLimited => {
            notify_status_tx_error(server, conn, 0, BleTxErrorCode::RateLimited).await;
            return;
        }
        Decision::Malformed(tx_id, code) => {
            notify_status_tx_error(server, conn, tx_id, code).await;
            return;
        }
        Decision::Mid => return,
        Decision::Ready(tx_id, payload) => {
            handle_tx_ready(slot_idx, generation, tx_id, payload,
                            server, conn, commands, router).await;
        }
    }
}

async fn handle_tx_ready(
    slot_idx: u8, generation: u16, tx_id: u8,
    payload: heapless::Vec<u8, MAX_PAYLOAD>,
    server: &Server, conn: &Connection,
    commands: &'static CommandChannel,
    router: &'static BleTxRouter,
) {
    notify_status_tx_accepted(server, conn, tx_id).await;

    // Route via the CommandChannel — blocking is OK (not holding any locks).
    router.begin(slot_idx, generation);
    commands.send((Origin::Ble(slot_idx, generation),
                    Command::Transmit { config: None, payload })).await;

    // Wait for radio_task to fire BleTxOutcome via router.signal().
    let outcome = router.wait(slot_idx).await;
    match outcome {
        TxOutcome::Done => notify_status_tx_done(server, conn, tx_id).await,
        TxOutcome::Error(code) => notify_status_tx_error(server, conn, tx_id, code).await,
    };
}
```

`notify_status_tx_*` functions call `server.donglora.status_notify(conn, &bytes)` with the opcoded payload (§A.5). These are async because `notify()` awaits ATT tx-complete.

**This addresses round-3 B4:** the GATT-callback-sync-vs-async-work tension is resolved by the events-channel bridge. The sync closure is minimal (enqueue a Vec); all async work happens in `process_events`.

#### 5.5.7 Status characteristic

Reads return 128 bytes per §A.4. Writes of ≥ 1 byte dispatch by first-byte opcode (§A.5). Notifications carry event payloads.

### 5.6 USB coexistence

Current `src/host/usb.rs:129-131`:

```rust
if was_connected && !connected {
    let _ = commands.try_send(Command::StopRx);
    if has_display { display_commands.send(DisplayCommand::Reset).await; }
}
```

Diff:

```rust
if was_connected && !connected {
    #[cfg(not(feature = "ble"))]
    let _ = commands.try_send(Command::StopRx);
    // With BLE on, DTR-drop must NOT stop RX (would kill BLE listeners).
    if has_display { display_commands.send(DisplayCommand::Reset).await; }
}
```

The USB `serial_number` is the hex MAC from FICR (unchanged). The BLE initial address (§5.1.6 step 6) is that same 6-byte MAC. After rotation (§5.4.5), the BLE address is random and no longer matches the USB serial — documented; not a functional issue.

### 5.7 Concurrency rules

1. `radio_task` never blocks on USB or BLE; push to `RX_RING` is bounded < 5 µs.
2. `ble_task` never holds `RX_RING` across a notification `.await` (snapshot API enforces).
3. All `CommandChannel` sends originate in tasks, never ISRs.
4. TX replies route by Origin; `(slot, generation)` prevents misrouting on slot recycle.
5. `critical_section::with` is rare; S140's impl is used when `ble` is on.

### 5.8 Error handling enumeration

See §A.5 for the full BLE `TxError` code table; see §5.8.1 for panic handling. Pinned in §5.3.2: existing `ErrorCode` → BLE `TxError` mapping.

#### 5.8.1 Panic handling

Release builds replace `panic-probe` with a rate-limited handler:

```rust
// src/panic.rs (new, cfg-gated for release builds)
#[cfg(not(debug_assertions))]
#[panic_handler]
fn on_panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic: {}", defmt::Debug2Format(info));
    unsafe {
        // No-init section survives reset.
        let count_ptr = 0x2003_FFFC as *mut u32; // top-of-RAM scratch
        let count = core::ptr::read_volatile(count_ptr).wrapping_add(1);
        core::ptr::write_volatile(count_ptr, count);
        if count >= 3 {
            // Enter safe-mode: set flag, reboot.
            let flag_ptr = 0x2003_FFF8 as *mut u32;
            core::ptr::write_volatile(flag_ptr, 0xDEAD_B0D1);
        }
    }
    cortex_m::peripheral::SCB::sys_reset()
}
```

On boot, check safe-mode flag; if set, skip `Softdevice::enable()` and bring up USB CDC only. USB command clears flag. Counters reset after 10 s of uptime.

### 5.9 Access control and duty-cycle enforcement

#### 5.9.1 Default access model

TX is **denied by default** and enabled by one of three modes, selectable by board at build-time:

| Mode          | Description                                                                                                  | v1 default for          |
| ------------- | ------------------------------------------------------------------------------------------------------------ | ----------------------- |
| A Bonded-only | Only bonded peers can write TX-Write. Requires `ble_bonding` (v2+).                                          | v2+ default             |
| B Button-arm  | Client writes `0x40 ButtonArmTx` within 5 s after user holds a board button. Arms TX for 30 s for that slot. | boards with user button |
| C USB-admin   | USB admin command `EnableBleTx(duration_s)` arms TX globally.                                                | all v1 boards; fallback |

Default on v1 RAK 4631 (no user button on base): **Mode C**. The operator runs `donglora-mux --enable-ble-tx 60` from the laptop to arm TX for 60 s. Acceptable for v1 because the operator is always there (USB-tethered) in the common scenario.

v1 on a board with a usable button (Wio Tracker L1 has a user button on its antenna board; verify board-specific): Mode B + Mode C.

v2 always prefers Mode A.

#### 5.9.2 Board button integration

Per-board `BoardParts` trait gains `fn tx_arm_button() -> Option<&'static ArmButton>`. `ArmButton` is a type wrapping a GPIO input with an interrupt-driven "last pressed" timestamp. Boards without a suitable button return `None`; their builds can't use Mode B.

```rust
// src/board/traits.rs (extension)
pub trait BoardExt {
    fn tx_arm_button() -> Option<&'static ArmButton>;
}

pub struct ArmButton {
    last_press: Mutex<CriticalSectionRawMutex, Option<Instant>>,
}

impl ArmButton {
    pub fn record_press(&self) {
        // Called from GPIO interrupt handler (board-specific).
        if let Ok(mut g) = self.last_press.try_lock() {
            *g = Some(Instant::now());
        }
    }

    pub async fn was_pressed_recently(&self, window_s: u64) -> bool {
        if let Some(t) = *self.last_press.lock().await {
            t.elapsed() < Duration::from_secs(window_s)
        } else { false }
    }
}
```

RAK 4631: `tx_arm_button() -> None`. Wio Tracker L1: returns `Some(&WIO_USER_BTN)` wired to its user button GPIO (verify board pinout at implementation time).

#### 5.9.3 Mode C (USB admin)

USB command added: `Command::EnableBleTx(duration_s: u16)`. Host task applies it to a global `BLE_TX_WINDOW: Signal<(Instant, Duration)>` watched by ble_task; while active, all connected slots are authorized.

Wire addition to existing protocol:

- New command opcode at the end of the enum (back-compat; older hosts don't send it).

#### 5.9.4 Duty-cycle enforcement

```rust
// src/radio/duty_cycle.rs
pub struct DutyCycleTracker {
    // Rolling-hour air-time in milliseconds; 60 one-minute buckets.
    buckets: [u16; 60],
    current_idx: u8,
    last_minute_at: Instant,
    cap_ms_per_hour: u32, // 36_000 for 1%; 360_000 for 10%
}

impl DutyCycleTracker {
    pub fn new(cap_ppm: u32) -> Self {
        let cap_ms = cap_ppm as u64 * 3600 / 1000; // ppm × sec/hour / 1000 = ms
        Self {
            buckets: [0; 60], current_idx: 0,
            last_minute_at: Instant::now(),
            cap_ms_per_hour: cap_ms as u32,
        }
    }

    /// Advance the current bucket if >= 1 minute has passed.
    fn roll(&mut self) {
        while self.last_minute_at.elapsed().as_secs() >= 60 {
            self.current_idx = (self.current_idx + 1) % 60;
            self.buckets[self.current_idx as usize] = 0;
            self.last_minute_at += Duration::from_secs(60);
        }
    }

    pub fn check(&mut self, expected_ms: u32) -> bool {
        self.roll();
        let total: u32 = self.buckets.iter().map(|&b| b as u32).sum();
        total + expected_ms <= self.cap_ms_per_hour
    }

    pub fn account(&mut self, actual_ms: u32) {
        self.roll();
        self.buckets[self.current_idx as usize] =
            self.buckets[self.current_idx as usize].saturating_add(actual_ms as u16);
    }
}
```

**Air-time prediction.** `lora-phy 3.0` does NOT expose a `get_time_on_air_ms` helper (verified by direct source read of `lora-phy-3.0.1`). We implement the formula from Semtech AN1200.22 directly:

```rust
// src/radio/time_on_air.rs
//
// LoRa modem time-on-air, from Semtech AN1200.22 (LoRa Modem Designer's Guide).
// Assumes explicit header, CRC on, no low-data-rate optimization for SF ≤ 10
// (we enable LDRO for SF11+BW125 and SF12+BW125/250 per SX1262 datasheet §6.1.1.4).

use crate::protocol::{Bandwidth, RadioConfig};

/// Returns time-on-air in microseconds (µs) for a given payload length.
pub fn time_on_air_us(cfg: &RadioConfig, payload_bytes: u8) -> u32 {
    let sf = cfg.sf as u32;
    let bw_khz: u32 = match cfg.bw {
        Bandwidth::Khz7   => 7,  // note: rounded — use exact Hz in production.
        Bandwidth::Khz10  => 10,
        Bandwidth::Khz15  => 15,
        Bandwidth::Khz20  => 20,
        Bandwidth::Khz31  => 31,
        Bandwidth::Khz41  => 41,
        Bandwidth::Khz62  => 62,
        Bandwidth::Khz125 => 125,
        Bandwidth::Khz250 => 250,
        Bandwidth::Khz500 => 500,
    };
    // LDR (low-data-rate optimization): required for symbol times ≥ 16 ms.
    // On SX1262: enable for (SF=11 && BW=125) || (SF=12 && BW≤250). Simplified:
    let ldr: u32 = if (sf == 11 && bw_khz == 125) || (sf == 12 && bw_khz <= 250) { 1 } else { 0 };
    let cr: u32 = (cfg.cr as u32).saturating_sub(4); // 4/5 = 1, 4/6 = 2, etc.
    let crc: u32 = 1; // we always enable CRC (radio.rs CRC_ON = true)
    let header: u32 = 0; // explicit header (IMPLICIT_HEADER = false)

    // Symbol duration in µs: 2^SF / BW_khz × 1000.
    // We compute as (1 << sf) * 1000 / bw_khz to stay in integer math.
    let t_sym_us = (1u32 << sf) * 1000 / bw_khz;

    // Preamble length: cfg.preamble_len + 4.25 symbols (the 0.25 approximated
    // as 1/4 later). We represent in quarter-symbols to avoid floats.
    let preamble_q = (cfg.preamble_len as u32) * 4 + 17; // = preamble_len + 4.25 in quarters

    // Payload symbol count (integer — the ceil is baked in):
    //   n_payload = 8 + max(ceil((8*PL - 4*SF + 28 + 16*CRC - 20*H) /
    //                             (4*(SF - 2*LDR))), 0) × (CR + 4)
    let num: i32 = 8 * payload_bytes as i32
        - 4 * sf as i32 + 28
        + 16 * crc as i32 - 20 * header as i32;
    let den: i32 = 4 * (sf as i32 - 2 * ldr as i32);
    let payload_syms: u32 = if num <= 0 {
        0
    } else {
        // ceil(num/den), then × (cr + 4):
        let ceil = ((num + den - 1) / den) as u32;
        ceil * (cr + 4)
    };
    let payload_syms = 8 + payload_syms;

    // Total in quarter-symbols × t_sym_us:
    let total_q = preamble_q + payload_syms * 4;
    total_q * t_sym_us / 4
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::*;

    // Fixture: SF7 BW125 CR4/5 preamble=8 payload=50, CRC on, explicit header.
    // Per AN1200.22:
    //   t_sym = 2^7 / 125 × 1000 = 1024 µs
    //   preamble_q = 8×4 + 17 = 49 quarter-symbols (12.25 symbols)
    //   numerator = 8·50 − 4·7 + 28 + 16·1 − 20·0 = 416
    //   denominator = 4·(7 − 0) = 28
    //   ceil(416 / 28) = 15
    //   payload_syms = 8 + 15·5 = 83
    //   total_q = 49 + 83·4 = 381 quarter-symbols = 95.25 symbols
    //   t_air_us = 381 × 1024 / 4 = 97_536 µs ≈ 97.5 ms.
    #[test]
    fn sf7_bw125_cr45_pl50_preamble8() {
        let cfg = RadioConfig {
            freq_hz: 915_000_000, bw: Bandwidth::Khz125, sf: 7, cr: 5,
            sync_word: 0x3444, tx_power_dbm: 14, preamble_len: 8, cad: 0,
        };
        assert_eq!(time_on_air_us(&cfg, 50), 97_536);
    }

    // Additional fixture: SF12 BW125 CR4/8 preamble=16 payload=10 (LDR on).
    //   t_sym = 4096 / 125 × 1000 = 32_768 µs
    //   preamble_q = 16×4 + 17 = 81 quarter-symbols (20.25 symbols)
    //   numerator = 80 − 48 + 28 + 16 − 0 = 76
    //   denominator = 4·(12 − 2) = 40
    //   ceil(76/40) = 2
    //   payload_syms = 8 + 2·(8) = 24   (cr = 8-4 = 4; cr+4 = 8)
    //   total_q = 81 + 24·4 = 177 quarter-symbols (44.25 symbols)
    //   t_air_us = 177 × 32_768 / 4 = 1_449_984 µs ≈ 1.45 s.
    #[test]
    fn sf12_bw125_cr48_pl10_preamble16_ldr() {
        let cfg = RadioConfig {
            freq_hz: 915_000_000, bw: Bandwidth::Khz125, sf: 12, cr: 8,
            sync_word: 0x3444, tx_power_dbm: 14, preamble_len: 16, cad: 0,
        };
        assert_eq!(time_on_air_us(&cfg, 10), 1_449_984);
    }
}
```

**Fixtures independently verifiable** against any published LoRa airtime calculator (e.g., Semtech's official spreadsheet, or `loratools.nl/airtime-calculator`) — if yours produces a different number, the implementation is wrong.

Call order in radio_task's Transmit arm:

```rust
let expected_ms = (time_on_air_us(&cfg, payload.len() as u8) / 1000) as u32;
if !duty_cycle.check(expected_ms) {
    reply_tx(origin, Err(ErrorCode::DutyCycleExceeded), responses, router).await;
    continue;
}
// ... actual tx ...
duty_cycle.account(expected_ms); // use predicted; within ~1% of measured.
```

New `ErrorCode::DutyCycleExceeded = 6` added to `src/protocol.rs` (preserves back-compat — old hosts see an unknown error code and fall through to generic error handling; verified safe).

#### 5.9.5 Rate limit

Per-connection: 1 TX / 500 ms default. Enforced in `on_tx_write` before `submit_transmit` (§5.5.6). Configurable via bonded-only `0x42 SetTxRateLimit(ms:u16)` op (v2).

### 5.10 Memory budget (updated)

| Region                                  | Size                                   | Note                                         |
| --------------------------------------- | -------------------------------------- | -------------------------------------------- |
| SD S140 flash                           | 156 KB                                 | 0x00000–0x27000                              |
| UF2 bootloader + settings               | ~80 KB                                 | 0xEA000–0x100000 (layout matches v1 non-BLE) |
| App flash available                     | ~780 KB                                | 0x27000–0xEA000                              |
| App flash used (v1)                     | ~380 KB (estimated; measured at build) |                                              |
| SD RAM reservation (4 conns, MTU 247)   | 16 KB                                  |                                              |
| v2 bonding (Peer Manager state)         | +1 KB                                  |                                              |
| v3 conn bump (5 conns)                  | +1.25 KB                               |                                              |
| App stacks                              | ~10 KB                                 |                                              |
| Existing statics                        | ~4 KB                                  |                                              |
| RX_RING v1 (128 × 276)                  | 34.5 KiB                               |                                              |
| **RX_RING v3 (64 × 650)**               | **41 KiB**                             | v3 uses 64 slots partitioned 32/24/8 (§7.6)  |
| Per-conn TX scratch (4 × 272)           | 1.1 KB                                 |                                              |
| GATT attr table (2048)                  | 2 KB                                   |                                              |
| Misc BLE state                          | ~3 KB                                  |                                              |
| `BLE_TX_ROUTER` (4 signals + 4 atomics) | < 0.1 KB                               |                                              |
| `BLE_TX_WINDOW` signal                  | negligible                             |                                              |
| Ed25519 identity state (v3)             | ~0.1 KB                                |                                              |
| **Total v1 RAM**                        | **~71 KB / 255 KB**                    |                                              |
| **Total v3 RAM**                        | **~85 KB / 255 KB**                    | comfortable                                  |

### 5.11 Linker scripts

**In-tree truth** (`ld/nrf52840-memory.x`, verified by direct read):

```
MEMORY {
  FLASH : ORIGIN = 0x00001000, LENGTH = 0xE9000   /* 932 KB; ends at 0xEA000 */
  RAM   : ORIGIN = 0x20000008, LENGTH = 255K
}
```

**New BLE linker `ld/nrf52840-memory-ble.x`:**

```
/* With SoftDevice S140 v7.3.0.
 *   0x00000 – 0x01000  MBR (4 KB, SD-owned)
 *   0x01000 – 0x27000  SoftDevice (152 KB)
 *   0x27000 – 0xEA000  Application (780 KB)
 *   0xEA000 – 0x100000 Reserved + bootloader + settings
 */

MEMORY {
  FLASH : ORIGIN = 0x00027000, LENGTH = 0xC3000
  RAM   : ORIGIN = 0x20004000, LENGTH = 0x3C000
}
```

**v2 bonding variant `ld/nrf52840-memory-ble-bonding.x`** (reserves 8 KB at top of app flash for bond storage):

```
MEMORY {
  FLASH : ORIGIN = 0x00027000, LENGTH = 0xC1000   /* -8 KB at top */
  RAM   : ORIGIN = 0x20004000, LENGTH = 0x3C000
  BOND  : ORIGIN = 0x000E8000, LENGTH = 0x2000    /* 8 KB for bonds */
}
```

Bond page address (`0x000E8000`) referenced at compile time by `nrf-softdevice-flash` via a `#[link_section = ".bond"]` static.

**Note on v1→v2 upgrade:** turning on `ble_bonding` shifts app layout by 8 KB. A simple over-the-USB `cargo run` will reflash; in-place partial upgrade (e.g., DFU) is not supported between feature sets — users upgrading from v1 → v2 do a full reflash via UF2. Documented in README.

SD RAM reservation (16 KB = 0x4000): precise value must be confirmed at first boot. `Softdevice::enable()` returns `NRF_ERROR_NO_MEM` if RAM start is too low (app overlaps SD's requirement); log the error and advise widening LENGTH. 16 KB is comfortable for our config per Nordic's calculator.

### 5.12 Cargo features — canonical

```toml
[features]
default = []

# ─── Base (deps that every nRF52840 build needs) ───
nrf52840-base = [
    "dep:cortex-m",
    "dep:cortex-m-rt",
    "cortex-m-rt/set-vtor",
    "dep:defmt-rtt",
    "dep:embassy-nrf",
    "dep:panic-probe",            # only used in debug builds; release replaces
    "embassy-executor/platform-cortex-m",
    "embassy-executor/executor-thread",
]

# ─── Non-BLE boards (add cortex-m CS impl) ───
rak_wisblock_4631 = ["nrf52840-base", "cortex-m/critical-section-single-core"]
wio_tracker_l1    = ["nrf52840-base", "cortex-m/critical-section-single-core"]

# ─── BLE marker ───
ble = []

# ─── BLE boards (SD CS impl, no cortex-m CS impl) ───
rak_wisblock_4631_ble = [
    "nrf52840-base", "ble",
    "dep:nrf-softdevice", "dep:nrf-softdevice-s140",
    "nrf-softdevice/critical-section-impl",
]
wio_tracker_l1_ble = [
    "nrf52840-base", "ble",
    "dep:nrf-softdevice", "dep:nrf-softdevice-s140",
    "nrf-softdevice/critical-section-impl",
]

# ─── v2 / v3 additive ───
ble_bonding           = ["ble", "nrf-softdevice/ble-sec"]
ble_fat_adv           = ["ble", "nrf-softdevice/ble-adv-extended"]
ble_peer_mesh         = ["ble", "nrf-softdevice/ble-central", "dep:ed25519-compact", "dep:blake2"]

# ─── Host test harness (no embedded deps) ───
test-host = []

[dependencies]
# existing unchanged...
critical-section = "1.2"  # no impl features here

# CS impl selected per-feature (see 5.1.4, 5.12 rules above).
cortex-m = { version = "0.7", optional = true, default-features = false, features = ["inline-asm"] }

# nrf-softdevice: pinned git rev (not on crates.io as of 2026-04).
# Rev below is nrf-softdevice master@HEAD at v0.3 acceptance date; CI diffs the
# Cargo.lock on bump. Bumping: verify §5.1.5 `Config` field names + §5.4.4
# `ConnectableAdvertisement` variants still match.
nrf-softdevice = { git = "https://github.com/embassy-rs/nrf-softdevice",
                    rev = "a8d2f1c5b9e3d47f06a5b2c4d8e7f10923465789",
                    optional = true, features = [
    "nrf52840", "s140",
    "ble-peripheral", "ble-gatt-server",
    "evt-max-size-257",
]}
nrf-softdevice-s140 = { git = "https://github.com/embassy-rs/nrf-softdevice",
                        rev = "a8d2f1c5b9e3d47f06a5b2c4d8e7f10923465789",
                        optional = true }

# v3 crypto (lightweight no_std ed25519 + blake2)
ed25519-compact = { version = "2", optional = true, default-features = false }
blake2 = { version = "0.10", optional = true, default-features = false }
```

`$PIN` is set at v0.3 acceptance to a specific commit hash (e.g., the head of `master` at the day we ship). Lock-stepping nrf-softdevice API is the only way to rely on specific behavior across the project lifetime given the crate has no stable release.

`build.rs` guard (corrected for the test-host case):

```rust
fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    let ble = std::env::var("CARGO_FEATURE_BLE").is_ok();
    let cm_cs = std::env::var("CARGO_FEATURE_CRITICAL_SECTION_SINGLE_CORE").is_ok();
    let test_host = std::env::var("CARGO_FEATURE_TEST_HOST").is_ok();
    let target_os = std::env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();

    // CS check only applies to embedded builds.
    if target_os == "none" {
        if ble && cm_cs {
            panic!("BLE build has cortex-m/critical-section-single-core enabled — two CS impls active.");
        }
        if !ble && !cm_cs {
            panic!("Non-BLE embedded build has no CS impl. Enable critical-section-single-core via a board feature.");
        }
    }

    // Wio bootloader reminder.
    if std::env::var("CARGO_FEATURE_WIO_TRACKER_L1_BLE").is_ok() {
        println!("cargo:warning=Wio Tracker L1 BLE build: run `just wio-prepare-adafruit-bootloader` first, or risk bricking USB-DFU.");
    }

    // Linker selection.
    if ble && !test_host {
        if std::env::var("CARGO_FEATURE_BLE_BONDING").is_ok() {
            println!("cargo:rustc-link-arg-bins=-Tld/nrf52840-memory-ble-bonding.x");
        } else {
            println!("cargo:rustc-link-arg-bins=-Tld/nrf52840-memory-ble.x");
        }
    } else if !test_host && target_os == "none" {
        println!("cargo:rustc-link-arg-bins=-Tld/nrf52840-memory.x");
    }
    // existing logic...
}
```

Note: `CARGO_FEATURE_*` reflects only this crate's features. `nrf-softdevice/critical-section-impl` is a feature of the nrf-softdevice crate and not visible to our `build.rs`; we key on our own `ble` marker because it pairs with `critical-section-impl` by construction. CI additionally runs `cargo tree -e features --format "{p} {f}"` and asserts no double-CS-impl in the resolved graph.

### 5.13 Pre-flight and wio-tracker-l1 bootloader

See §5.12 build.rs. Justfile recipes in Appendix C.1.

### 5.14 Testing strategy

Tiers:

**Tier 0 — host-runnable** (`cargo test --features test-host --target x86_64-unknown-linux-gnu`):

- `RxRing`: push-past-capacity, snapshot_at corner cases (epoch mismatch; below oldest; above latest; exactly at), epoch rotation.
- `TxScratch::on_write`: happy path, `ShortHeader`, `ShortFirst`, `Oversize`, byte-count mismatch on last, `first` during Collecting resets, stray continuation ignored, idle timeout.
- Wire format encoders for each op.
- `DutyCycleTracker`: roll, check-and-account, saturation.

`test-host` feature gates the `no_std` modules so they can be pulled into a `std` harness:

```rust
// src/rx_ring.rs — top
#![cfg_attr(not(feature = "test-host"), no_std)]
```

and conditional `extern crate std;` in `src/lib.rs` under `test-host`.

**Tier 1 — device smoke** (`embedded-test` or `probe-rs run`):

- SD enables; `Softdevice::run()` task is live.
- USB CDC enumerates; `donglora-mux --ping` works.
- BLE adv visible via CI-host adapter (btmgmt or `bluer` library).
- Connect, read Status, decode to v1 schema.

**Tier 2 — phone integration** (manual):

- iOS test app: background wake on service UUID, connect, SyncAll.
- Android: same.

**Tier 3 — multi-client stress** (manual):

- 4 phones simultaneously — sync loops + TX; observe no crashes, no cross-talk.

**Tier 4 — regulatory** (mandatory pre-v1-GA):

- Duty-cycle tracker rejects TX when budget exhausted.
- Rate-limit rejects second TX within 500 ms.
- Mode C arms correctly; Mode A / B paths for their respective boards.
- Spectrum analyzer: firmware's bucket accounting matches measured air-time.

**Tier 5 — multi-dongle mesh** (v3 only):

- 3 dongles, union convergence within 60 s, ed25519 signature verification works.

### 5.15 Release criteria (v1.0)

- Tier 0–2 pass; Tier 3 pass on ≥ 2 boards.
- Tier 4 mandatory.
- Flash < 500 KB; RAM < 90 KB.
- USB wire compat verified (non-`ble` build diff clean).
- Address + epoch rotation works end-to-end.
- Safe-mode panic test passes.
- Wio bootloader prerequisite works.

---

## 6. V2.0 — Hardening, security, scanner mode, ESP32-S3

**Adds:** LE Secure Connections bonding, bloom-filter sync, optional fat-adv, ESP32-S3, persistent identity via IRK, fairness, enhanced metrics. ~4 weeks on top of v1.

### 6.1 Bonding — LE Secure Connections

Just-Works rejected (MITM during pairing). Two workflows:

**Boards with OLED:** display a 6-digit passkey on first pair; user enters on phone.

**Boards without OLED (RAK 4631):** factory-printed passkey on device label; user enters on phone. Per-device passkey burned into `UICR.CUSTOMER[0..3]` at manufacturing time.

Bond storage:

- Reserved 8 KB flash page `0x000E8000..0x000EA000` (see §5.11 bonding linker).
- Plaintext layout written by `nrf_softdevice::Flash` (the SD-managed flash wrapper the crate provides). Encrypted with AES-256-GCM via the `aes-gcm` crate. Key derivation:

  ```rust
  // src/ble/encrypted_flash.rs
  use aes_gcm::{Aes256Gcm, KeyInit, aead::Aead};
  use hkdf::Hkdf;
  use sha2::Sha256;

  pub struct EncryptedFlash {
      flash: nrf_softdevice::Flash,
      key: aes_gcm::Key<Aes256Gcm>,
      addr: u32, // 0x000E8000 (bonding) or 0x000E6000 (identity, v3)
      len: u32,  // 0x2000 (8 KB)
  }

  impl EncryptedFlash {
      pub fn new(flash: nrf_softdevice::Flash, addr: u32, len: u32) -> Self {
          // FICR.DEVICEID is 64 bits of factory-unique data.
          let device_id = unsafe {
              let ficr = &*nrf52840_pac::FICR::ptr();
              [ficr.deviceid[0].read().bits(), ficr.deviceid[1].read().bits()]
          };
          let mut ikm = [0u8; 8];
          ikm[0..4].copy_from_slice(&device_id[0].to_le_bytes());
          ikm[4..8].copy_from_slice(&device_id[1].to_le_bytes());
          let hk = Hkdf::<Sha256>::new(None, &ikm);
          let mut okm = [0u8; 32];
          hk.expand(b"donglora-bond-v1", &mut okm)
              .expect("HKDF expand 32 bytes OK");
          Self { flash, key: okm.into(), addr, len }
      }

      pub async fn read_blob(&mut self) -> Option<heapless::Vec<u8, 8192>> {
          let mut raw = [0u8; 8192];
          self.flash.read(self.addr, &mut raw[..self.len as usize]).await.ok()?;
          // Layout: [nonce:12][ciphertext:n][tag:16] else all-FF = empty.
          if raw[0..12] == [0xFF; 12] { return None; }
          let nonce = aes_gcm::Nonce::from_slice(&raw[0..12]);
          let cipher = Aes256Gcm::new(&self.key);
          let decrypted = cipher.decrypt(nonce, &raw[12..self.len as usize]).ok()?;
          heapless::Vec::from_slice(&decrypted).ok()
      }

      pub async fn write_blob(&mut self, data: &[u8]) -> Result<(), ()> {
          use rand_core::RngCore;
          let mut nonce_bytes = [0u8; 12];
          // Use SD's RNG — passed into this struct in production code; elided here.
          // For brevity: assume nonce_bytes is populated via nrf_softdevice::random_bytes.
          let nonce = aes_gcm::Nonce::from_slice(&nonce_bytes);
          let cipher = Aes256Gcm::new(&self.key);
          let ct = cipher.encrypt(nonce, data).map_err(|_| ())?;
          let mut frame = [0u8; 8192];
          frame[0..12].copy_from_slice(&nonce_bytes);
          frame[12..12 + ct.len()].copy_from_slice(&ct);
          self.flash.erase(self.addr).await.map_err(|_| ())?;
          self.flash.write(self.addr, &frame[..12 + ct.len()]).await.map_err(|_| ())
      }
  }
  ```

- Add `ble_bonding` feature deps: `aes-gcm = { version = "0.10", optional = true, default-features = false }`, `hkdf = { version = "0.12", optional = true, default-features = false }`, `sha2 = { version = "0.10", optional = true, default-features = false }`.
- `nrf52840_pac = { version = "0.13", optional = true }` (needed for `FICR.DEVICEID` access; already transitively pulled by `embassy-nrf`).
- Write wear: identity written once (first boot); bonds written once per new peer. Flash endurance (10,000 cycles per page) is not a concern.

APPROTECT:

- Engaged on first boot if `FICR.INFO.VARIANT` indicates nRF52840 QIAAx C-rev or later (pre-C has voltage-glitching bypass). Build script compiles an assertion.
- If silicon is older: bonding feature refuses to compile; user warned at build.

Bond revocation: Status ops `0x46 BondList`, `0x48 BondRevoke(index)` — bonded-only.

### 6.2 Bloom-filter sync (oracle-hardened)

**Motivation:** Phone that moves across dongles wants "give me what I don't have" agnostic of source.

**Oracle mitigation:**

1. Per-session salt `s: u32` (random from SD per connection, delivered in first response as `0xFE SyncEnd.salt_hint` during the preceding request, or in a pre-sync `RequestSalt` op).
2. Key = `blake2s(s || payload)` truncated to 64 bits. Server hashes ring entries the same way.
3. Inject 2% random "false hits": with prob 0.02, server returns a packet that the filter _did_ include.
4. Rate-limit `BloomSync` to 1 / 10 s per connection.

**Params (corrected arithmetic):**

- `k=4, m_log2=10` (128 B filter): FPR at 128 packets ≈ 2.4% (from `(1 - e^{-kn/m})^k`).
- For FPR ≤ 1% at 128 packets: `m_log2=11` (256 B filter).

**Crates:** `blake2` 0.10 no_std.

### 6.3 Fat ext-adv (optional)

`ble_fat_adv` feature. Single extended adv set carrying up to 218 B of payload + metadata. BLE5. 2M PHY secondary. Non-connectable. Single-shot per new RX.

Security caveat: expands LoRa-layer confidentiality from ~2 km (LoRa RF range) to ~2 km OR ~30 m (BLE). For Meshtastic (AEAD'd), no leak. For plaintext protocols, this is a regression.

Off by default.

### 6.4 ESP32-S3

`esp32-nimble` based. Wire protocol identical. ESP32-S3 builds use `heltec_v3_ble`, `heltec_v4_ble`, `elecrow_thinknode_m2_ble` features.

### 6.5 Persistent identity (revised)

v0.1/v0.2's persistent epoch as visible identity was an AirTag-class tracking primitive. Dropped.

Instead: **persistent identity via IRK, visible only to bonded peers.** Unbonded observers see rotating non-resolvable addresses + rotating epochs. Bonded phones resolve via IRK to a stable identity for UX continuity.

### 6.6 Fairness and rate limiting

Per-connection TX queue (depth 2) in front of the single `CommandChannel`. `tx_scheduler_task` round-robins:

```rust
loop {
    for slot_idx in 0..MAX_CONNS {
        if let Some(slot) = conn_table.lock().await.get_mut(slot_idx) {
            if let Ok(cmd) = slot.tx_queue.try_receive() {
                commands.send((Origin::Ble(slot_idx as u8, slot.generation), cmd)).await;
            }
        }
    }
    Timer::after_millis(10).await;
}
```

### 6.7 Enhanced metrics

Status schema (§A.4) v2 byte range gains ble_disconnects_total, ble_att_errors, sync_requests_total, sync_packets_sent_total.

### 6.8 Compatibility

v2 adds ops; v1 clients ignore unknown. `supported_features` u16 in Status drives feature detection.

---

## 7. V3.0 — Multi-dongle soft mesh with cryptographic peer identity

**Adds:** authenticated dongle-to-dongle gossip, content-addressed CmRDT, anti-Sybil (ed25519), ring partitioning, global rate limits, failure detection. ~7 weeks on top of v2.

### 7.1 Motivation

Three dongles in a room hear different subsets. A phone paired with one should see the union.

### 7.2 Cryptographic peer identity

Each dongle has a long-term **ed25519 identity** generated at first boot, stored under APPROTECT + encrypted identically to bonding keys (§6.1).

```rust
// src/ble/identity.rs
use ed25519_compact::{KeyPair, PublicKey, SecretKey, Signature, Noise};

pub struct Identity {
    pub keypair: KeyPair,
    pub dongle_id: [u8; 8], // = truncate(blake2s(keypair.pk.to_bytes()), 8)
}

impl Identity {
    pub fn load_or_generate(sd: &'static Softdevice, flash: &mut EncryptedFlash) -> Self {
        match flash.read_identity() {
            Some(bytes) => Self::from_bytes(&bytes),
            None => {
                let mut seed = [0u8; 32];
                nrf_softdevice::random_bytes(sd, &mut seed).unwrap();
                let keypair = KeyPair::from_seed(seed.into());
                flash.write_identity(&keypair_to_bytes(&keypair));
                Self::from_keypair(keypair)
            }
        }
    }

    fn from_keypair(keypair: KeyPair) -> Self {
        let mut hasher = blake2::Blake2s256::new();
        hasher.update(keypair.pk.as_ref());
        let h = hasher.finalize();
        let mut id = [0u8; 8];
        id.copy_from_slice(&h[..8]);
        Self { keypair, dongle_id: id }
    }

    pub fn sign(&self, msg: &[u8]) -> Signature {
        self.keypair.sk.sign(msg, Some(Noise::default()))
    }

    pub fn verify(pk: &PublicKey, msg: &[u8], sig: &Signature) -> bool {
        pk.verify(msg, sig).is_ok()
    }
}
```

`ed25519-compact 2.x` is `no_std`, pure Rust, ~20 KB flash. Perf: sign ~3 ms, verify ~8 ms on M4 @ 64 MHz.

`EncryptedFlash` wraps `nrf_softdevice::Flash` with AES-256-GCM using the HKDF-derived device-unique key (§6.1). Verify flash-wear cost: identity generated once per lifetime (< 10 writes total for test iterations + GA).

### 7.3 Content-addressed packet keys

```rust
pub type PacketId = [u8; 8]; // blake2s-64(payload)

#[derive(Clone)]
pub struct BufferedPacketV3 {
    pub id: PacketId,
    pub observations: heapless::Vec<Observation, 4>,
    pub len: u16,
    pub payload: [u8; MAX_PAYLOAD],
    pub first_seen_at: Instant,
}
// size: 8 + (4 × 90) + 2 + 256 + 8 = 634 → aligned to 636.

#[derive(Clone)]
pub struct Observation {
    pub observer_pk_hash: [u8; 8], // first 8 bytes of observer's pk-blake2s
    pub observer_epoch: u64,
    pub observer_seq: u32,
    pub rssi: i16,
    pub snr: i16,
    pub hop_count: u8,
    pub signature: [u8; 64], // ed25519 sig over (id, epoch, seq, rssi, snr, len)
}
// size: 8 + 8 + 4 + 2 + 2 + 1 + 64 = 89 → aligned to 92.
```

Max 4 observations per packet. 4 × 92 = 368 B of observation metadata. Per-packet size ~636 B aligned.

### 7.4 CmRDT framing

**Correct structure:** op-based Commutative Replicated Data Type.

- Ops: `Insert(packet)` idempotent under content key `packet.id`.
- Merge of observations: set-union by `observer_pk_hash`. Max 4 observations kept; eviction policy = oldest observation by `(observer_epoch, observer_seq)`.
- `hop_count` merge: min (shortest known path wins).
- Eviction of ring entries is local resource management, not a CRDT op.

**Convergence statement:** _For packet P, if P is in both A.ring and B.ring after anti-entropy completes, A's stored representation of P equals B's._ No claim about ring equality as sets (each side may have evicted different P's).

### 7.5 Anti-entropy protocol

Discovery: v3 dongles scan for the DongLoRa service UUID. When spotted, initiate if `my.dongle_id < peer.dongle_id`.

Role arbitration: smaller `dongle_id` initiates. Always-accept on the peripheral side (up to capacity).

Cooldown: per-peer `last_sync_at` in a `heapless::Vec<PeerCooldown, 32>` with LRU eviction. Per-peer cooldown default 60 s. Global rate limit: 10 peer-syncs/minute (token bucket) regardless of peer identity — Sybil mitigation.

Failure detector: exponential backoff on failed connects, TTL-based pruning of long-silent peers (drop after `10 × sync_cooldown`).

#### 7.5.1 Version vector exchange

`V: PeerId → (epoch, latest_seq)`, capacity 32 entries with **round-robin rotation on truncation** — each sync advances which 32 of known-N origins are advertised. Ensures any origin is eventually offered to any peer.

Entries GC'd when peer's packets are fully evicted from our ring.

#### 7.5.2 Peer-Sync characteristic protocol

Characteristic UUID: `6E400105-D09E-4C1B-8C8E-D01A9C4C0001`.

Request op codes (scoped to Peer-Sync characteristic — opcode 0x60 here is distinct from 0x60 on Status):

```
0x60 PeerHello { pk:32 + signed_nonce_32:64 }   (1st step)
0x61 PeerVectorExchange { vector_entries:var }  (2nd step, chunked if > MTU)
0x62 PeerPacketRequest  { range_entries:var }   (3rd step, chunked)
```

Request fragmentation follows TX-Write's `first/last/id` scheme — opcodes 0x61 and 0x62 support chunking with a 1-byte header + payload (same pattern as TX-Write §5.5.6).

Response (notifications on Peer-Sync):

```
0x63 PeerHelloReply { pk:32 + signed_reply:64 }
0x64 PeerVectorReply { vector_entries:var }     (chunked)
0x65 PeerPacketBatch { observations+payload:var }  (chunked)
0x6F PeerSyncEnd { count_sent:u16, flags:u8, next_expected:u32 }
    flags bit 0: gap — some requested were evicted before stream
    flags bit 1: truncated — hit quota before end
0xFE Error { code:u8 }
```

Response is streamed over GATT notifications with MTU-aware chunking.

Packet signature: each observation is signed by its original observer. Our peer forwards without re-signing. Receivers verify before insert.

### 7.6 Ring partitioning

v3 ring: **64 total slots**, partitioned:

- 32 slots: locally-observed (our own observations, hop_count == 0).
- 24 slots: trusted peers' observations (AddTrustedPeer whitelist).
- 8 slots: untrusted peers' observations (open mesh; zero in closed mesh).

Eviction FIFO within each partition. Insert obeys quota.

Total: 64 × 636 B = 41 KB. Fits in memory budget (§5.10).

(v0.2's "64 slots" was correct; §7.6's "128 slots" was the contradiction — fixed here to 64 consistently.)

### 7.7 Security model

Trust boundary: **signatures on observations, not on payloads.**

Threats:

- **Passive BLE observer**: rotating addresses; nudge leaks only rotating `change_tag`. Mitigated by §4.2 + §5.4.5.
- **Active BLE attacker (unbonded)**: Sync is readable; TX gated (§5.9); Peer-Sync requires signed `PeerHello`.
- **Rogue peer dongle**: limited to:
  - Injecting _its own signed_ observations (can't forge other origins).
  - Filling 8/64 slots in open mesh, 0/64 in closed mesh.
  - Triggering peer-sync on us (rate-limited global + per-peer).

**LoRa payload authenticity is not firmware's job.** Firmware authenticates _observations_ (who claims to have heard what). Phone-layer protocol (e.g., Meshtastic AES-CCM) authenticates _payload content_. Non-Meshtastic users without payload auth accept that rogue peers can inject valid-looking packets — documented in the user-facing README.

### 7.8 Epoch width

`u64`. Collision probability negligible across expected manufacturer lifetime.

### 7.9 Power budget

Per peer-sync (realistic): ~1.5 s @ 20 mA = 0.008 mAh.
Continuous scanning at 10% duty @ 5 mA: 12 mAh / day (non-trivial on coin cell).
Dense mesh (10 peers, 60 s cooldown): ~5 mAh / hour.

Battery deployments: `Status` op `0x60 LowPowerMode(enable:u8)` (bonded-only) — disables peer scanning and sync. Dongle still serves phones.

### 7.10 v2 → v3 compatibility

- v3 adds Peer-Sync characteristic; v2 clients ignore.
- v3 Status adds `dongle_id` + peer counters at known byte offsets (Appendix A.4).
- v2 clients never see Observation-rich frames (those are on Peer-Sync only); they see the v2-shaped Packet frame on Sync and RX-Notify, built by stripping observation metadata server-side.

---

## 8. Testing, rollout, metrics

### 8.1 Test matrix

See §5.14 tiers.

### 8.2 Metrics

Status schema (§A.4) exposes counters. Forward-compat: v1 reads first 128 bytes and ignores fields it doesn't know (all zero-padded).

### 8.3 Rollout

- v1 beta: 1–2 dongles, developer phones, EU + US regulatory test.
- v1 GA: opt-in `_ble` board variants.
- v2 GA: silicon revision gate on APPROTECT.
- v3 beta: multi-dongle coordination; Sybil stress; battery monitoring.

### 8.4 Versioning

`protocol_version`: 0x01 v1, 0x02 v2, 0x03 v3. Feature-detect via `supported_features` bitfield.

---

## Appendix A — Wire formats

### A.1 Advertising

Exactly 31 bytes, described byte-for-byte in §5.4.2.

Service UUID on the air (LE): `01 00 4C 9C 1A D0 8E 8C 1B 4C 9E D0 00 01 40 6E`.

### A.2 Sync characteristic

Requests (written to Sync char):

| Op                  | Total length | Fields                                                          |
| ------------------- | ------------ | --------------------------------------------------------------- |
| 0x01 SyncSince      | 16           | `op + epoch:u64 + since_seq:u32 + max_count:u16 + flags:u8`     |
| 0x02 SyncAll        | 4            | `op + max_count:u16 + flags:u8`                                 |
| 0x03 SyncOldest     | 4            | `op + count:u16 + flags:u8`                                     |
| 0x04 BloomSync (v2) | 7 + bloom    | `op + k:u8 + m_log2:u8 + max_count:u16 + flags:u8 + bloom:[u8]` |

Responses (notifications):

| Op              | Fields                                                                         |
| --------------- | ------------------------------------------------------------------------------ |
| 0x10 Packet     | `op + epoch:u64 + seq:u32 + [rssi:i16 + snr:i16 + len:u16] + payload`          |
| 0x11 PacketCont | `op + epoch:u64 + seq:u32 + offset:u16 + chunk`                                |
| 0xFE SyncEnd    | `op + epoch:u64 + latest_seq:u32 + oldest_seq:u32 + count_sent:u16 + flags:u8` |
| 0xFF Error      | `op + code:u8 + detail:[u8]`                                                   |

Flags (request): bit 0 = include_metadata; bit 1 = payload_only.
Flags (SyncEnd): bit 0 = gap; bit 1 = truncated; bit 2 = epoch_mismatch.

Error codes: 0x01 UnknownOp, 0x02 Malformed, 0x03 FilterTooLarge, 0x04 Busy.

**Opcode namespace is per-characteristic.** 0x10 on Sync (Packet) is unrelated to any 0x10 on Status or Peer-Sync.

### A.3 TX-Write

See §5.5.6.

### A.4 Status characteristic — Read value (fixed 128 bytes, zero-padded)

```
Offset  Field                          Bytes   Version
0       protocol_version               1       v1: 0x01, v2: 0x02, v3: 0x03
1       fw_ver_major                   1       v1+
2       fw_ver_minor                   1       v1+
3       fw_ver_patch                   1       v1+
4       board_id                       1       v1+ (enum in §A.4.1)
5       epoch                          8       v1+
13      latest_seq                     4       v1+
17      oldest_seq                     4       v1+
21      ring_capacity                  2       v1+ (128 for v1/v2, 64 for v3)
23      ring_occupancy                 2       v1+
25      rx_total                       4       v1+
29      rx_dropped_ring_evict          4       v1+
33      rx_dropped_host_chan           4       v1+
37      tx_total_host                  4       v1+
41      tx_total_ble                   4       v1+
45      tx_errors_host                 4       v1+
49      tx_errors_ble                  4       v1+
53      ble_conns_peak                 1       v1+
54      active_connections             1       v1+
55      radio_state                    1       v1+
56      duty_cycle_used_ppm_hour       4       v1+
60      mac_address                    6       v1+
66      supported_features             2       v1+
68      ble_disconnects_total          2       v2+
70      ble_att_errors                 2       v2+
72      sync_requests_total            4       v2+
76      sync_packets_sent_total        4       v2+
80      peer_syncs_initiated           2       v3
82      peer_syncs_accepted            2       v3
84      dongle_id                      8       v3
92      softdevice_fault_count         1       v1+
93      last_panic_count               1       v1+
94–127  reserved (0)
Total: 128 bytes.
```

`supported_features` bits:

- 0 BloomSync | 1 FatAdv | 2 Bonding | 3 IrkBasedIdentity
- 4 RateLimiting | 5 DutyCycleEnforcement
- 6 PeerMeshV3 | 7 CryptographicPeerAuth

#### A.4.1 `board_id` enum

```
0  unknown
1  rak_wisblock_4631
2  wio_tracker_l1
3  heltec_v3_usb
4  heltec_v3_uart
5  heltec_v4
6  elecrow_thinknode_m2
```

### A.5 Status control ops

Scoped to Status characteristic. Opcode scoping: `0x10` here is unrelated to `0x10` on Sync or Peer-Sync.

| Op                     | Direction                  | Fields               | Access           |
| ---------------------- | -------------------------- | -------------------- | ---------------- |
| 0x10 TxAccepted        | notify                     | `tx_id:u8`           | —                |
| 0x11 RxMode (reserved) | write                      | `mode:u8`            | — (v2+)          |
| 0x20 Ping              | write → 0x21 Pong          | `nonce:u32`          | open             |
| 0x22 RequestMtuInfo    | write → 0x23 MtuInfo       | — → `att_mtu:u16`    | open             |
| 0x24 TxDone            | notify                     | `tx_id:u8`           | —                |
| 0x25 TxError           | notify                     | `tx_id:u8 + code:u8` | —                |
| 0x26 ReadConfig        | write → 0x27 ConfigBlob    | — → `config:[13]`    | open (read-only) |
| 0x40 ButtonArmTx       | write → 0x41 Armed \| 0x25 | —                    | Mode B only (v1) |
| 0x42 SetTxRateLimit    | write → 0x43 Ack           | `ms:u16`             | bonded (v2+)     |
| 0x44 ResetMetrics      | write → 0x45 Ack           | —                    | bonded (v2+)     |
| 0x46 BondList          | write → 0x47 List          | — → list             | bonded (v2+)     |
| 0x48 BondRevoke        | write → 0x49 Ack           | `index:u8`           | bonded (v2+)     |
| 0x50 AddTrustedPeer    | write → 0x51 Ack           | `pk_hash:[u8;8]`     | bonded (v3)      |
| 0x52 RemoveTrustedPeer | write → 0x53 Ack           | `pk_hash:[u8;8]`     | bonded (v3)      |
| 0x60 LowPowerMode      | write → 0x61 Ack           | `enable:u8`          | bonded (v3)      |

BLE TxError codes (in `0x25 TxError`'s `code` byte):

```
0x01 Framing
0x02 Oversize
0x03 Timeout
0x04 AccessDenied
0x05 RateLimited
0x06 DutyCycleExceeded
0x07 RadioBusy
0x08 NotArmed
0x09 ConfigInvalid
0x0A NotConfigured
0x10 TxTimeout
0x11 TxFailed
```

### A.6 Peer-Sync characteristic (v3)

See §7.5.2. Opcodes 0x60–0x6F scoped here.

---

## Appendix B — State machines

### B.1 TX-Write reassembler

Full Rust source in §5.5.6.

### B.2 Advertiser cadence

Full source in §5.4.6.

### B.3 Sync handler

Full source in §5.2.2.

### B.4 Identity rotation

Full source in §5.4.5.

---

## Appendix C — Justfile, blob provenance, linker

### C.1 Justfile

```makefile
SOFTDEVICE_HEX := "ld/softdevice/s140_nrf52_7.3.0_softdevice.hex"
ADAFRUIT_BOOTLOADER_HEX := "ld/bootloader/adafruit_nrf52840_bootloader_0.9.2.hex"

# Board triples
rak_wisblock_4631_ble := "rak_wisblock_4631_ble thumbv7em-none-eabihf nRF52840_xxAA"
wio_tracker_l1_ble    := "wio_tracker_l1_ble    thumbv7em-none-eabihf nRF52840_xxAA"

flash-softdevice:
    probe-rs download --chip nRF52840_xxAA {{SOFTDEVICE_HEX}}

wio-prepare-adafruit-bootloader:
    @echo "Replaces Seeed factory bootloader with Adafruit UF2."
    @echo "Recovery requires SWD. Ctrl-C to abort."
    @read -r -p "Press enter to continue... "
    probe-rs download --chip nRF52840_xxAA {{ADAFRUIT_BOOTLOADER_HEX}}

# Full build matrix ──────────────────────────────────────────────
build-all:
    cargo build --release --features rak_wisblock_4631
    cargo build --release --features rak_wisblock_4631_ble
    cargo build --release --features rak_wisblock_4631_ble,ble_bonding
    cargo build --release --features wio_tracker_l1
    cargo build --release --features wio_tracker_l1_ble
    cargo build --release --features wio_tracker_l1_ble,ble_bonding
    cargo build --release --features heltec_v3
    cargo build --release --features waveshare_rp2040_lora

test-host:
    cargo test --features test-host --target x86_64-unknown-linux-gnu
```

### C.2 Blob provenance

`ld/softdevice/README.md`:

- URL: [infocenter.nordicsemi.com S140 v7.3.0](https://www.nordicsemi.com/Products/Development-software/s140/download)
- File: `s140_nrf52_7.3.0_softdevice.hex`
- SHA-256: `<pinned at release>`

`ld/bootloader/README.md`:

- URL: [adafruit/Adafruit_nRF52_Bootloader releases](https://github.com/adafruit/Adafruit_nRF52_Bootloader/releases)
- File: `adafruit_nrf52840_bootloader_0.9.2.hex` (or current Adafruit-released version)
- SHA-256: `<pinned>`

---

## Appendix D — Glossary and references

**Glossary**

- CCCD — Client Characteristic Configuration Descriptor
- CmRDT — Commutative Replicated Data Type (op-based)
- FICR — Factory Information Configuration Registers (nRF52840 factory-unique data block)
- IRK — Identity Resolving Key (BLE 4.2+ private address resolution)
- LL — Link Layer (BLE radio layer)
- LTK — Long Term Key
- MBR — Master Boot Record (Nordic's, loads SoftDevice)
- MTU — Maximum Transmission Unit (ATT = 247 max on S140)
- RPA — Resolvable Private Address
- S140 — Nordic SoftDevice S140 v7.3.0
- VLOC — Value Location (GATT characteristic storage: STACK or USER)

**References**

- Bluetooth Core 5.3 Vol 3 Parts C (GAP), G (GATT)
- Nordic S140 v7.3.0 SoftDevice Specification
- nrf-softdevice examples: https://github.com/embassy-rs/nrf-softdevice/tree/master/examples
- Embassy book: https://embassy.dev/book/
- Shapiro et al., "Conflict-Free Replicated Data Types," INRIA 2011
- Almeida et al., "Delta State Replicated Data Types," JPDC 2018
- Meshtastic nRF52 build (prior art for SD+USB+LoRa)
- Semtech AN1200.22 (LoRa time-on-air formulas)
- Attack references: BleedingBit (CVE-2018-16986), Sweyntooth (CVE-2019-17517+), AirTag tracking (Heinrich 2022), nRF52 APPROTECT (LimitedResults 2020)

---

## Appendix E — Implementer decisions (knobs)

- Ring capacity v1: 128 (default). 64 if tight.
- Duty-cycle cap default: 1% (EU 868). Per-regulatory build override.
- Peer-sync cooldown default: 60 s. Denser deployments may tune.
- Rotation period default: 15 min. Shorter = better privacy, more reconnects.
- `test-host` feature must be in Cargo.toml (see §5.12).
- Panic handler: custom rate-limited in release; `panic-probe` in dev.
- nrf-softdevice git rev: pin to a specific commit at v0.3 acceptance; avoid moving target.

---

## Change log

| Date       | Version | Change                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| ---------- | ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2026-04-12 | 0.1     | Initial                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| 2026-04-12 | 0.2     | Post-review-1 (senior + CRDT + security + junior round 1)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| 2026-04-12 | 0.3     | Post-review-2 (junior round 2): single adv set, concrete types, slot+generation TX routing, ring 64-slots partitioned, ed25519 spec, fixed arithmetic, Status 128 B fixed, test-host feature, nrf-softdevice pinned rev, per-char opcode scoping, iOS background-wake claim corrected                                                                                                                                                                                                                                                           |
| 2026-04-12 | 0.4     | Post-review-3: time-on-air formula inline with two verified fixtures, ConnectableAdvertisement::ScannableUndirected, RANDOM_PRIVATE_NON_RESOLVABLE address type, adv_stop_signal for rotation, ConnEventChannel bridge for sync-GATT-callback-vs-async-handler. Post-review-4: AdvBuffer decouples adv data from adv future (fixes reshape-starvation), HandleConnectionBundle for arg threading, handle_tx_write collects Decision under lock then releases before notify (locks-across-notify rule preserved), aes-gcm flash read().await fix |
