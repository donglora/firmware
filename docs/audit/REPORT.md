# Security Audit Report — donglora firmware v1.5.2

Auditor: **Mythos** (adversarial security review per `MYTHOS.md`)
Repo: `/home/swaits/Code/donglora-org/firmware`, HEAD `688938d` (chore: release 1.5.2)
Phases: Recon (`01-recon.md`) → Threats (`02-threats.md`) → Targeted analysis (`03-findings-draft.md`) → Verification → this report.

---

## Executive summary

**3 findings total: 0 Critical, 0 High, 0 Medium, 2 Low, 1 Info.**

The firmware is a small, tightly-scoped, no_std embedded Rust binary that exposes a SX126x/SX127x radio over a single host transport (USB CDC-ACM or UART). It performs **no networking, no filesystem I/O, no shell exec, no template rendering, no SQL/NoSQL/XML, no cryptography, and no deserialization beyond explicit byte slicing**. The whole vulnerability landscape MYTHOS targets is out of scope by construction; the realistic attack surface is exactly two paths: (1) inbound frames on the host transport, and (2) inbound LoRa packets on the configured RF parameters.

Both paths were traced end-to-end:

- **Host-transport ingress.** Every reachable byte goes through a CRC-checked, COBS-framed `FrameDecoder` → strict typed `Command::parse` → numeric-bounded `validate_lora`. Reserved bits, unknown enum values, length mismatches, and tag=0 are all explicitly rejected and turned into `ERR(...)` responses. No reachable panic, no buffer overrun, no integer overflow, no OOB access was found.
- **OTA RX ingress.** Payload bytes are bounded to 255 (SX126x register width); copied into a `heapless::Vec<u8, 255>` and forwarded to the host verbatim. The firmware never parses OTA bytes — there is no interpretation surface for a remote attacker to exploit.

The remaining findings concern **disclosure** (MAC visible in the USB serial descriptor — by design but worth documenting), **supply-chain hardening** (git dep not commit-pinned), and **reliability on one board variant** (esp-println output collides with host UART on lilygo_tbeam classic only). All are Low or Info severity.

**Top 3 systemic observations (NOT findings — design choices flagged for transparency):**

1. **No authentication on host transport.** Anyone with kernel-level access to the CDC-ACM/UART device node controls the radio. The product is intentionally a "transparent dongle" — physical attachment is the auth boundary.
2. **No RF-regulatory enforcement.** Frequency, sync word, and TX power are whatever the host commands, within hardware capability. FCC / CE / ETSI compliance is the user's responsibility.
3. **`TX skip_cad=1` bypasses listen-before-talk on demand.** Documented in `PROTOCOL.md`; available to any host-side actor.

These are deliberate features of a transparent LoRa bridge; treating them as bugs would be a category error. They are surfaced here so a security reader knows the trust model.

**Recommended remediation priorities:** None are urgent. The two Low findings can be addressed when convenient as hardening work; the Info finding is a documentation matter.

---

## Findings

### [LOW] lora-phy git dependency uses mutable branch ref, not a pinned commit

- **File(s):** `Cargo.toml:35`
- **Category:** Supply chain
- **Status:** Confirmed
- **Severity:** Low (CVSS-ish 4.4 — depends on a future `cargo update` and a maintainer-account compromise; reproducible builds are not affected today)
- **CWE:** CWE-829 (Inclusion of functionality from untrusted control sphere)

#### Summary

`lora-phy` is sourced from a git fork via `branch = "fixed"` rather than `rev = "<sha>"`. `Cargo.lock` currently pins the specific commit `043d7659…` (confirmed in `Cargo.lock:632-640`), so builds in this exact tree are reproducible. However, any future `cargo update -p lora-phy` (or running on a developer machine without a committed lockfile, which the firmware repo _does_ commit) will silently advance to whatever sits on `branch=fixed` at that moment. If the `swaits/lora-rs` GitHub account were compromised, the next `cargo update` ingests the malicious commit and the next-built firmware ships it.

#### Data flow

1. `Cargo.toml:35` declares `lora-phy = { git = "https://github.com/swaits/lora-rs.git", branch = "fixed", features = ["defmt-03"] }`.
2. `Cargo.lock:634` records the current resolved commit hash.
3. Any contributor running `cargo update` (or any auto-update tooling on the maintainer's machine) replaces the lockfile entry with whatever is at HEAD of the branch at update time.
4. The resulting binary executes whatever the new HEAD commit contains, with no human-readable diff at the dependency boundary unless someone inspects the lockfile delta.

#### Proof of concept

Reproducibility today is fine — `cargo build` against the committed lockfile uses commit `043d7659…`. The exposure is **temporal**: the moment `cargo update -p lora-phy` runs, the lockfile takes whatever commit is then on the branch. There is no PoC to demonstrate beyond reading the manifest.

#### Impact

A maintainer-account compromise (GitHub credential theft, PAT leak, repo-write malware) lets the attacker push a backdoored commit to the `fixed` branch. The next build that runs `cargo update` ships the backdoor into every flashed dongle. Severity is bounded by (a) the rate at which `cargo update` is run and (b) the difficulty of compromising the maintainer's GitHub. It's not zero — pinning to a commit closes the window.

#### Fix

Switch the manifest from `branch = "..."` to `rev = "<commit_sha>"`:

```toml
lora-phy = { git = "https://github.com/swaits/lora-rs.git", rev = "043d7659dfe2148913f16b2f7ea3e9241642b770", features = ["defmt-03"] }
```

Re-pin (bump the `rev`) on each lora-phy upgrade. Drops the temporal exposure entirely; the dep changes only when a human edits the manifest. `TODO.md` already plans to drop the fork once upstream PRs merge — at that point switch back to a crates.io version, which gets the same effect from the registry's immutability.

---

### [LOW] On `lilygo_tbeam` (ESP32 classic) only, esp-println output collides with host UART

- **File(s):** `src/board/lilygo_tbeam.rs:112-120`, `Cargo.toml:73-77`, `src/main.rs:53-57`
- **Category:** Wire-level reliability / information disclosure
- **Status:** Confirmed (frame corruption when error-level logs fire); not reachable from wire input today
- **Severity:** Low (reliability with disclosure side-effects; not directly attacker-triggerable)
- **CWE:** CWE-209 (Information Exposure Through an Error Message) — secondary

#### Summary

For the `lilygo_tbeam` build target (ESP32 classic, SX1276, CP2102-bridged UART), the host transport is `UART0` on `GPIO1/GPIO3`. `esp-println 0.17` is enabled with the `auto` (default) + `defmt-espflash` features. On ESP32 classic there is no USB-Serial-JTAG peripheral, so `auto` falls back to `UART0` — the _same_ UART the host protocol uses. Any defmt output (in production builds, only `error!` survives the compile-time filter) writes bytes onto the host wire mid-frame, corrupting the binary protocol the host is reading.

For `heltec_v3_uart` and `elecrow_thinknode_m2` (ESP32-S3, which _does_ have USB-Serial-JTAG), `esp-println auto` selects USB-Serial-JTAG — a _different_ peripheral. Those boards are unaffected.

#### Data flow

1. `Cargo.toml:73-77` pulls `esp-println` with `defmt-espflash` (no `uart` / `jtag-serial` feature explicitly set → `auto` picks at runtime).
2. `src/main.rs:53-57` installs `esp_println` as defmt's global handler for the esp boards.
3. `src/board/lilygo_tbeam.rs:113-120` initializes `Uart::new(p.UART0, ...).with_tx(p.GPIO1).with_rx(p.GPIO3)` as the host transport.
4. The ESP32 classic has no `USB_SERIAL_JTAG` peripheral → esp-println's runtime auto-selector falls back to UART0.
5. When a `defmt::error!(...)` or `defmt::panic!(...)` line executes on this board, esp-println writes bytes to UART0 → host sees them interleaved with COBS-framed protocol bytes.

#### Reachability from wire input

Walked the `error!` and `panic!` callsites: all fire only on hardware faults (`Recovering: exhausted ... transitioning to Dead`, `radio chart entered Dead state`, `lora cold init failed`, chart-invariant panics that are themselves not wire-reachable). **None are reachable from a malformed command.** Treat as a reliability concern, not a wire-attack.

#### Proof of concept

Run firmware compiled for `lilygo_tbeam`, intentionally induce a hardware-side error (e.g., disconnect the SX1276 SPI bus while the host is sending commands). Observe `defmt error!` output corrupting the COBS frames on the same UART. Without hardware tampering there is no PoC.

#### Impact

If the chip ever wedges hard enough to drive the chart into `Recovering` exhaustion or `Dead`, the host suddenly sees frame-CRC errors caused by defmt bytes instead of the chip-failure response it should have received. The host's framing layer recovers at the next clean `0x00` sentinel, but the lost frames could be the very `ERadio` reply that would have signaled the underlying failure. **Diagnostic blindness during hardware failure** on this one board variant.

#### Fix

Pick one of these for the lilygo_tbeam build:

- **Option A (preferred):** add `"no-op"` to the `esp-println` feature set for the `lilygo_tbeam` board feature, so the defmt global handler discards output entirely. Trade-off: no `probe-rs` defmt log capability on this board. Acceptable since the board has no SWD path anyway — it's CP2102-only.
- **Option B:** explicitly route esp-println to a _different_ UART (e.g., `UART1`) by adding the `uart` feature with manual pin selection. Requires reading the esp-println 0.17 docs to confirm the pin override hook.

This is also the spirit of the existing `debug-checkpoint` feature for UART-only boards (the LED-blink workaround), per `Cargo.toml:139-144`. The same "no shared UART for protocol + logs" rule should apply.

---

### [INFO] MAC address disclosed in USB serial descriptor and `GET_INFO` without authentication

- **File(s):** `src/host/usb.rs:182-189`, `src/main.rs:159-161, 180-181`
- **Category:** Information disclosure / privacy
- **Status:** Confirmed
- **Severity:** Info (intentional disclosure of a stable device identifier; standard for USB devices but worth documenting)
- **CWE:** CWE-200 (Exposure of Sensitive Information)

#### Summary

The device's 6-byte hardware MAC (nRF52840 FICR.DEVICEADDR / ESP32 eFuse / RP2040 flash-UID-derived) appears in two places, both readable without any authentication:

1. The USB CDC-ACM serial-number string descriptor (12 chars uppercase hex, no separator).
2. `GET_INFO.mcu_uid` (first 6 bytes; remaining 26 are zero).

Both surfaces are present at every boot. USB descriptors are read by every OS during enumeration without user consent; `GET_INFO` is wire-reachable by anyone who can write to the CDC-ACM/UART device.

#### Data flow (USB descriptor)

1. `src/main.rs:99-109` — boot reads `parts.mac` via `board::Board::mac_address()`.
2. `src/main.rs:118-124` — MAC passed to `host_task`.
3. `src/host/usb.rs:185-189` — MAC bytes formatted as 12 ASCII hex chars; assigned to `config.serial_number`.
4. Embassy USB stack publishes it as USB string descriptor (the OS retrieves on enumeration).

#### Data flow (GET_INFO)

1. `src/main.rs:159-161` — `mcu_uid[..6].copy_from_slice(&mac); Info.mcu_uid_len = 6`.
2. Any host issuing a valid `PING`-tag-2-format `GET_INFO` frame gets the `Info` blob back, including the 6-byte MAC.

#### Impact

The MAC is a permanent, factory-burned device identifier. A user moving a dongle between hosts/networks (laptop ↔ desk; home ↔ office; conference Wi-Fi) is correlatable across contexts by any malicious userspace program that enumerates USB. The information is _useful_ — `donglora-mux` and friends rely on it to distinguish multiple dongles — so it cannot simply be removed.

**This is also the standard model for USB devices in general.** Every USB device with a serial-number descriptor exposes a stable identifier; the user gets no consent prompt at enumeration time. Treat this finding as documentation rather than a remediation request.

#### Fix (optional, only if you care about cross-host de-anonymization)

- Replace the MAC in the USB serial descriptor with a per-install random byte string generated once and stored in MCU non-volatile memory (if available). Keep the real MAC accessible via authenticated `GET_INFO` if/when authentication is added.
- Or: leave the descriptor MAC alone (it's the standard pattern) and document the privacy expectation in `README.md` so users understand the dongle is trackable.

The Info-severity rating is the right call: this is a transparency / documentation question, not an actionable bug.

---

## Appendix A — Out-of-scope items

The audit did not cover:

- **Host-side applications** that consume the protocol (`ai-bot`, `bridge`, `client-py`, `mux-rs`, `mux-py`, `client-rs`, `tools`). Each is a separate audit target.
- **The `donglora-protocol` crate's invariants** beyond how the firmware uses them. The sibling repo at `/home/swaits/Code/donglora-org/protocol/` already maintains a fuzz suite (`protocol/fuzz/`) and mutation tests (`protocol/mutants.out*`); those validate the codec independent of this audit.
- **Physical-layer / RF-regulatory compliance** (FCC / CE / ETSI). The firmware is _intentionally_ transparent; user is responsible.
- **Side-channel analysis** (timing, power, electromagnetic) — irrelevant on a device with no secrets.
- **Build-time compromise** (malicious `build.rs`, compromised toolchain, supply-chain attacks against rustc / espup / cargo). The Low finding above is the closest the audit gets to this surface.
- **Bootloader / DFU surface** (UF2 / Adafruit nRF52 bootloader / espflash). Those are upstream code paths; this audit covers the application image only.
- **Hardware tampering / cold-boot RAM scraping / glitching the MCU**. Out of scope for source-code audit.

## Appendix B — Negative findings (audited and concluded not vulnerable)

Each item below was checked at source level and confirmed safe. This list is the most useful part of the audit: it documents what was _examined_ and why a Critical / High finding does not exist.

- **Frame decoder buffer handling** (`donglora-protocol-1.2.0/src/frame.rs::FrameDecoder::feed`): fixed-size buf, bounded index, overflow → resync. No OOB. No panic. No unbounded growth.
- **CRC-16/CCITT-FALSE on every frame**: cryptographically weak (it's a CRC, not a MAC) but appropriate for transport integrity. Defends against the firmware acting on bit-flipped frames.
- **`Command::parse` for every type_id** (`donglora-protocol-1.2.0/src/commands.rs:133-181`): reserved bits, unknown enum values, length mismatches, empty payloads all rejected explicitly with typed errors that the firmware turns into `ERR(ELENGTH|EPARAM|EMODULATION|EUNKNOWN_CMD)`. No silent acceptance.
- **`Modulation::decode` and `LoRaConfig::decode`** (`donglora-protocol-1.2.0/src/modulation.rs:743-755, 224-263`): every byte either bound-checked by `from_u8` or `match` arms reject non-{0,1} booleans. `WrongLength` returns on size mismatch.
- **`validate_lora`** (`radio.rs:2125-2145`): freq, SF, BW, power, preamble bounds. The two unchecked fields (`preamble_len` upper bound and `sync_word`) were traced into downstream `toa_us` (u64 math with `u32::MAX` clamp — confirmed overflow-safe) and `set_lora_sync_word` (per memory-note, silent acceptance is intentional).
- **`try_start_rx_hw` → `prepare_for_rx` chain** (`radio.rs:1348-1438`): `MAX_OTA_PAYLOAD as u8 = 255` is hardware-bounded (SX126x payload-length register is one byte). No truncation today; would surface at compile-time if `MAX_OTA_PAYLOAD` ever changed.
- **`collect_rx_packet`** (`radio.rs:1664-1702`): `rx_buf: [u8; MAX_OTA_PAYLOAD]` matches the chip-level cap. `copy_len = len.min(MAX_OTA_PAYLOAD)`. `extend_from_slice` is bounds-safe.
- **Panic reachability from wire input**: 6 panic sites enumerated; all reach only via chart-shape violations, not from network input. Specifically, the `NonZeroU16::new(tag).expect(...)` at `radio.rs:937` is gated by the `if tag == 0 { ... return; }` at `host/framing.rs:36-40` _before_ the envelope can be enqueued.
- **`unsafe` blocks** (4 sites): one-time-init `static mut` patterns (sound under stated preconditions, before any task spawns) and MMIO `read_volatile` / `write_volatile` (sound for `RESETREAS` / `FICR.DEVICEADDR`).
- **Cross-task races / TOCTOU**: `embassy_sync::channel::Channel<CriticalSectionRawMutex, ...>` for every cross-task hop. `portable_atomic::AtomicBool` with `AcqRel` swaps for shared flags. The radio chip has exactly one user (radio task), so no SPI-bus race.
- **USB-descriptor / control-transfer handling**: `embassy_usb` third-party. The firmware's `CTRL_BUF = 128` is explicitly oversized to defeat a known panic site in the 0.5/0.6 era (documented in source comment at `host/usb.rs:191-196`).
- **`embassy_usb_driver::EndpointError` handling** (`host/usb.rs:281-313`): `Disabled` is logged at debug only (normal during enumeration), `BufferOverflow` at warn — and the decoder is reset and the read loop sleeps 100 ms to avoid burning the executor on a stuck driver.
- **Stack overflow from nested parse calls**: depth-bounded (`feed → emit_decoded → Command::parse → Modulation::decode → LoRaConfig::decode`); cumulative stack use ≈ 1.5 KB on a 4–32 KB task stack.
- **DoS via async-err flood**: `RADIO_THROTTLED` gates `emit_async_err`; outbound depth-64 ring absorbs bursts; the channel can never grow.
- **DoS via Reset queue flood**: `RESET_PENDING` AtomicBool dedupes injected resets; documented at `channel.rs:16-29`.
- **Reachable `defmt::panic!()` in `radio_task` epilogue**: gated by chart non-termination invariant (every leaf has a `during:` activity); not reachable by wire input.
- **TX path payload handling**: `pending_tx.data` is `heapless::Vec<u8, MAX_OTA_PAYLOAD>` — bounded at parse-time. `lora.tx()` writes whatever bytes the host supplied. No memory-corruption surface; no auth surface to bypass.
- **OTA RX → outbound queue**: the firmware does not interpret OTA bytes. Wire encoding (`events.rs:228-249`) is a fixed-layout memcpy with bound checks at the front (`if buf.len() < total { Err }`).
- **`InboundEvent::Reset` clears the radio chart`incoming_tag` slot** (`radio.rs:923-930`) so a tag-echoing action that runs after Reset doesn't pull a stale tag.
- **No deserializers that take untrusted input by reflection** (no serde, no XML, no YAML, no pickle, no protobuf with dynamic types).
- **No SQL, NoSQL, shell, template, regex, HTTP, DNS, file, zip, XML surfaces in this firmware.**

## Appendix C — Confidence and methodology

- **Confidence:** High for the negative-findings list (every claim was traced with file:line); High for the three findings (each verified by re-reading the cited code after writing the draft).
- **Method:** MYTHOS Phase 1–5 walkthrough as defined in `MYTHOS.md`. Static review only; no runtime fuzzing was performed in this pass. The donglora-protocol crate's own fuzz suite (sibling repo) is presumed to validate codec invariants and is not duplicated here.
- **Not done:** `cargo audit` / `cargo deny` were not run as part of this audit. Recommended as a CI addition (CHANGELOG.md mentions hardening pulses; this would fit naturally). At the time of writing, no known reachable CVE for any used dependency surface is known to the auditor.

## Self-check

- [x] Every finding has file + line numbers
- [x] Every Confirmed finding has a PoC, a data-flow trace, or an unambiguous "this is what's in the source today" reference (the Low / Info ratings make a remote PoC moot for the supply-chain and disclosure findings)
- [x] Every finding has a one-sentence impact statement
- [x] No findings are pure pattern-matches without reachability analysis
- [x] Severity ratings are calibrated — nothing rated above Low; that matches what the audit actually found
- [x] Negative findings appendix exists
- [x] No suggestions beyond what's needed to fix the actual bug
