# Firmware TODO List

## `lora-phy` fix: drop fork pin once PRs #428 / #431 / #432 merge + release

Upstream PRs (all open):

- [#428 — *phy: add pre-command BUSY checks per Semtech
  datasheets*](https://github.com/lora-rs/lora-rs/pull/428) (sx126x):
  pre-command BUSY check + standby/IRQ-clear hygiene around CAD.
- [#431 — *phy: sx127x: clear stale IRQ flags in do_rx before mode
  change*](https://github.com/lora-rs/lora-rs/pull/431).
- [#432 — *phy: sx127x: clear stale IRQ flags before reconfiguring DIO
  mapping in set_irq_params*](https://github.com/lora-rs/lora-rs/pull/432).

Fork branch we ship from:
[`swaits/lora-rs @ fixed`](https://github.com/swaits/lora-rs/tree/fixed)
— a merge of all three fix branches kept up-to-date with upstream
`main`.

Firmware consumes the fixes via a direct `git` dependency in
`Cargo.toml`:

```toml
lora-phy = { git = "https://github.com/swaits/lora-rs.git", branch = "fixed", features = ["defmt-03"] }
```

When all three PRs merge and a `lora-phy` release ships on crates.io
that contains them, switch this line back to a normal
`lora-phy = "X.Y"` crates.io version and drop the explanatory comment
block above it.

Hardware impact: large stability gain on every board — the sx126x
BUSY-race fix lands on every nRF/RP2040 board, and the sx127x IRQ-clear
fixes unblock TBeam classic in particular. Hardware-verified on Wio
Tracker L1 (sx126x): 20/20 clean-boot TX after the fix (vs ~50%
before). LilyGo T-Beam classic (sx127x): TX confirmed working
2026-04-30 on the post-esp-hal-1.1 build.
