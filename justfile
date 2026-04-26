# Run every recipe with mise's tool versions active — so `just` invocations
# of cargo, espflash, rust-objcopy, etc. always resolve to the pinned
# versions in mise.toml, even if the user has a different version elsewhere
# on PATH (e.g. a stray `cargo install espflash`).
set shell := ["mise", "exec", "--", "sh", "-c"]

builds_dir := "builds"
version := `sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml`

# All known boards. ADD A NEW BOARD HERE and in the `_info` case below.
#
# Board metadata ("feature target chip") lives in the `_info` recipe —
# Xtensa targets are built with `+esp` + `-Zbuild-std=core,alloc` (the
# self-contained esp toolchain ships its own rust-src; the public
# nightly drifts ahead and breaks core-stdlib compilation).
boards := "heltec_v3 heltec_v3_uart heltec_v4 elecrow_thinknode_m2 lilygo_tbeam lilygo_tbeam_supreme rak_wisblock_4631 waveshare_rp2040_lora wio_tracker_l1"

# Install all required tools and toolchains
setup:
    mise trust --yes
    mise install
    @# ESP Xtensa toolchain (not a rustup component — installed out-of-band by espup)
    @if ! rustup toolchain list | grep -q "^esp"; then \
        echo "ESP toolchain not found, installing via espup (this may take a while)..."; \
        espup install || { echo "error: espup install failed" >&2; exit 1; }; \
    fi

# Build release firmware for all boards, installing toolchains as needed
build-all:
    @trap 'exit 130' INT; \
    for board in {{boards}}; do \
        if just _can_build $board; then \
            echo "── building $board ──"; \
            just build $board || exit $?; \
        else \
            echo "── skipping $board (toolchain install failed) ──"; \
        fi; \
    done

# Check all boards compile, installing toolchains as needed
check-all:
    @trap 'exit 130' INT; \
    for board in {{boards}}; do \
        if just _can_build $board; then \
            echo "── checking $board ──"; \
            just check $board || exit $?; \
        else \
            echo "── skipping $board (toolchain install failed) ──"; \
        fi; \
    done

# Check a single board compiles (clippy with warnings as errors)
check board:
    @just _cargo {{board}} "clippy --release" "-- -D warnings"

# Build release firmware and copy to builds/ with a readable name.
# `extra_features` (comma-separated) is appended to the board feature,
# e.g. `just build wio_tracker_l1 release debug-checkpoint`.
build board profile="release" extra_features="":
    @just _cargo {{board}} "build --{{profile}}" "" "{{extra_features}}"
    @just _copy_firmware {{board}} {{profile}}

# Build and flash a board (espflash for Xtensa, UF2 DFU for ARM).
# `extra_features` is passed through to the build, e.g.
# `just flash wio_tracker_l1 debug-checkpoint`.
flash board extra_features="":
    @just _ensure_tools
    @just build {{board}} release "{{extra_features}}"
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    case "$target" in \
        xtensa-*) \
            port=$(just _find_port "303a:1001" "1209:5741" "10c4:ea60" "1a86:7522" "1a86:7523"); \
            if [ -n "$port" ]; then \
                echo "Using $port"; \
                espflash flash -p "$port" "{{builds_dir}}/donglora-{{board}}-v{{version}}.elf"; \
            else \
                echo "No port found, falling back to espflash auto-detection..." >&2; \
                espflash flash "{{builds_dir}}/donglora-{{board}}-v{{version}}.elf"; \
            fi ;; \
        *) just _flash_uf2 {{board}} ;; \
    esac

# Flash ARM board via debug probe (requires J-Link or similar).
# `extra_features` is passed through to the build.
# Sets DEFMT_LOG at build time so defmt::info!/warn!/error! are actually
# compiled in (default filter is ERROR-only — strips info logs).
# `embassy_usb=error` silences the benign Linux modem-manager
# `SET_INTERFACE: alt setting out of range` WARN that fires once per
# enumeration; it's a normal STALL response (correct USB behavior),
# the WARN is just embassy_usb noise. Other crates inherit the `info`
# default so chart traces, our own info!/warn!/error! all still fire.
flash-probe board extra_features="":
    @just _ensure_tools
    @DEFMT_LOG=info,embassy_usb=error just build {{board}} release "{{extra_features}}"
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    probe-rs run --chip $chip "{{builds_dir}}/donglora-{{board}}-v{{version}}.elf"

# UF2-flash via the bootloader, then probe-rs attach for RTT log streaming.
# Required on boards (e.g. RAK4631) where the bootloader gates the app
# behind a "valid app flag" in its settings page that only the bootloader
# itself can set — `probe-rs run`'s direct SWD writes bypass that flag and
# leave the bootloader stuck in DFU mode. UF2 path goes through the
# bootloader, sets the flag, then we attach for RTT (no reset, no reflash).
flash-attach board extra_features="":
    @just _ensure_tools
    @DEFMT_LOG=info,embassy_usb=error just build {{board}} release "{{extra_features}}"
    @just _flash_uf2 {{board}}
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    sleep 1; \
    probe-rs attach --chip $chip --no-catch-reset \
      "{{builds_dir}}/donglora-{{board}}-v{{version}}.elf"

# Attach probe-rs for live RTT/defmt streaming WITHOUT flashing or
# resetting the chip. Logs are tee'd to a UTC-timestamped file in /tmp
# so soak captures don't clobber each other across runs. Use this with
# the dongle already running the desired firmware (flash via `just flash`
# or `just flash-attach` first).
attach board:
    @just _ensure_tools
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    log="/tmp/donglora-soak-$(date -u +%Y%m%dT%H%M%SZ).log"; \
    echo "Logging to $log"; \
    probe-rs attach --chip $chip --no-catch-reset \
      "{{builds_dir}}/donglora-{{board}}-v{{version}}.elf" 2>&1 \
      | tee "$log"

# Show binary size for a release build
size board:
    @just _cargo {{board}} "size --release"

# Run host-side protocol unit tests
test:
    DONGLORA_HOST_TEST=1 cargo test

# Check for outdated dependencies
outdated:
    cargo outdated

# ── Private helpers ───────────────────────────────────────────────────

# Install mise-managed tools if any are missing (fast no-op when current)
[private]
_ensure_tools:
    @mise trust --yes . 2>/dev/null; mise install --quiet

# Run a cargo command for a board, optionally with extra Cargo features
# (comma-separated, e.g. "debug-checkpoint").
# Xtensa: esp toolchain (cargo + rustc + rust-src together) + -Zbuild-std=core
[private]
_cargo board cmd extra_args="" extra_features="":
    @just _ensure_tools
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    [ -n "{{extra_features}}" ] && feat="$feat,{{extra_features}}"; \
    extra=""; buildstd=""; \
    case "$target" in xtensa-*) \
        just _require_esp_toolchain; \
        [ -f "$HOME/export-esp.sh" ] && . "$HOME/export-esp.sh"; \
        extra="+esp"; buildstd="-Zbuild-std=core,alloc";; \
    esac; \
    cargo $extra {{cmd}} --target "$target" --features "$feat" $buildstd {{extra_args}}

# Ensure a board's toolchain is available, auto-installing if needed
[private]
_can_build board:
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    case "$target" in \
        xtensa-*) just _require_esp_toolchain ;; \
        *) rustup target list --installed | grep -q "^$target$" || rustup target add "$target" ;; \
    esac

# Ensure the ESP Xtensa toolchain is installed (espup installed via mise).
# The esp toolchain ships with its own cargo, rustc, and rust-src — no extra
# components are needed on top.
[private]
_require_esp_toolchain:
    @just _ensure_tools
    @if ! rustup toolchain list | grep -q "^esp"; then \
        echo "ESP toolchain not found, installing via espup (this may take a while)..." >&2; \
        espup install || { echo "error: espup install failed" >&2; exit 1; }; \
    fi

# Find a serial port matching any of the given VID:PID pairs (checked in order)
[private]
_find_port +vid_pids:
    @for vidpid in {{vid_pids}}; do \
        vid="${vidpid%%:*}"; pid="${vidpid##*:}"; \
        for dev in /dev/ttyACM* /dev/ttyUSB*; do \
            [ -e "$dev" ] || continue; \
            info=$(udevadm info --query=property --name="$dev" 2>/dev/null) || continue; \
            dev_vid=$(echo "$info" | sed -n 's/^ID_VENDOR_ID=//p' | tr '[:upper:]' '[:lower:]'); \
            dev_pid=$(echo "$info" | sed -n 's/^ID_MODEL_ID=//p' | tr '[:upper:]' '[:lower:]'); \
            if [ "$dev_vid" = "$vid" ] && [ "$dev_pid" = "$pid" ]; then \
                echo "$dev"; exit 0; \
            fi; \
        done; \
    done

[private]
_copy_firmware board profile:
    @mkdir -p {{builds_dir}}
    @set -- $(just _info {{board}}); feat=$1; target=$2; chip=$3; \
    src="target/$target/{{profile}}/donglora"; \
    dst="{{builds_dir}}/donglora-{{board}}-v{{version}}"; \
    if [ -f "$src" ]; then \
        cp "$src" "$dst.elf"; echo "→ $dst.elf"; \
        case "$target" in \
            xtensa-*) \
                espflash save-image --chip "$chip" --merge --skip-padding "$src" "$dst.bin"; \
                echo "→ $dst.bin"; \
                ;; \
            thumbv6m-*) \
                rust-objcopy -O ihex "$src" "$dst.hex"; \
                cargo-hex-to-uf2 hex-to-uf2 --input-path "$dst.hex" --output-path "$dst.uf2" --family rp2040; \
                rm "$dst.hex"; \
                echo "→ $dst.uf2"; \
                ;; \
            thumbv7em-*) \
                rust-objcopy -O ihex "$src" "$dst.hex"; \
                cargo-hex-to-uf2 hex-to-uf2 --input-path "$dst.hex" --output-path "$dst.uf2" --family nrf52840; \
                rm "$dst.hex"; \
                echo "→ $dst.uf2"; \
                ;; \
        esac; \
    fi

# Copy UF2 to a mounted UF2 bootloader drive
[private]
_flash_uf2 board:
    @uf2="{{builds_dir}}/donglora-{{board}}-v{{version}}.uf2"; \
    if [ ! -f "$uf2" ]; then \
        echo "error: $uf2 not found — run 'just build {{board}}' first" >&2; exit 1; \
    fi; \
    mnt="$(find /run/media -maxdepth 3 -name INFO_UF2.TXT -printf '%h\n' -quit 2>/dev/null)"; \
    if [ -z "$mnt" ]; then \
        echo "error: no UF2 drive found — double-tap reset to enter bootloader" >&2; exit 1; \
    fi; \
    echo "Copying to $mnt ..."; \
    cp "$uf2" "$mnt/"; \
    sync; \
    echo "Done — board will reboot automatically"

# Single source of truth for per-board metadata: "feature target chip".
# ADD A NEW BOARD HERE and in the `boards` list near the top of this file.
[private]
_info name:
    @case "{{name}}" in \
        heltec_v3)             echo "heltec_v3 xtensa-esp32s3-none-elf esp32s3" ;; \
        heltec_v3_uart)        echo "heltec_v3_uart xtensa-esp32s3-none-elf esp32s3" ;; \
        heltec_v4)             echo "heltec_v4 xtensa-esp32s3-none-elf esp32s3" ;; \
        elecrow_thinknode_m2)  echo "elecrow_thinknode_m2 xtensa-esp32s3-none-elf esp32s3" ;; \
        lilygo_tbeam)          echo "lilygo_tbeam xtensa-esp32-none-elf esp32" ;; \
        lilygo_tbeam_supreme)  echo "lilygo_tbeam_supreme xtensa-esp32s3-none-elf esp32s3" ;; \
        rak_wisblock_4631)     echo "rak_wisblock_4631 thumbv7em-none-eabihf nRF52840_xxAA" ;; \
        waveshare_rp2040_lora) echo "waveshare_rp2040_lora thumbv6m-none-eabi rp2040" ;; \
        wio_tracker_l1)        echo "wio_tracker_l1 thumbv7em-none-eabihf nRF52840_xxAA" ;; \
        *) echo "Unknown board: {{name}}" >&2; exit 1 ;; \
    esac
