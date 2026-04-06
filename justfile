set shell := ["bash", "-c"]

# Board definitions: feature target chip
# Xtensa targets use nightly cargo + esp rustc + -Zbuild-std=core.
# Tool versions are pinned in mise.toml; run `just setup` to install everything.
heltec_v3 := "heltec_v3 xtensa-esp32s3-none-elf esp32s3"
heltec_v4 := "heltec_v4 xtensa-esp32s3-none-elf esp32s3"
rak_wisblock_4631  := "rak_wisblock_4631 thumbv7em-none-eabihf nRF52840_xxAA"
wio_tracker_l1     := "wio_tracker_l1 thumbv7em-none-eabihf nRF52840_xxAA"

builds_dir := "builds"

# All known boards
boards := "heltec_v3 heltec_v4 rak_wisblock_4631 wio_tracker_l1"

# Install all required tools and toolchains
setup:
    mise trust --yes
    mise install
    @# ESP Xtensa toolchain (not managed by mise)
    @if ! rustup toolchain list | grep -q "^esp"; then \
        echo "ESP toolchain not found, installing via espup (this may take a while)..."; \
        espup install || { echo "error: espup install failed" >&2; exit 1; }; \
    fi
    @# nightly rust-src for -Zbuild-std
    @if ! rustup component list --toolchain nightly --installed 2>/dev/null | grep -q "^rust-src"; then \
        rustup component add rust-src --toolchain nightly; \
    fi
    @# ARM target
    @rustup target list --installed | grep -q "^thumbv7em-none-eabihf$" || rustup target add thumbv7em-none-eabihf

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

# Check a single board compiles
check board:
    @just _cargo {{board}} check

# Run clippy on a single board
clippy board:
    @just _cargo {{board}} "clippy" "-- -D warnings"

# Build release firmware and copy to builds/ with a readable name
build board profile="release":
    @just _cargo {{board}} "build --{{profile}}"
    @just _copy_firmware {{board}} {{profile}}

# Build and flash a board (espflash for Xtensa, UF2 DFU for ARM)
flash board:
    @just _ensure_tools
    @just build {{board}} release
    @read -r feat target chip <<< "$(just _info {{board}})"; \
    case "$target" in \
        xtensa-*) \
            port=$(just _find_port "303a:1001" "1209:5741"); \
            if [ -n "$port" ]; then \
                echo "Using $port"; \
                espflash flash -p "$port" "{{builds_dir}}/donglora-{{board}}-release.elf"; \
            else \
                echo "No port found, falling back to espflash auto-detection..." >&2; \
                espflash flash "{{builds_dir}}/donglora-{{board}}-release.elf"; \
            fi ;; \
        *) just _flash_uf2 {{board}} ;; \
    esac

# Flash ARM board via debug probe (requires J-Link or similar)
flash-probe board:
    @just _ensure_tools
    @just build {{board}} release
    @read -r feat target chip <<< "$(just _info {{board}})"; \
    probe-rs run --chip $chip "{{builds_dir}}/donglora-{{board}}-release.elf"

# Show binary size for a release build
size board:
    @just _cargo {{board}} "size --release"

# Run host-side protocol unit tests
test:
    DONGLORA_HOST_TEST=1 cargo test

# Check for outdated dependencies
outdated:
    @# embassy-executor 0.10 renamed arch-* → platform-* features.
    @# esp-rtos 0.2 still pins embassy-executor ^0.9, blocking the upgrade.
    @# cargo-outdated crashes trying to resolve 0.10 with the old feature name.
    cargo outdated --exclude embassy-executor

# ── Private helpers ───────────────────────────────────────────────────

# Install mise-managed tools if any are missing (fast no-op when current)
[private]
_ensure_tools:
    @mise trust --yes . 2>/dev/null; mise install --quiet

# Run a cargo command for a board.
# Xtensa: nightly cargo + esp rustc (via RUSTC override) + -Zbuild-std=core
[private]
_cargo board cmd extra_args="":
    @just _ensure_tools
    @read -r feat target chip <<< "$(just _info {{board}})"; \
    env=""; extra=""; \
    case "$target" in xtensa-*) \
        just _require_esp_toolchain; \
        [ -f "$HOME/export-esp.sh" ] && . "$HOME/export-esp.sh"; \
        env="RUSTC=$(rustup which rustc --toolchain esp)"; extra="+nightly"; buildstd="-Zbuild-std=core,alloc";; \
    esac; \
    eval $env cargo $extra {{cmd}} --target "$target" --features "$feat" $buildstd {{extra_args}}

# Ensure a board's toolchain is available, auto-installing if needed
[private]
_can_build board:
    @read -r feat target chip <<< "$(just _info {{board}})"; \
    case "$target" in \
        xtensa-*) just _require_esp_toolchain ;; \
        *) rustup target list --installed | grep -q "^$target$" || rustup target add "$target" ;; \
    esac

# Ensure the ESP Xtensa toolchain is installed (espup installed via mise)
[private]
_require_esp_toolchain:
    @just _ensure_tools
    @if ! rustup toolchain list | grep -q "^esp"; then \
        echo "ESP toolchain not found, installing via espup (this may take a while)..." >&2; \
        espup install || { echo "error: espup install failed" >&2; exit 1; }; \
    fi; \
    if ! rustup component list --toolchain nightly --installed 2>/dev/null | grep -q "^rust-src"; then \
        echo "Installing nightly rust-src (needed for -Zbuild-std)..." >&2; \
        rustup component add rust-src --toolchain nightly || { echo "error: failed to install rust-src" >&2; exit 1; }; \
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
    @read -r feat target chip <<< "$(just _info {{board}})"; \
    src="target/$target/{{profile}}/donglora"; \
    dst="{{builds_dir}}/donglora-{{board}}-{{profile}}"; \
    if [ -f "$src" ]; then \
        cp "$src" "$dst.elf"; echo "→ $dst.elf"; \
        case "$target" in \
            thumbv7em-*) \
                mise exec -- rust-objcopy -O ihex "$src" "$dst.hex"; \
                mise exec -- cargo-hex-to-uf2 hex-to-uf2 --input-path "$dst.hex" --output-path "$dst.uf2" --family nrf52840; \
                rm "$dst.hex"; \
                echo "→ $dst.uf2"; \
                ;; \
        esac; \
    fi

# Copy UF2 to a mounted UF2 bootloader drive
[private]
_flash_uf2 board:
    @uf2="{{builds_dir}}/donglora-{{board}}-release.uf2"; \
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

[private]
_info name:
    @if [ "{{name}}" == "heltec_v3" ]; then echo "{{heltec_v3}}"; \
     elif [ "{{name}}" == "heltec_v4" ]; then echo "{{heltec_v4}}"; \
     elif [ "{{name}}" == "rak_wisblock_4631" ]; then echo "{{rak_wisblock_4631}}"; \
     elif [ "{{name}}" == "wio_tracker_l1" ]; then echo "{{wio_tracker_l1}}"; \
     else echo "Unknown board: {{name}}" >&2; exit 1; fi
