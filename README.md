# PAMI Control Firmware (CDFR 2027)

[![Build](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/actions/workflows/rust_ci.yml/badge.svg?branch=master)](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/actions/workflows/rust_ci.yml)
![Rust](https://img.shields.io/badge/Rust-no__std-orange?logo=rust)
![Target](https://img.shields.io/badge/target-ESP32--S3-blue?logo=espressif)
![esp-hal](https://img.shields.io/badge/esp--hal-1.2-lightgrey)

Low-level motion and actuation firmware for the autonomous PAMI robots participating in the **Coupe de France de Robotique 2027 (Eurobot)**.

Built with **Embedded Rust**, [`esp-hal`](https://github.com/esp-rs/esp-hal), and the **Embassy** async executor on the **ESP32-S3** microcontroller (Xtensa architecture).

---

## Documentation

Architecture, pin map, the BrainBoard SPI protocol, safety behaviour and testing are documented in the
[wiki](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/wiki).

---

## Prerequisites

1. Install Rust via [rustup](https://rustup.rs), then the Xtensa toolchain (the ESP32-S3 isn't supported by upstream Rust):
   ```bash
   cargo install espup espflash
   espup install
   ```
2. Give your user access to the serial port (`/dev/ttyACM*`), then log out and back in:
   ```bash
   sudo usermod -a -G dialout $USER   # Arch / Fedora: uucp instead of dialout
   ```
3. Export the Xtensa environment in every new terminal (or add it to your shell profile / [direnv](https://direnv.net/)):
   ```bash
   source $HOME/export-esp.sh
   ```

---

## Development Workflow

### Build

```bash
cargo check            # fast compile-check
cargo build --release
```

### Tests

Tests run on the board itself ([embedded-test](https://github.com/probe-rs/embedded-test)) and live in `tests/`. With the board on USB:
```bash
cargo install probe-rs-tools --locked   # once
cargo hil
```
`cargo hil` stops at the first failing suite, and `otos` needs the sensor plugged in. Without it, pick the suites:
```bash
cargo hil --test math --test protocol --test board
```

### Flashing to the ESP32-S3

With the board on USB, flash and open the serial monitor:
```bash
cargo run --release
```

Expected serial output on boot:
```text
INFO - Board config start
INFO - Board config done
```

To exit the serial monitor, press `Ctrl + C`.
