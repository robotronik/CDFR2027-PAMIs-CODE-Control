# PAMI Control Firmware (CDFR 2027)

[![Build](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/actions/workflows/rust_ci.yml/badge.svg?branch=master)](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/actions/workflows/rust_ci.yml)
![Rust](https://img.shields.io/badge/Rust-no__std-orange?logo=rust)
![Target](https://img.shields.io/badge/target-ESP32--S3-blue?logo=espressif)
![esp-hal](https://img.shields.io/badge/esp--hal-1.2-lightgrey)

Low-level motion and actuation firmware for the autonomous PAMI robots participating in the **Coupe de France de Robotique 2027 (Eurobot)**.

Built with **Embedded Rust**, [`esp-hal`](https://github.com/esp-rs/esp-hal), and the **Embassy** async executor on the **ESP32-S3** microcontroller (Xtensa architecture).

---

## Architecture & Subsystems

The firmware is organized into Embassy async tasks. Core 1 runs only `MotorControlTask`; Core 0 runs everything else. Tasks own their hardware and talk only through `embassy_sync` channels declared in `main.rs`.

```
src/
├── main.rs                   # Entrypoint: spawns tasks on both cores, declares the inter-task channels
├── lib.rs                    # Library root: exposes the modules to main.rs and tests/
├── board.rs                  # Hardware initialization, pin definitions, and peripheral mapping
├── math_utils.rs             # Pure maths helpers (angle wrapping)
├── brain_comm.rs             # BrainBoard link root module
├── brain_comm/
│   ├── communication.rs      # BrainCommTask: SPI slave link, routes requests to the owning task
│   └── protocol.rs           # Wire format encoding/decoding (pure functions, no I/O)
├── motor_control.rs          # Locomotion root module (runs on Core 1)
├── motor_control/
│   ├── motors.rs             # MotorControlTask: 3 DC motors via DRV8871 H-bridges (50Hz), owns the OTOS
│   ├── otos.rs               # Otos: Optical Tracking Odometry Sensor driver over I2C0
│   ├── motor_commands.rs     # MotionCommand inbox (BrainComm -> MotorControl)
│   └── robot_state.rs        # RobotPos watch (MotorControl -> BrainComm)
├── servo_control.rs          # Servo root module
├── servo_control/
│   ├── servos.rs             # ServoControlTask: 4 RC servo actuator channels
│   └── servo_commands.rs     # ServoCommand inbox (BrainComm -> ServoControl)
├── utility.rs                # Housekeeping root module
└── utility/
    ├── battery.rs            # BatteryTask: 2S battery voltage on GPIO1 (1 Hz), low flag to BrainComm
    ├── button.rs             # ButtonTask: Boot button monitoring on GPIO0
    └── led.rs                # LedTask: Heartbeat status LED on GPIO16
```

### Hardware Pin Mapping (ESP32-S3-WROOM-1)

| Subsystem | Signal Name | ESP32-S3 Pin | Hardware Interface |
| :--- | :--- | :--- | :--- |
| **Drivetrain** | `DC_1_IN1` / `IN2` | GPIO 4 / GPIO 5 | Motor 1 DRV8871 H-Bridge |
| | `DC_2_IN1` / `IN2` | GPIO 6 / GPIO 7 | Motor 2 DRV8871 H-Bridge |
| | `DC_3_IN1` / `IN2` | GPIO 8 / GPIO 9 | Motor 3 DRV8871 H-Bridge |
| **Odometry (OTOS)** | `SDA` / `SCL` | GPIO 17 / GPIO 18 | Connector `I2C1` (`I2C0` peripheral) |
| **Brain Link** | `CLK` / `CS` / `Handshake` | GPIO 38 / GPIO 39 / GPIO 40 | Connector `SPI1` (`SPI2` peripheral) |
| | `MISO` / `MOSI` | GPIO 41 / GPIO 42 | Connector `SPI1` (`SPI2` peripheral) |
| **Servos** | `SERVO_1` / `SERVO_2` | GPIO 47 / GPIO 48 | RC Servo PWM Outputs 1 & 2 |
| | `SERVO_3` / `SERVO_4` | GPIO 14 / GPIO 15 | RC Servo PWM Outputs 3 & 4 |
| **Power Sense** | `BATTERY_LEVEL` | GPIO 1 | `ADC1_CH0` (30kΩ/10kΩ divider, 1/4 Vbatt) |
| **User I/O** | `STATUS_LED` | GPIO 16 | Active-High LED (via 470Ω resistor) |
| | `BOOT` | GPIO 0 | Active-Low push button `SW_BOOT1` |
| **Debug / UART** | `TXD0` / `RXD0` | GPIO 43 / GPIO 44 | Connector `UART1` (`UART0` peripheral) |

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
