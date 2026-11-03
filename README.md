# PAMI Control Firmware (CDFR 2027)

Low-level motion and actuation firmware for the autonomous PAMI robots participating in the **Coupe de France de Robotique 2027 (Eurobot)**.

Built with **Embedded Rust**, [`esp-hal`](https://github.com/esp-rs/esp-hal), and the **Embassy** async executor on the **ESP32-S3** microcontroller (Xtensa architecture).

---

## Architecture & Subsystems

The firmware execution is organized into modular Embassy async tasks running concurrently:

```
src/
├── board.rs                  # Hardware initialization, pin definitions, and peripheral mapping
├── communication.rs          # BrainCommTask: High-level SPI communication with BrainBoard
├── motor_control.rs          # Locomotion root module
├── motor_control/
│   ├── motors.rs             # MotorControlTask: 3 DC motors via DRV8871 H-bridges (50Hz)
│   └── otos.rs               # OtosTask: Optical Tracking Odometry Sensor over I2C0 (20Hz)
├── servo_control.rs          # ServoControlTask: 4 RC servo actuator channels
├── utility.rs                # Housekeeping root module
├── utility/
│   ├── battery.rs            # BatteryTask: ADC battery voltage monitoring on GPIO1 (1Hz)
│   ├── button.rs             # ButtonTask: Boot button monitoring on GPIO0
│   └── led.rs                # LedTask: Heartbeat status LED on GPIO16
└── main.rs                   # System entrypoint and Embassy task spawning
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

### 1. ESP-IDF Installation

ESP-IDF tools, libraries, and bootloader support are required for compiling and linking ESP32-S3 firmware.

1. **Install system dependencies**:
   - Ubuntu / Debian:
     ```bash
     sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
     ```
   - Arch Linux:
     ```bash
     sudo pacman -S git wget flex bison gperf python python-pip cmake ninja ccache dfu-util libusb
     ```

2. **Clone and install ESP-IDF**:
   ```bash
   mkdir -p ~/esp
   cd ~/esp
   git clone --recursive https://github.com/espressif/esp-idf.git -b v5.3.1
   cd ~/esp/esp-idf
   ./install.sh esp32s3
   ```

3. **Export ESP-IDF environment**:
   ```bash
   . ~/esp/esp-idf/export.sh
   ```

---

### 2. Rust & Espressif Xtensa Toolchain

Because the ESP32-S3 uses the Xtensa architecture, the customized Rust compiler with Xtensa target support is required.

1. Install standard Rust via `rustup` if not already installed:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   ```
2. Install `espup` (Espressif toolchain manager):
   ```bash
   cargo install espup
   ```
3. Install the Xtensa Rust toolchain:
   ```bash
   espup install
   ```

---

### 3. Flashing & Monitoring Tools

Install `espflash` / `cargo-espflash`:
```bash
cargo install cargo-espflash espflash
```

---

### 4. Linux Serial Port Permissions

Ensure your user account has access to serial devices (`/dev/ttyUSB*` or `/dev/ttyACM*`):
```bash
sudo usermod -a -G dialout $USER
# On Arch / Fedora:
# sudo usermod -a -G uucp $USER
```
*(Log out and log back in for group changes to take effect).*

---

## Installation & Environment Setup

Clone the repository and enter the project directory:
```bash
git clone <repository-url>
cd CDFR2027-PAMIs-CODE-Control
```

Before compiling or flashing, export the Xtensa environment variables in your terminal session:
```bash
source $HOME/export-esp.sh
```

> **Tip**: You can add `source $HOME/export-esp.sh` to your shell profile (`~/.bashrc` or `~/.zshrc`) or use [direnv](https://direnv.net/) to source it automatically.

---

## Development Workflow

### Check & Compilation

To run a fast compile-check:
```bash
source $HOME/export-esp.sh
cargo check
```

To compile a release binary:
```bash
cargo build --release
```

### Flashing to the ESP32-S3

Connect the ESP32-S3 board via USB. 

Run the application with automatic flashing and serial monitor:
```bash
cargo run --release
```

Alternatively, use `espflash` directly:
```bash
espflash flash --monitor target/xtensa-esp32s3-none-elf/release/pami-control
```

Expected serial output on boot:
```text
Board config start
Board config done
MotorControl task tick
BrainComm task tick
ServoControl task tick
OTOS task tick
BatteryTask tick
LED toggle call
```

To exit the serial monitor, press `Ctrl + R` then `Ctrl + C` (or `Ctrl + ]`).
