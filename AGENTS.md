# AGENTS.md

Control firmware for a PAMI robot (Coupe de France de Robotique 2027, Robotronik).
ESP32-S3, Rust `no_std`, esp-hal 1.2 + esp-rtos (Embassy executor). It's a holonomic
3-motor drivetrain, an OTOS odometry sensor and 4 servos, driven over SPI by a separate BrainBoard.

## Build & flash

```sh
source ~/export-esp.sh   # required first, or the Xtensa linker is not found
cargo build
cargo run                # flashes with espflash and opens the serial monitor
```

Toolchain is `esp` (see `rust-toolchain.toml`), target `xtensa-esp32s3-none-elf`.

Tests run on the board with embedded-test: `cargo hil` (board on USB, needs probe-rs:
`cargo install probe-rs-tools --locked`). Suites live in `tests/`, one `[[test]]` entry per file
in `Cargo.toml`. CI only compiles them (`cargo test --no-run`). The crate is a lib (`src/lib.rs`)
plus the firmware bin (`src/main.rs`) so tests can import modules.

## Architecture

- **Core 1** runs only `MotorControlTask`, on its own executor with an 8 KB stack. It owns
  the motors and the OTOS. Drivers that bind interrupts (e.g. `Otos::new`) must be created
  inside the Core 1 closure in `main.rs`.
- **Core 0** runs everything else: LED, button, battery, `ServoControlTask`, `BrainCommTask`.
- **Tasks own their hardware and data.** They communicate only through `embassy_sync`
  channels declared as statics in `main.rs` and passed in as endpoints (no other globals):
  - `MOTION_COMMANDS` (`Channel`): BrainComm -> MotorControl
  - `ROBOT_POS` (`Watch`, latest value): MotorControl -> BrainComm
  - `SERVO_COMMANDS` (`Channel`): BrainComm -> ServoControl
  
  Use `CriticalSectionRawMutex` for anything crossing cores.
- The BrainBoard is the SPI master. Wire format spec: [wiki](https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/wiki/BrainBoard-SPI-Protocol).
  `brain_comm/protocol.rs` encodes/decodes frames (pure functions, no I/O),
  `brain_comm/communication.rs` runs the SPI exchange and routes requests in `handle`.
- `board.rs` is the single source of truth for pin mapping and peripheral ownership.

## Code conventions

- Modules use the `foo.rs` + `foo/` layout. The root file only declares private `mod`s and
  re-exports the public API with explicit `pub use`.
- Each task module defines its own inbox: command enum, queue depth const, and
  `Channel`/`Sender`/`Receiver` type aliases.
- Each task is spawned through a `XxxTask::spawn(&spawner, ...)` wrapper.
- Comments only where the code isn't self-explanatory, as single-line `//`.
- Log with the `log` macros (`info!`, `warn!`, `debug!`...), not `println!`. The filter is
  `ESP_LOG` in `.cargo/config.toml`, read at build time (e.g. `info,pami_control::motor_control=debug`).
- Keep maths (angle wrapping, control law, protocol decoding) free of HAL types so it can
  be host-tested later.

## Working rules

- Commits go straight to `master`, message style `(type:) summary`
  (e.g. `(feat:)`, `(refactor:)`, `(cleanup:)`). Don't commit `.claude/`.
- Prefer the codebase-memory-mcp graph over reading files to save tokens: `search_graph` /
  `trace_path` for symbols and call chains, `get_code_snippet` for one function,
  `get_architecture` for an overview. Read or grep files only for literals or when the graph
  lacks coverage. Re-index with `index_repository` (mode `fast`) after large changes.
- Known gaps, deferred on purpose: panic handler doesn't stop the motors, no Brain-silence
  watchdog, servos on plain GPIO (need LEDC PWM), motor control law and drivers are stubs.
