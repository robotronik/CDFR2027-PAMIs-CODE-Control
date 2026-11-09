# BrainBoard <-> PAMI SPI protocol

The BrainBoard is the SPI master (mode 0), the PAMI is the slave. Every transaction exchanges
exactly **16 bytes** in each direction. Multi-byte values are little-endian, floats are IEEE-754
`f32`, distances in **mm**, angles in **rad** (except the servo angle, see below).

## Handshake pin

The PAMI drives the handshake pin **high** when it has armed a transfer with a fresh reply frame,
and **low** as soon as that transfer completes. The Brain must only start a transaction while the
pin is high.

## Command frame (Brain -> PAMI)

| Byte  | Field  | Notes                                  |
|-------|--------|----------------------------------------|
| 0     | magic  | `0xA5`                                 |
| 1     | opcode | see below                              |
| 2     | seq    | free-running counter chosen by Brain   |
| 3..15 | data   | 12 bytes, layout depends on opcode     |
| 15    | crc    | CRC-8 of bytes 0..15                   |

| Opcode | Name        | Data                                                       |
|--------|-------------|------------------------------------------------------------|
| `0x00` | `Nop`       | ignored (send zeros). Use it to just read the position.    |
| `0x01` | `Stop`      | ignored                                                    |
| `0x02` | `TargetPos` | `x: f32` (3..7), `y: f32` (7..11), `theta: f32` (11..15)   |
| `0x03` | `SetServo`  | `id: u8` (3), `angle_deg: u8` (4, 0..=180), rest ignored   |

## Reply frame (PAMI -> Brain)

| Byte  | Field  | Notes                                                       |
|-------|--------|-------------------------------------------------------------|
| 0     | magic  | `0xA5`                                                      |
| 1     | status | result of the **previous** command, see below               |
| 2     | seq    | seq byte of the previous command                            |
| 3..15 | pos    | `x: f32`, `y: f32`, `theta: f32`: latest odometry (mm, rad) |
| 15    | crc    | CRC-8 of bytes 0..15                                        |

SPI is full-duplex and the reply is loaded before the Brain clocks, so the status for command N
arrives in transaction N+1. To get it immediately, follow with a `Nop`. The first reply after
boot has status `Ack` and seq `0`.

| Status | Name            | Meaning                                         |
|--------|-----------------|-------------------------------------------------|
| `0x00` | `Ack`           | command accepted and queued                     |
| `0x01` | `BadMagic`      | byte 0 was not `0xA5`                           |
| `0x02` | `BadCrc`        | CRC mismatch                                    |
| `0x03` | `UnknownOpcode` | opcode not in the table                         |
| `0x04` | `BadArgument`   | non-finite float, or servo angle > 180          |
| `0x05` | `QueueFull`     | target task's inbox full, command dropped       |

## CRC

CRC-8/SMBUS: polynomial `0x07`, init `0x00`, no reflection, no final XOR.
Check value: `crc8(b"123456789") == 0xF4`.

## Extending

Add an opcode: new row above, new `Request` variant + decode arm in `src/brain_comm/protocol.rs`,
new arm in `handle` (`src/brain_comm/communication.rs`). New failure: append a `Status` code.
