// SPI frame <-> request/response encoding. Pure functions, no I/O.
// Wire format: https://github.com/robotronik/CDFR2027-PAMIs-CODE-Control/wiki/BrainBoard-SPI-Protocol

use crate::motor_control::RobotPos;
use crate::servo_control::SERVO_COUNT;

pub const FRAME_LEN: usize = 16;
pub const MAGIC: u8 = 0xA5;

pub type Frame = [u8; FRAME_LEN];

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Status {
    Ack = 0,
    BadMagic = 1,
    BadCrc = 2,
    UnknownOpcode = 3,
    BadArgument = 4,
    QueueFull = 5,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Request {
    Nop,
    Stop,
    TargetPos { x: f32, y: f32, theta: f32 },
    SetServo { id: u8, angle_deg: u8 },
}

// CRC-8/SMBUS: poly 0x07, init 0x00
pub fn crc8(bytes: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}

// Returns the seq byte even on error, so the reply can still echo it
pub fn decode(frame: &Frame) -> (u8, Result<Request, Status>) {
    (frame[2], decode_request(frame))
}

fn decode_request(frame: &Frame) -> Result<Request, Status> {
    if frame[0] != MAGIC {
        return Err(Status::BadMagic);
    }
    if crc8(&frame[..15]) != frame[15] {
        return Err(Status::BadCrc);
    }
    let data = &frame[3..15];
    match frame[1] {
        0x00 => Ok(Request::Nop),
        0x01 => Ok(Request::Stop),
        0x02 => Ok(Request::TargetPos {
            x: read_f32(data, 0)?,
            y: read_f32(data, 4)?,
            theta: read_f32(data, 8)?,
        }),
        0x03 if (data[0] as usize) < SERVO_COUNT && data[1] <= 180 => Ok(Request::SetServo { id: data[0], angle_deg: data[1] }),
        0x03 => Err(Status::BadArgument),
        _ => Err(Status::UnknownOpcode),
    }
}

fn read_f32(data: &[u8], at: usize) -> Result<f32, Status> {
    let value = f32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]]);
    if value.is_finite() { Ok(value) } else { Err(Status::BadArgument) }
}

// Bit 7 of the reply status byte, set while the battery is low
pub const BATTERY_LOW_FLAG: u8 = 0x80;

pub fn encode_reply(status: Status, seq: u8, pos: &RobotPos, battery_low: bool) -> Frame {
    let mut frame = [0u8; FRAME_LEN];
    frame[0] = MAGIC;
    frame[1] = status as u8 | if battery_low { BATTERY_LOW_FLAG } else { 0 };
    frame[2] = seq;
    frame[3..7].copy_from_slice(&pos.x.to_le_bytes());
    frame[7..11].copy_from_slice(&pos.y.to_le_bytes());
    frame[11..15].copy_from_slice(&pos.theta.to_le_bytes());
    frame[15] = crc8(&frame[..15]);
    frame
}
