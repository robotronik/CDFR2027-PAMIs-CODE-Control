// SPI slave link to the BrainBoard; routes each request to the task that owns it
mod communication;
mod protocol;

pub use communication::{BrainCommLinks, BrainCommTask, handle};
pub use protocol::{
    BATTERY_LOW_FLAG, ODOMETRY_NOT_READY_FLAG, RESTARTED_FLAG, FRAME_LEN, Frame, MAGIC, Request, Status, crc8, decode, encode_reply,
};
