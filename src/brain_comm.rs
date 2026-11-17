mod communication;
mod protocol;

pub use communication::{BrainCommLinks, BrainCommTask, handle};
pub use protocol::{
    BATTERY_LOW_FLAG, FRAME_LEN, Frame, MAGIC, ODOMETRY_NOT_READY_FLAG, RESTARTED_FLAG, Request,
    Status, crc8, decode, encode_reply,
};
