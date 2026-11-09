// SPI slave link to the BrainBoard; routes each request to the task that owns it
mod communication;
mod protocol;

pub use communication::{BrainCommLinks, BrainCommTask, handle};
pub use protocol::{FRAME_LEN, Frame, MAGIC, Request, Status, crc8, decode, encode_reply};
