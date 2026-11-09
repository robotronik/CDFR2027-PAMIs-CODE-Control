// ServoControl inbox. The channel instance lives in main.rs.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Channel};

#[derive(Clone, Copy, Debug)]
pub enum ServoCommand {
    SetAngle { id: u8, angle_deg: u8 },
}

pub const SERVO_COMMAND_DEPTH: usize = 4;

pub type ServoCommandChannel = Channel<CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
pub type ServoCommandSender =
    channel::Sender<'static, CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
pub type ServoCommandReceiver =
    channel::Receiver<'static, CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
