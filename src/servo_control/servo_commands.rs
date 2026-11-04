/*
 * Servo commands consumed by the ServoControl task (Core 0), and the
 * FIFO channel types that carry them. The channel itself is declared in
 * main.rs, which wires the tasks together.
 */

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Channel};

// Servo command sent from BrainComm to ServoControl
#[derive(Clone, Copy, Debug)]
pub enum ServoCommand {
    SetAngle { id: u8, angle_deg: f32 },
}

// Maximum number of queued servo commands
pub const SERVO_COMMAND_DEPTH: usize = 4;

pub type ServoCommandChannel = Channel<CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
pub type ServoCommandSender =
    channel::Sender<'static, CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
pub type ServoCommandReceiver =
    channel::Receiver<'static, CriticalSectionRawMutex, ServoCommand, SERVO_COMMAND_DEPTH>;
