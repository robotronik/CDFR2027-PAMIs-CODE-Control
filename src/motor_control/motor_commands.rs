// MotorControl inbox. The channel instance lives in main.rs.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Channel};

#[derive(Clone, Copy, Debug)]
pub enum MotionCommand {
    Stop,
    Velocity { vx: f32, vy: f32, omega: f32 },
    TargetPos { x: f32, y: f32, theta: f32 },
}

pub const MOTION_COMMAND_DEPTH: usize = 4;

pub type MotionCommandChannel =
    Channel<CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
pub type MotionCommandSender =
    channel::Sender<'static, CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
pub type MotionCommandReceiver =
    channel::Receiver<'static, CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
