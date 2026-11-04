/*
 * Motion commands consumed by the MotorControl task (Core 1), and the
 * inter-core FIFO channel types that carry them. The channel itself is
 * declared in main.rs, which wires the tasks together.
 */

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{self, Channel};

// Motion command sent from BrainComm (Core 0) to MotorControl (Core 1)
#[derive(Clone, Copy, Debug)]
pub enum MotionCommand {
    Stop,
    Velocity { vx: f32, vy: f32, omega: f32 },
    TargetPos { x: f32, y: f32, theta: f32 },
}

// Maximum number of queued motion commands
pub const MOTION_COMMAND_DEPTH: usize = 4;

pub type MotionCommandChannel =
    Channel<CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
pub type MotionCommandSender =
    channel::Sender<'static, CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
pub type MotionCommandReceiver =
    channel::Receiver<'static, CriticalSectionRawMutex, MotionCommand, MOTION_COMMAND_DEPTH>;
