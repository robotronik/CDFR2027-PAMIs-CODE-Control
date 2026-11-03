use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;

// Motion command sent from SPI task (Core 0) to MotorControl task (Core 1)
#[derive(Clone, Copy, Debug)]
pub enum MotionCommand {
    Stop,
    Velocity { vx: f32, vy: f32, omega: f32 },
    TargetPose { x: f32, y: f32, theta: f32 },
}

// Current robot position telemetry sent from Core 1 back to Core 0
#[derive(Clone, Copy, Debug, Default)]
pub struct RobotPose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
}

/*
 * Inter-Core Communication Channels
 * - MOTION_COMMAND_CHANNEL: FIFO queue delivering target motion commands to Core 1.
 * - ROBOT_POSE_WATCH: Single-slot watch channel publishing the freshest robot pose to Core 0.
 */
pub static MOTION_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotionCommand, 4> = Channel::new();
pub static ROBOT_POSE_WATCH: Watch<CriticalSectionRawMutex, RobotPose, 2> = Watch::new();
