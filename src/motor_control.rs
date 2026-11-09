// Drivetrain control loop on Core 1: owns the motors and the OTOS
mod motor_commands;
mod motors;
mod otos;
mod robot_state;

pub use motor_commands::{MotionCommand, MotionCommandChannel, MotionCommandSender};
pub use motors::{MotorControlTask, motor_duty};
pub use otos::{Otos, decode_pose};
pub use robot_state::{RobotPos, RobotPosReceiver, RobotPosWatch};
