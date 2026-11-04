// Drivetrain control loop on Core 1: owns the motors and the OTOS
mod motor_commands;
mod motors;
mod otos;
mod robot_state;

pub use motor_commands::{MotionCommand, MotionCommandChannel, MotionCommandSender};
pub use motors::MotorControlTask;
pub use otos::Otos;
pub use robot_state::{RobotPos, RobotPosReceiver, RobotPosWatch};
