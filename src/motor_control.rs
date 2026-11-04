/* Motor control module: owns the drivetrain motors and the OTOS odometry
* sensor, runs the control loop, and defines its own inbox (motion commands)
* and outbox (robot position).
*/
mod motor_commands;
mod motors;
mod otos;
mod robot_state;

pub use motor_commands::{MotionCommand, MotionCommandChannel, MotionCommandSender};
pub use motors::MotorControlTask;
pub use otos::Otos;
pub use robot_state::{RobotPos, RobotPosReceiver, RobotPosWatch};
