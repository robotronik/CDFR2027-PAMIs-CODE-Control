/* Servo control module: owns the actuator servos and defines its own inbox
* (servo commands).
*/
mod servo_commands;
mod servos;

pub use servo_commands::{ServoCommand, ServoCommandChannel, ServoCommandSender};
pub use servos::ServoControlTask;
