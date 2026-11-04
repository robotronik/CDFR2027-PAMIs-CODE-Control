use crate::board::DcMotorPins;
use crate::protocol::{MOTION_COMMAND_CHANNEL, MotionCommand, ROBOT_POS_CHANNEL, RobotPos};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_println::println;

#[embassy_executor::task]
async fn run_motor_control(mut motors: [DcMotorPins; 3]) {
    /*
     * Real-time drivetrain motor control loop pinned to Core 1.
     * Consumes motion commands from Core 0 and publishes updated robot pos.
     */
    let mut current_pos = RobotPos::default();
    let pos_sender = ROBOT_POS_CHANNEL.sender();

    loop {
        Timer::after(Duration::from_millis(20)).await;

        // Check for newly arrived target commands from Core 0
        if let Ok(command) = MOTION_COMMAND_CHANNEL.try_receive() {
            println!("MotorControl (Core 1) received command: {:?}", command);
            match command {
                MotionCommand::Stop => {
                    current_pos.vx = 0.0;
                    current_pos.vy = 0.0;
                    current_pos.omega = 0.0;
                }
                MotionCommand::Velocity { vx, vy, omega } => {
                    current_pos.vx = vx;
                    current_pos.vy = vy;
                    current_pos.omega = omega;
                }
                MotionCommand::TargetPos { x, y, theta } => {
                    current_pos.x = x;
                    current_pos.y = y;
                    current_pos.theta = theta;
                }
            }
        }

        // Drivetrain kinematics update
        let _ = &mut motors;

        // Publish latest robot position back to Core 0
        pos_sender.send(current_pose);

        println!("MotorControl (Core 1) tick");
    }
}

pub struct MotorControlTask;

impl MotorControlTask {
    pub fn spawn(spawner: &Spawner, motors: [DcMotorPins; 3]) {
        let token = run_motor_control(motors)
            .expect("Failed to create MotorControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
