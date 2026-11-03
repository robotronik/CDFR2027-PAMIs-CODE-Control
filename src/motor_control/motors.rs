use crate::board::DcMotorPins;
use crate::protocol::{MOTION_COMMAND_CHANNEL, ROBOT_POSE_WATCH, MotionCommand, RobotPose};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_println::println;

#[embassy_executor::task]
async fn run_motor_control(mut motors: [DcMotorPins; 3]) {
    /*
     * Real-time drivetrain motor control loop pinned to Core 1.
     * Consumes motion commands from Core 0 and publishes updated robot pose.
     */
    let mut current_pose = RobotPose::default();
    let pose_sender = ROBOT_POSE_WATCH.sender();

    loop {
        Timer::after(Duration::from_millis(20)).await;

        // Check for newly arrived target commands from Core 0
        if let Ok(command) = MOTION_COMMAND_CHANNEL.try_receive() {
            println!("MotorControl (Core 1) received command: {:?}", command);
            match command {
                MotionCommand::Stop => {
                    current_pose.vx = 0.0;
                    current_pose.vy = 0.0;
                    current_pose.omega = 0.0;
                }
                MotionCommand::Velocity { vx, vy, omega } => {
                    current_pose.vx = vx;
                    current_pose.vy = vy;
                    current_pose.omega = omega;
                }
                MotionCommand::TargetPose { x, y, theta } => {
                    current_pose.x = x;
                    current_pose.y = y;
                    current_pose.theta = theta;
                }
            }
        }

        // Drivetrain kinematics update
        let _ = &mut motors;

        // Publish latest robot position back to Core 0
        pose_sender.send(current_pose);

        println!("MotorControl (Core 1) tick");
    }
}

pub struct MotorControlTask;

impl MotorControlTask {
    pub fn spawn(spawner: &Spawner, motors: [DcMotorPins; 3]) {
        let token = run_motor_control(motors).expect("Failed to create MotorControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
