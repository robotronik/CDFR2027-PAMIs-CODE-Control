use super::motor_commands::{MotionCommand, MotionCommandReceiver};
use super::otos::Otos;
use super::robot_state::{RobotPos, RobotPosSender};
use crate::board::DcMotorPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_println::println;

const CONTROL_PERIOD: Duration = Duration::from_millis(20);
const MAX_DT: Duration = Duration::from_millis(50);

fn calculate_motor_commands(delta_pos: RobotPos, dt: Duration) -> [i16; 3] {
    let dt_s = dt.as_micros() as f32 * 1e-6;

    let commands: [i16; 3] = [0; 3];

    commands
}

#[embassy_executor::task]
async fn run_motor_control(
    mut motors: [DcMotorPins; 3],
    mut otos: Otos,
    command_receiver: MotionCommandReceiver,
    pos_sender: RobotPosSender,
) {
    /*
     * Real-time drivetrain motor control loop pinned to Core 1.
     * Consumes motion commands from Core 0 and publishes updated robot pos.
     */
    let mut target_pos = RobotPos::default();

    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_read_time = Instant::now();

    loop {
        ticker.next().await;

        // Update current position
        let read_time = Instant::now();
        let current_pos = otos.read_pos();

        // Check for newly arrived target commands from Core 0
        if let Ok(command) = command_receiver.try_receive() {
            println!("MotorControl (Core 1) received command: {:?}", command);
            match command {
                /* These are placeholder commands, subject to change */
                MotionCommand::Stop => {
                    // Hold the current pose so a previous TargetPos is abandoned
                    target_pos.x = current_pos.x;
                    target_pos.y = current_pos.y;
                    target_pos.theta = current_pos.theta;
                    target_pos.vx = 0.0;
                    target_pos.vy = 0.0;
                    target_pos.omega = 0.0;
                }
                MotionCommand::Velocity { vx, vy, omega } => {
                    target_pos.vx = vx;
                    target_pos.vy = vy;
                    target_pos.omega = omega;
                }
                MotionCommand::TargetPos { x, y, theta } => {
                    target_pos.x = x;
                    target_pos.y = y;
                    target_pos.theta = theta;
                }
            }
        }

        // Time elapsed since the previous position read, clamped against stalls
        let dt = (read_time - last_read_time).min(MAX_DT);
        last_read_time = read_time;

        let delta_pos: RobotPos = target_pos - current_pos;
        let commands: [i16; 3] = calculate_motor_commands(delta_pos, dt);

        // TODO: trigger MCPWM generator here

        // Publish latest robot position back to Core 0
        pos_sender.send(current_pos);

        println!("MotorControl (Core 1) tick");
    }
}

pub struct MotorControlTask;

impl MotorControlTask {
    pub fn spawn(
        spawner: &Spawner,
        motors: [DcMotorPins; 3],
        otos: Otos,
        command_receiver: MotionCommandReceiver,
        pos_sender: RobotPosSender,
    ) {
        let token = run_motor_control(motors, otos, command_receiver, pos_sender)
            .expect("Failed to create MotorControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
