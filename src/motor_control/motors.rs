use super::motor_commands::{MotionCommand, MotionCommandReceiver};
use super::otos::Otos;
use super::robot_state::{RobotPos, RobotPosSender};
use crate::board::DcMotorPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use log::debug;

const CONTROL_PERIOD: Duration = Duration::from_millis(20);
const MAX_DT: Duration = Duration::from_millis(50);

const PWM_FREQ: Rate = Rate::from_khz(20);
// Timer counts 0..PWM_PERIOD-1, so a timestamp of PWM_PERIOD is 100 % duty
const PWM_PERIOD: u16 = 1000;

// DRV8871 sign-magnitude: (IN1, IN2) duty, PWM on IN1 forward, on IN2 backward
pub fn motor_duty(command: i16) -> (u16, u16) {
    let duty = command.unsigned_abs().min(PWM_PERIOD);
    if command >= 0 { (duty, 0) } else { (0, duty) }
}

// Returns one signed duty per motor, in -PWM_PERIOD..=PWM_PERIOD
// delta_pos is in the world frame (OTOS): rotate by -theta before mapping to wheels
fn calculate_motor_commands(delta_pos: RobotPos, dt: Duration) -> [i16; 3] {
    let dt_s = dt.as_micros() as f32 * 1e-6;

    let commands: [i16; 3] = [0; 3];

    commands
}

#[embassy_executor::task]
async fn run_motor_control(
    motors: [DcMotorPins; 3],
    mcpwm0: MCPWM0<'static>,
    mut otos: Otos,
    command_receiver: MotionCommandReceiver,
    pos_sender: RobotPosSender,
) {
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
    let mut mcpwm = McPwm::new(mcpwm0, clock_cfg);
    // All operators share timer0 so the three motors switch in phase
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);
    mcpwm.operator2.set_timer(&mcpwm.timer0);

    let [m0, m1, m2] = motors;
    let mut pwm0 = mcpwm.operator0.with_pins(
        m0.in1,
        PwmPinConfig::UP_ACTIVE_HIGH,
        m0.in2,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let mut pwm1 = mcpwm.operator1.with_pins(
        m1.in1,
        PwmPinConfig::UP_ACTIVE_HIGH,
        m1.in2,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );
    let mut pwm2 = mcpwm.operator2.with_pins(
        m2.in1,
        PwmPinConfig::UP_ACTIVE_HIGH,
        m2.in2,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let timer_cfg = clock_cfg
        .timer_clock_with_frequency(PWM_PERIOD - 1, PwmWorkingMode::Increase, PWM_FREQ)
        .unwrap();
    mcpwm.timer0.start(timer_cfg);

    let mut target_pos = RobotPos::default();

    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_read_time = Instant::now();

    loop {
        ticker.next().await;

        let read_time = Instant::now();
        let current_pos = otos.read_pos();

        if let Ok(command) = command_receiver.try_receive() {
            debug!("MotorControl (Core 1) received command: {:?}", command);
            match command {
                // Placeholder commands, subject to change
                MotionCommand::Stop => {
                    target_pos = RobotPos {
                        x: current_pos.x,
                        y: current_pos.y,
                        theta: current_pos.theta,
                        ..Default::default()
                    }
                }
                MotionCommand::TargetPos { x, y, theta } => {
                    target_pos = RobotPos {
                        x,
                        y,
                        theta,
                        ..Default::default()
                    }
                }
            }
        }

        // Clamped so a stall doesn't produce a huge step
        let dt = (read_time - last_read_time).min(MAX_DT);
        last_read_time = read_time;

        let commands = calculate_motor_commands(target_pos - current_pos, dt);

        let (in1, in2) = motor_duty(commands[0]);
        pwm0.0.set_timestamp(in1);
        pwm0.1.set_timestamp(in2);
        let (in1, in2) = motor_duty(commands[1]);
        pwm1.0.set_timestamp(in1);
        pwm1.1.set_timestamp(in2);
        let (in1, in2) = motor_duty(commands[2]);
        pwm2.0.set_timestamp(in1);
        pwm2.1.set_timestamp(in2);

        pos_sender.send(current_pos);
    }
}

pub struct MotorControlTask;

impl MotorControlTask {
    pub fn spawn(
        spawner: &Spawner,
        motors: [DcMotorPins; 3],
        mcpwm0: MCPWM0<'static>,
        otos: Otos,
        command_receiver: MotionCommandReceiver,
        pos_sender: RobotPosSender,
    ) {
        let token = run_motor_control(motors, mcpwm0, otos, command_receiver, pos_sender)
            .expect("Failed to create MotorControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
