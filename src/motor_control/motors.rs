use super::motor_commands::{MotionCommand, MotionCommandReceiver};
use super::otos::Otos;
use super::robot_state::{RobotPos, RobotPosSender, RobotVel};
use crate::board::DcMotorPins;
use core::f32::consts::PI;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use libm::{cosf, roundf, sinf};
use log::{debug, info, warn};

// PID per world axis (x, y, theta), then projected onto the three omni wheels
#[derive(Default)]
pub struct ControlLoop {
    error_integral: [f32; 3],
}

impl ControlLoop {
    // Gains map mm of wheel travel to duty (TODO: calibrate)
    const KP: f32 = 1.0;
    const KI: f32 = 0.1;
    const KD: f32 = 0.1;
    // Anti-windup clamp, in mm.s
    const INTEGRAL_LIMIT: f32 = 100.0;
    // Robot centre to wheel contact, in mm (TODO: measure)
    const WHEEL_DISTANCE: f32 = 1.0;
    // Motor i sits at this angle in the robot frame, CCW from the robot x axis (TODO: check order and spin sign)
    const WHEEL_ANGLES: [f32; 3] = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0];

    // Call on each new target so integral windup doesn't carry over
    pub fn reset(&mut self) {
        self.error_integral = [0.0; 3];
    }

    // error = target - current. Targets are still, so the error derivative is -velocity.
    // current_heading is needed on top of the error: it says which world direction each wheel faces.
    pub fn calculate_motor_commands(
        &mut self,
        error: RobotPos,
        velocity: RobotVel,
        current_heading: f32,
        dt: Duration,
    ) -> [i16; 3] {
        let dt_s = dt.as_micros() as f32 * 1e-6;

        // Theta scaled to wheel travel so one set of gains fits all three axes
        let position_error = [error.x, error.y, error.theta * Self::WHEEL_DISTANCE];
        let velocity_error = [
            -velocity.vx,
            -velocity.vy,
            -velocity.omega * Self::WHEEL_DISTANCE,
        ];
        let mut axis_command = [0.0; 3];
        for axis in 0..3 {
            self.error_integral[axis] = (self.error_integral[axis] + position_error[axis] * dt_s)
                .clamp(-Self::INTEGRAL_LIMIT, Self::INTEGRAL_LIMIT);
            axis_command[axis] = Self::KP * position_error[axis]
                + Self::KI * self.error_integral[axis]
                + Self::KD * velocity_error[axis];
        }
        let [x_command, y_command, rotation_command] = axis_command;

        // Each wheel pushes along the tangent of its world angle; rotation adds equally to all wheels
        let wheel_commands = Self::WHEEL_ANGLES.map(|wheel_angle| {
            let world_angle = wheel_angle + current_heading;
            -sinf(world_angle) * x_command + cosf(world_angle) * y_command + rotation_command
        });

        // Scale all wheels together when one saturates, so the direction is kept
        let largest = wheel_commands
            .iter()
            .fold(0.0f32, |acc, command| acc.max(command.abs()));
        let scale = (PWM_PERIOD as f32 / largest).min(1.0);
        wheel_commands.map(|command| roundf(command * scale) as i16)
    }
}

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

    let mut set_duties = |duties: [(u16, u16); 3]| {
        pwm0.0.set_timestamp(duties[0].0);
        pwm0.1.set_timestamp(duties[0].1);
        pwm1.0.set_timestamp(duties[1].0);
        pwm1.1.set_timestamp(duties[1].1);
        pwm2.0.set_timestamp(duties[2].0);
        pwm2.1.set_timestamp(duties[2].1);
    };

    otos.init().await;

    let mut target_pos = RobotPos::default();
    // Stop needs the current pose, only known after the OTOS read. Pending at boot so the robot
    // holds where it is: after a warm restart the OTOS pose is kept and origin may be far away.
    let mut stop_pending = true;
    let mut otos_ok = true;
    let mut control_loop = ControlLoop::default();

    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut last_read_time = Instant::now();

    loop {
        ticker.next().await;

        // Drained before the OTOS read so commands keep flowing during a sensor outage; the last one wins
        while let Ok(command) = command_receiver.try_receive() {
            debug!("MotorControl (Core 1) received command: {:?}", command);
            match command {
                MotionCommand::Stop => {
                    stop_pending = true;
                    control_loop.reset();
                }
                MotionCommand::TargetPos { x, y, theta } => {
                    target_pos = RobotPos { x, y, theta };
                    stop_pending = false;
                    control_loop.reset();
                }
            }
        }

        let read_time = Instant::now();
        let (current_pos, current_vel) = match otos.read_pos().await {
            Ok(reading) => reading,
            Err(err) => {
                // Steering on a stale position is worse than braking for a tick
                if otos_ok {
                    warn!("MotorControl: OTOS read failed ({:?}), braking", err);
                    otos_ok = false;
                }
                // Both inputs high = DRV8871 brake (both low would coast)
                set_duties([(PWM_PERIOD, PWM_PERIOD); 3]);
                // Empty watch: BrainComm reports ODOMETRY_NOT_READY
                pos_sender.clear();
                continue;
            }
        };

        if !otos_ok {
            info!("MotorControl: OTOS reads recovered");
            otos_ok = true;
        }

        // Brake, then hold the pose where the robot was when Stop arrived
        if stop_pending {
            target_pos = current_pos;
            stop_pending = false;
        }

        // Clamped so a stall doesn't produce a huge step
        let dt = (read_time - last_read_time).min(MAX_DT);
        last_read_time = read_time;

        let commands = control_loop.calculate_motor_commands(
            target_pos - current_pos,
            current_vel,
            current_pos.theta,
            dt,
        );
        set_duties(commands.map(motor_duty));

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
