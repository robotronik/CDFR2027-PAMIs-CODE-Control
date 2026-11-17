use super::control_law::{CONTROL_PERIOD, ControlLoop, PWM_PERIOD, motor_duty};
use super::motor_commands::{MotionCommand, MotionCommandReceiver};
use super::otos::Otos;
use super::robot_state::{RobotPos, RobotPosSender};
use crate::board::DcMotorPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use log::{debug, info, warn};

const MAX_DT: Duration = Duration::from_millis(50);

const PWM_FREQ: Rate = Rate::from_khz(20);
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
