// Control law, free of HAL types so the host simulator in sim/ can build it
use super::robot_state::{RobotPos, RobotVel};
use core::f32::consts::PI;
use embassy_time::Duration;
use libm::{cosf, roundf, sinf};

// PID per world axis (x, y, theta), then projected onto the three omni wheels
#[derive(Clone, Copy, Debug)]
pub struct ControlParams {
    // Gains map mm of wheel travel to duty
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    // Anti-windup clamp, in mm.s
    pub integral_limit: f32,
    // Robot centre to wheel contact, in mm
    pub wheel_distance: f32,
}

impl ControlParams {
    // TODO: calibrate the gains and measure wheel_distance
    pub const DEFAULT: Self = Self {
        kp: 1.0,
        ki: 0.1,
        kd: 0.1,
        integral_limit: 100.0,
        wheel_distance: 1.0,
    };
}

// Motor i sits at this angle in the robot frame, CCW from the robot x axis (TODO: check order and spin sign)
pub const WHEEL_ANGLES: [f32; 3] = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0];

pub struct ControlLoop {
    pub params: ControlParams,
    error_integral: [f32; 3],
}

impl Default for ControlLoop {
    fn default() -> Self {
        Self {
            params: ControlParams::DEFAULT,
            error_integral: [0.0; 3],
        }
    }
}

impl ControlLoop {
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
        let ControlParams {
            kp,
            ki,
            kd,
            integral_limit,
            wheel_distance,
        } = self.params;

        // Theta scaled to wheel travel so one set of gains fits all three axes
        let position_error = [error.x, error.y, error.theta * wheel_distance];
        let velocity_error = [-velocity.vx, -velocity.vy, -velocity.omega * wheel_distance];
        let mut axis_command = [0.0; 3];
        for axis in 0..3 {
            self.error_integral[axis] = (self.error_integral[axis] + position_error[axis] * dt_s)
                .clamp(-integral_limit, integral_limit);
            axis_command[axis] = kp * position_error[axis]
                + ki * self.error_integral[axis]
                + kd * velocity_error[axis];
        }
        let [x_command, y_command, rotation_command] = axis_command;

        // Each wheel pushes along the tangent of its world angle; rotation adds equally to all wheels
        let wheel_commands = WHEEL_ANGLES.map(|wheel_angle| {
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

pub const CONTROL_PERIOD: Duration = Duration::from_millis(20);

// Timer counts 0..PWM_PERIOD-1, so a timestamp of PWM_PERIOD is 100 % duty
pub const PWM_PERIOD: u16 = 1000;

// DRV8871 sign-magnitude: (IN1, IN2) duty, PWM on IN1 forward, on IN2 backward
pub fn motor_duty(command: i16) -> (u16, u16) {
    let duty = command.unsigned_abs().min(PWM_PERIOD);
    if command >= 0 { (duty, 0) } else { (0, duty) }
}
