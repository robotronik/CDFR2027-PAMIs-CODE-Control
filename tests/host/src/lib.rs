// Firmware maths that is free of HAL types, tested on the host: `cargo test` in this folder.
// On-device suites live in ../hil and run with `cargo hil`.

#[path = "../../../src/motor_control/control_law.rs"]
pub mod control_law;
#[path = "../../../src/motor_control/robot_state.rs"]
pub mod robot_state;

#[cfg(test)]
mod tests {
    use crate::control_law::{ControlLoop, ControlParams, motor_duty};
    use crate::robot_state::{RobotPos, RobotVel, wrap_angle};
    use core::f32::consts::{PI, TAU};
    use embassy_time::Duration;

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-5,
            "{} != {}",
            actual,
            expected
        );
    }

    #[test]
    fn wrap_angle_keeps_in_range_values() {
        assert_close(wrap_angle(0.0), 0.0);
        assert_close(wrap_angle(-0.5), -0.5);
    }

    // Both ends of the range map to -pi, since the range is [-pi, pi)
    #[test]
    fn wrap_angle_maps_pi_to_minus_pi() {
        assert_close(wrap_angle(PI), -PI);
        assert_close(wrap_angle(-PI), -PI);
    }

    #[test]
    fn wrap_angle_removes_full_turns() {
        assert_close(wrap_angle(TAU + 0.5), 0.5);
        assert_close(wrap_angle(-TAU - 0.5), -0.5);
    }

    // Pose difference: x/y subtract plainly, theta takes the short way across ±pi (6 - 2pi, not 6)
    #[test]
    fn robot_pos_sub_takes_shortest_rotation() {
        let a = RobotPos {
            x: 1.0,
            y: 2.0,
            theta: 3.0,
        };
        let b = RobotPos {
            x: 0.5,
            y: 3.0,
            theta: -3.0,
        };
        let d = a - b;
        assert_close(d.x, 0.5);
        assert_close(d.y, -1.0);
        assert_close(d.theta, 6.0 - TAU);
    }

    #[test]
    fn motor_duty_drives_one_input_by_sign() {
        assert_eq!(motor_duty(0), (0, 0));
        assert_eq!(motor_duty(500), (500, 0));
        assert_eq!(motor_duty(-500), (0, 500));
    }

    // Out-of-range speeds clamp to full duty (1000), including i16::MIN without overflow
    #[test]
    fn motor_duty_clamps_to_full_duty() {
        assert_eq!(motor_duty(2000), (1000, 0));
        assert_eq!(motor_duty(i16::MIN), (0, 1000));
    }

    // Round gains, and max wheel speed = PWM_PERIOD so duty equals wheel speed in mm/s
    fn unit_loop() -> ControlLoop {
        let mut control = ControlLoop::default();
        control.params = ControlParams {
            kp: 1.0,
            ki: 0.1,
            kd: 0.1,
            integral_limit: 100.0,
            max_wheel_speed: 1000.0,
            ..ControlParams::DEFAULT
        };
        control
    }

    fn y_error_command(control: &mut ControlLoop, y: f32, dt: Duration) -> [i16; 3] {
        control.calculate_motor_commands(
            RobotPos {
                y,
                ..Default::default()
            },
            RobotVel::default(),
            0.0,
            dt,
        )
    }

    // Zero dt and velocity leave only P (KP = 1): wheels at 0/120/240° split a +y error 1 : -1/2 : -1/2
    #[test]
    fn motor_commands_project_error_on_wheels() {
        let cmd = |x, y, theta, heading| {
            unit_loop().calculate_motor_commands(
                RobotPos { x, y, theta },
                RobotVel::default(),
                heading,
                Duration::from_ticks(0),
            )
        };
        assert_eq!(cmd(0.0, 100.0, 0.0, 0.0), [100, -50, -50]);
        // Same move seen from a robot turned 90°: the error is now along -x in the world
        assert_eq!(cmd(-100.0, 0.0, 0.0, PI / 2.0), [100, -50, -50]);
        // Pure rotation drives all wheels the same way
        let [a, b, c] = cmd(0.0, 0.0, 1.0, 0.0);
        assert!(a == b && b == c && a != 0);
        // Saturation scales all wheels together, keeping the ratio
        assert_eq!(cmd(0.0, 10000.0, 0.0, 0.0), [1000, -500, -500]);
    }

    // Wheel speed maps to duty through max_wheel_speed: 100 mm/s out of 2000 is 5 % duty
    #[test]
    fn motor_commands_scale_by_max_wheel_speed() {
        let mut control = unit_loop();
        control.params.max_wheel_speed = 2000.0;
        let cmd = y_error_command(&mut control, 100.0, Duration::from_ticks(0));
        assert_eq!(cmd, [50, -25, -25]);
    }

    // I term is capped: a 10 mm y error for 20 s integrates to 200, clamped to 100, so P 10 + I 0.1 * 100 = 20
    #[test]
    fn motor_commands_clamp_integral() {
        let cmd = y_error_command(&mut unit_loop(), 10.0, Duration::from_secs(20));
        assert_eq!(cmd, [20, -10, -10]);
    }

    // Outside INTEGRAL_ZONE the integral is reset: a 100 mm error for 20 s gives P alone
    #[test]
    fn motor_commands_skip_integral_far_from_target() {
        let mut control = unit_loop();
        y_error_command(&mut control, 10.0, Duration::from_secs(20));
        let cmd = y_error_command(&mut control, 100.0, Duration::from_secs(20));
        assert_eq!(cmd, [100, -50, -50]);
    }

    // Inside DEADZONE the error counts as zero and the integral is held, not grown
    #[test]
    fn motor_commands_ignore_tiny_errors() {
        assert_eq!(
            y_error_command(&mut unit_loop(), 0.5, Duration::from_secs(20)),
            [0, 0, 0]
        );
        let mut control = unit_loop();
        y_error_command(&mut control, 10.0, Duration::from_secs(2));
        let cmd = y_error_command(&mut control, 0.5, Duration::from_secs(20));
        assert_eq!(cmd, [2, -1, -1]);
    }

    // D term damps: moving +y at 1000 mm/s with no error (KD = 0.1) pushes back like a -100 mm y error
    #[test]
    fn motor_commands_damp_velocity() {
        let velocity = RobotVel {
            vy: 1000.0,
            ..Default::default()
        };
        let cmd = unit_loop().calculate_motor_commands(
            RobotPos::default(),
            velocity,
            0.0,
            Duration::from_ticks(0),
        );
        assert_eq!(cmd, [-100, 50, 50]);
    }
}
