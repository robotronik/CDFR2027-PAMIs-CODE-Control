// Pure maths tests, run on the board with `cargo hil`

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::f32::consts::{PI, TAU};
    use embassy_time::Duration;
    use pami_control::motor_control::{ControlLoop, RobotPos, RobotVel, motor_duty, wrap_angle};
    use pami_control::servo_control::servo_duty;
    use pami_control::utility::battery_low;

    fn assert_close(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 1e-5,
            "{} != {}",
            actual,
            expected
        );
    }

    // Started like main.rs so hardware tests can be added to this suite
    #[init]
    fn init() {
        rtt_target::rtt_init_log!();
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    }

    // Angles already in [-pi, pi) come back unchanged
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

    // Extra full turns are removed, in both directions
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

    // Positive speed drives IN1, negative drives IN2, zero leaves both at 0
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

    // 0/90/180° give 500/1500/2500 us pulses, angles above 180° clamp to 180°
    #[test]
    fn servo_duty_maps_angle_to_pulse_and_clamps() {
        // 14-bit duty over a 20 ms period: 500 us -> 409, 1500 us -> 1228, 2500 us -> 2048
        assert_eq!(servo_duty(0), 409);
        assert_eq!(servo_duty(90), 1228);
        assert_eq!(servo_duty(180), 2048);
        assert_eq!(servo_duty(200), servo_duty(180));
    }

    // Zero dt and velocity leave only P (KP = 1): wheels at 0/120/240° split a +y error 1 : -1/2 : -1/2
    #[test]
    fn motor_commands_project_error_on_wheels() {
        let cmd = |x, y, theta, heading| {
            ControlLoop::default().calculate_motor_commands(
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

    // D term damps: moving +y at 1000 mm/s with no error (KD = 0.1) pushes back like a -100 mm y error
    #[test]
    fn motor_commands_damp_velocity() {
        let velocity = RobotVel {
            vy: 1000.0,
            ..Default::default()
        };
        let cmd = ControlLoop::default().calculate_motor_commands(
            RobotPos::default(),
            velocity,
            0.0,
            Duration::from_ticks(0),
        );
        assert_eq!(cmd, [-100, 50, 50]);
    }

    // Goes low below 7000 mV, only recovers at 7200 mV or more
    #[test]
    fn battery_low_has_hysteresis() {
        assert!(!battery_low(false, 7000));
        assert!(battery_low(false, 6999));
        assert!(battery_low(true, 7199));
        assert!(!battery_low(true, 7200));
    }
}
