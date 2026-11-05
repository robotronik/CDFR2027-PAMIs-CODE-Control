// Pure maths tests, run on the board with `cargo hil`

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::f32::consts::{PI, TAU};
    use pami_control::math_utils::wrap_angle;
    use pami_control::motor_control::{RobotPos, motor_duty};

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
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    }

    #[test]
    fn wrap_angle_keeps_in_range_values() {
        assert_close(wrap_angle(0.0), 0.0);
        assert_close(wrap_angle(-0.5), -0.5);
    }

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

    #[test]
    fn robot_pos_sub_takes_shortest_rotation() {
        let a = RobotPos {
            x: 1.0,
            y: 2.0,
            theta: 3.0,
            vx: 0.5,
            vy: 0.0,
            omega: 1.0,
        };
        let b = RobotPos {
            x: 0.5,
            y: 3.0,
            theta: -3.0,
            vx: 0.0,
            vy: 0.5,
            omega: 0.0,
        };
        let d = a - b;
        assert_close(d.x, 0.5);
        assert_close(d.y, -1.0);
        assert_close(d.theta, 6.0 - TAU);
        assert_close(d.vx, 0.5);
        assert_close(d.vy, -0.5);
        assert_close(d.omega, 1.0);
    }

    #[test]
    fn motor_duty_drives_one_input_by_sign() {
        assert_eq!(motor_duty(0), (0, 0));
        assert_eq!(motor_duty(500), (500, 0));
        assert_eq!(motor_duty(-500), (0, 500));
    }

    #[test]
    fn motor_duty_clamps_to_full_duty() {
        assert_eq!(motor_duty(2000), (1000, 0));
        assert_eq!(motor_duty(i16::MIN), (0, 1000));
    }
}
