// OTOS tests, run on the board with `cargo hil`. `otos_*` tests need the sensor on connector I2C1.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::f32::consts::PI;
    use pami_control::board::Board;
    use pami_control::motor_control::{Otos, decode_pose};

    fn assert_close(actual: f32, expected: f32) {
        assert!((actual - expected).abs() < 1e-3, "{} != {}", actual, expected);
    }

    fn raw(values: [i16; 6]) -> [u8; 12] {
        let mut raw = [0u8; 12];
        for (i, v) in values.iter().enumerate() {
            raw[i * 2..i * 2 + 2].copy_from_slice(&v.to_le_bytes());
        }
        raw
    }

    #[init]
    fn init() -> Board {
        rtt_target::rtt_init_log!();
        Board::init()
    }

    // All-zero registers decode to a zero pose
    #[test]
    fn decode_pose_zero() {
        let pos = decode_pose(&[0; 12]);
        assert_eq!((pos.x, pos.y, pos.theta, pos.vx, pos.vy, pos.omega), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    }

    // Most negative raw values decode to the sensor limits: -10 m, -pi, -5 m/s, -2000 °/s
    #[test]
    fn decode_pose_full_scale() {
        let pos = decode_pose(&raw([i16::MIN; 6]));
        assert_close(pos.x, -10_000.0);
        assert_close(pos.y, -10_000.0);
        assert_close(pos.theta, -PI);
        assert_close(pos.vx, -5_000.0);
        assert_close(pos.vy, -5_000.0);
        assert_close(pos.omega, -2000.0 * PI / 180.0);
    }

    // Mixed-sign values land in the right fields with the right units
    #[test]
    fn decode_pose_field_order_and_sign() {
        // 1 m, -0.5 m, +90°, 1 m/s, -1 m/s, -180 °/s
        let pos = decode_pose(&raw([3277, -1638, 16384, 6554, -6554, -2949]));
        assert!((pos.x - 1000.0).abs() < 0.5);
        assert!((pos.y + 500.0).abs() < 0.5);
        assert_close(pos.theta, PI / 2.0);
        assert!((pos.vx - 1000.0).abs() < 0.2);
        assert!((pos.vy + 1000.0).abs() < 0.2);
        assert!((pos.omega + PI).abs() < 1e-3);
    }

    // Needs the OTOS plugged in: after init, a robot at rest reads (0, 0, 0) within 5 mm and ~1°
    #[test]
    #[timeout(10)]
    async fn otos_init_then_read_near_origin(board: Board) {
        let mut otos = Otos::new(board.i2c, board.peripherals.i2c0);
        otos.init().await;
        let pos = otos.read_pos().await.expect("OTOS read failed");
        // Robot still on the bench: tracking was just reset
        assert!(pos.x.abs() < 5.0 && pos.y.abs() < 5.0 && pos.theta.abs() < 0.02, "{:?}", pos);
    }
}
