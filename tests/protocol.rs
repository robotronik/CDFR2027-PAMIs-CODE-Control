// BrainComm frame codec tests, run on the board with `cargo hil`

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use pami_control::brain_comm::{
        FRAME_LEN, Frame, MAGIC, Request, Status, crc8, decode, encode_reply,
    };
    use pami_control::motor_control::RobotPos;

    fn frame(opcode: u8, seq: u8, data: &[u8]) -> Frame {
        let mut f = [0u8; FRAME_LEN];
        f[0] = MAGIC;
        f[1] = opcode;
        f[2] = seq;
        f[3..3 + data.len()].copy_from_slice(data);
        f[15] = crc8(&f[..15]);
        f
    }

    #[init]
    fn init() {
        rtt_target::rtt_init_log!();
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    }

    #[test]
    fn crc8_check_value() {
        assert_eq!(crc8(b"123456789"), 0xF4);
    }

    #[test]
    fn decode_nop_and_stop() {
        assert_eq!(decode(&frame(0x00, 7, &[])), (7, Ok(Request::Nop)));
        assert_eq!(decode(&frame(0x01, 8, &[])), (8, Ok(Request::Stop)));
    }

    #[test]
    fn decode_target_pos() {
        let mut d = [0u8; 12];
        d[0..4].copy_from_slice(&100.0f32.to_le_bytes());
        d[4..8].copy_from_slice(&(-50.5f32).to_le_bytes());
        d[8..12].copy_from_slice(&1.5f32.to_le_bytes());
        assert_eq!(
            decode(&frame(0x02, 1, &d)),
            (1, Ok(Request::TargetPos { x: 100.0, y: -50.5, theta: 1.5 }))
        );
    }

    #[test]
    fn decode_set_servo() {
        assert_eq!(
            decode(&frame(0x03, 2, &[3, 90])),
            (2, Ok(Request::SetServo { id: 3, angle_deg: 90 }))
        );
    }

    #[test]
    fn decode_rejects_garbage() {
        assert_eq!(decode(&[0x00; FRAME_LEN]).1, Err(Status::BadMagic));
        assert_eq!(decode(&[0xFF; FRAME_LEN]).1, Err(Status::BadMagic));
    }

    #[test]
    fn decode_rejects_bad_crc_but_keeps_seq() {
        let mut f = frame(0x01, 42, &[]);
        f[15] ^= 0xFF;
        assert_eq!(decode(&f), (42, Err(Status::BadCrc)));
    }

    #[test]
    fn decode_rejects_unknown_opcode() {
        assert_eq!(decode(&frame(0x7F, 0, &[])).1, Err(Status::UnknownOpcode));
    }

    #[test]
    fn decode_rejects_bad_arguments() {
        let mut d = [0u8; 12];
        d[0..4].copy_from_slice(&f32::NAN.to_le_bytes());
        assert_eq!(decode(&frame(0x02, 0, &d)).1, Err(Status::BadArgument));
        assert_eq!(decode(&frame(0x03, 0, &[0, 181])).1, Err(Status::BadArgument));
    }

    #[test]
    fn encode_reply_layout() {
        let pos = RobotPos { x: 1.0, y: 2.0, theta: -0.5, ..Default::default() };
        let f = encode_reply(Status::QueueFull, 9, &pos);
        assert_eq!(&f[0..3], &[MAGIC, 5, 9]);
        assert_eq!(&f[3..7], &1.0f32.to_le_bytes());
        assert_eq!(&f[7..11], &2.0f32.to_le_bytes());
        assert_eq!(&f[11..15], &(-0.5f32).to_le_bytes());
        assert_eq!(f[15], crc8(&f[..15]));
    }
}
