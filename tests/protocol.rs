// BrainComm frame codec and request routing tests, run on the board with `cargo hil`

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_sync::channel::Channel;
    use embassy_sync::watch::Watch;
    use pami_control::brain_comm::{
        BATTERY_LOW_FLAG, ODOMETRY_NOT_READY_FLAG, RESTARTED_FLAG, BrainCommLinks, FRAME_LEN, Frame, MAGIC, Request, Status, crc8, decode, encode_reply,
        handle,
    };
    use pami_control::motor_control::{
        MotionCommand, MotionCommandChannel, RobotPos, RobotPosWatch,
    };
    use pami_control::servo_control::{ServoCommand, ServoCommandChannel};
    use pami_control::utility::BatteryLowWatch;

    // The board resets between tests, so every test gets fresh inboxes
    static MOTION: MotionCommandChannel = Channel::new();
    static SERVO: ServoCommandChannel = Channel::new();
    static POS: RobotPosWatch = Watch::new();
    static BATTERY_LOW: BatteryLowWatch = Watch::new();

    fn links() -> BrainCommLinks {
        BrainCommLinks {
            motion: MOTION.sender(),
            servo: SERVO.sender(),
            robot_pos: POS.receiver().unwrap(),
            battery_low: BATTERY_LOW.receiver().unwrap(),
        }
    }

    fn f32s(values: [f32; 3]) -> [u8; 12] {
        let mut d = [0u8; 12];
        for (i, v) in values.iter().enumerate() {
            d[i * 4..i * 4 + 4].copy_from_slice(&v.to_le_bytes());
        }
        d
    }

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

    // CRC-8/SMBUS standard check value: "123456789" gives 0xF4
    #[test]
    fn crc8_check_value() {
        assert_eq!(crc8(b"123456789"), 0xF4);
    }

    // Nop and Stop frames decode to their request, data bytes ignored
    #[test]
    fn decode_nop_and_stop() {
        assert_eq!(decode(&frame(0x00, 7, &[])), Ok(Request::Nop));
        assert_eq!(decode(&frame(0x01, 8, &[])), Ok(Request::Stop));
    }

    // TargetPos decodes three little-endian f32s as x, y, theta
    #[test]
    fn decode_target_pos() {
        assert_eq!(
            decode(&frame(0x02, 1, &f32s([100.0, -50.5, 1.5]))),
            Ok(Request::TargetPos {
                x: 100.0,
                y: -50.5,
                theta: 1.5
            })
        );
    }

    // SetServo decodes id and angle, including the edge values id 0 and 180°
    #[test]
    fn decode_set_servo() {
        assert_eq!(
            decode(&frame(0x03, 2, &[3, 90])),
            Ok(Request::SetServo {
                id: 3,
                angle_deg: 90
            })
        );
        assert_eq!(
            decode(&frame(0x03, 2, &[0, 180])),
            Ok(Request::SetServo {
                id: 0,
                angle_deg: 180
            })
        );
    }

    // All-zero and all-0xFF frames (idle or floating bus) give BadMagic
    #[test]
    fn decode_rejects_garbage() {
        assert_eq!(decode(&[0x00; FRAME_LEN]), Err(Status::BadMagic));
        assert_eq!(decode(&[0xFF; FRAME_LEN]), Err(Status::BadMagic));
    }

    // A corrupted CRC gives BadCrc, and the reply still echoes the frame's seq
    #[test]
    fn handle_rejects_bad_crc_but_keeps_seq() {
        let mut f = frame(0x01, 42, &[]);
        f[15] ^= 0xFF;
        assert_eq!(handle(&f, &links()), (42, Status::BadCrc));
    }

    // Several errors at once report only the first one checked (magic)
    #[test]
    fn decode_reports_first_error_only() {
        let mut f = frame(0x7F, 0, &[]);
        f[0] = 0x00;
        f[15] ^= 0xFF;
        assert_eq!(decode(&f), Err(Status::BadMagic));
    }

    // An opcode not in the table gives UnknownOpcode
    #[test]
    fn decode_rejects_unknown_opcode() {
        assert_eq!(decode(&frame(0x7F, 0, &[])), Err(Status::UnknownOpcode));
    }

    // NaN/infinite floats, angle 181° and servo id 4 all give BadArgument
    #[test]
    fn decode_rejects_bad_arguments() {
        for bad in [
            [f32::NAN, 0.0, 0.0],
            [0.0, f32::INFINITY, 0.0],
            [0.0, 0.0, f32::NEG_INFINITY],
        ] {
            assert_eq!(
                decode(&frame(0x02, 0, &f32s(bad))),
                Err(Status::BadArgument)
            );
        }
        assert_eq!(
            decode(&frame(0x03, 0, &[0, 181])),
            Err(Status::BadArgument)
        );
        assert_eq!(
            decode(&frame(0x03, 0, &[4, 90])),
            Err(Status::BadArgument)
        );
    }

    // Reply bytes: magic, status, seq, x/y/theta as LE f32, then a valid CRC
    #[test]
    fn encode_reply_layout() {
        let pos = RobotPos {
            x: 1.0,
            y: 2.0,
            theta: -0.5,
            ..Default::default()
        };
        let f = encode_reply(Status::QueueFull, 0, 9, &pos);
        assert_eq!(&f[0..3], &[MAGIC, 5, 9]);
        assert_eq!(&f[3..7], &1.0f32.to_le_bytes());
        assert_eq!(&f[7..11], &2.0f32.to_le_bytes());
        assert_eq!(&f[11..15], &(-0.5f32).to_le_bytes());
        assert_eq!(f[15], crc8(&f[..15]));
    }

    // Flags land in bits 5-7 of the status byte, the code stays readable with & 0x1F, CRC still valid
    #[test]
    fn encode_reply_ors_flags_into_status() {
        let flags = BATTERY_LOW_FLAG | ODOMETRY_NOT_READY_FLAG | RESTARTED_FLAG;
        let f = encode_reply(Status::QueueFull, flags, 1, &RobotPos::default());
        assert_eq!(f[1], 0xE0 | Status::QueueFull as u8);
        assert_eq!(f[1] & 0x1F, Status::QueueFull as u8);
        assert_eq!(f[15], crc8(&f[..15]));
    }

    // Nop is acknowledged and sends nothing to either task
    #[test]
    fn handle_nop_queues_nothing() {
        assert_eq!(handle(&frame(0x00, 3, &[]), &links()), (3, Status::Ack));
        assert!(MOTION.is_empty() && SERVO.is_empty());
    }

    // Stop and TargetPos are acknowledged and reach the motion inbox only
    #[test]
    fn handle_routes_motion_commands() {
        let links = links();
        assert_eq!(handle(&frame(0x01, 4, &[]), &links), (4, Status::Ack));
        assert!(matches!(MOTION.try_receive(), Ok(MotionCommand::Stop)));

        assert_eq!(
            handle(&frame(0x02, 5, &f32s([10.0, -20.0, 0.5])), &links),
            (5, Status::Ack)
        );
        assert!(matches!(
            MOTION.try_receive(),
            Ok(MotionCommand::TargetPos {
                x: 10.0,
                y: -20.0,
                theta: 0.5
            })
        ));
        assert!(SERVO.is_empty());
    }

    // SetServo is acknowledged and reaches the servo inbox only
    #[test]
    fn handle_routes_servo_commands() {
        assert_eq!(
            handle(&frame(0x03, 6, &[2, 45]), &links()),
            (6, Status::Ack)
        );
        assert!(matches!(
            SERVO.try_receive(),
            Ok(ServoCommand::SetAngle {
                id: 2,
                angle_deg: 45
            })
        ));
        assert!(MOTION.is_empty());
    }

    // An invalid frame is reported and nothing is queued
    #[test]
    fn handle_drops_invalid_requests() {
        let mut f = frame(0x01, 7, &[]);
        f[15] ^= 0xFF;
        assert_eq!(handle(&f, &links()), (7, Status::BadCrc));
        assert!(MOTION.is_empty());
    }

    // A full motion inbox gives QueueFull, servo commands still go through
    #[test]
    fn handle_reports_full_inbox() {
        while !MOTION.is_full() {
            MOTION.try_send(MotionCommand::Stop).unwrap();
        }
        let links = links();
        assert_eq!(handle(&frame(0x01, 8, &[]), &links), (8, Status::QueueFull));
        assert_eq!(handle(&frame(0x03, 9, &[0, 90]), &links), (9, Status::Ack));
    }
}
