use super::servo_commands::{ServoCommand, ServoCommandReceiver};
use embassy_executor::Spawner;
use esp_hal::gpio::{DriveMode, Output};
use esp_hal::ledc::channel::{self, ChannelHW, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::peripherals::LEDC;
use esp_hal::time::Rate;
use log::{debug, warn};

pub const SERVO_COUNT: usize = 4;
const PERIOD_US: f32 = 20_000.0;
const DUTY_MAX: f32 = (1 << 14) as f32;
// shortcut: generic 500-2500 us range for 0-180°, calibrate per servo model once mounted
const PULSE_MIN_US: f32 = 500.0;
const PULSE_MAX_US: f32 = 2500.0;

// 14-bit LEDC duty for a 50 Hz servo pulse; out of range angles are clamped
pub fn servo_duty(angle_deg: f32) -> u32 {
    let pulse_us = PULSE_MIN_US + angle_deg.clamp(0.0, 180.0) / 180.0 * (PULSE_MAX_US - PULSE_MIN_US);
    (pulse_us / PERIOD_US * DUTY_MAX) as u32
}

#[embassy_executor::task]
async fn run_servo_control(
    servos: [Output<'static>; SERVO_COUNT],
    ledc: LEDC<'static>,
    command_receiver: ServoCommandReceiver,
) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut pwm_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    // No pulse until the first command: servos stay limp at boot
    let make_channel = |number, pin| {
        let mut channel = ledc.channel(number, pin);
        channel
            .configure(channel::config::Config { timer: &pwm_timer, duty_pct: 0, drive_mode: DriveMode::PushPull })
            .unwrap();
        channel
    };
    let [s0, s1, s2, s3] = servos;
    let channels = [
        make_channel(channel::Number::Channel0, s0),
        make_channel(channel::Number::Channel1, s1),
        make_channel(channel::Number::Channel2, s2),
        make_channel(channel::Number::Channel3, s3),
    ];

    loop {
        match command_receiver.receive().await {
            ServoCommand::SetAngle { id, angle_deg } => {
                let Some(channel) = channels.get(id as usize) else {
                    warn!("ServoControl: ignoring command for unknown servo {}", id);
                    continue;
                };
                channel.set_duty_hw(servo_duty(angle_deg));
                debug!("ServoControl: servo {} -> {:.1} deg", id, angle_deg);
            }
        }
    }
}

pub struct ServoControlTask;

impl ServoControlTask {
    pub fn spawn(
        spawner: &Spawner,
        servos: [Output<'static>; SERVO_COUNT],
        ledc: LEDC<'static>,
        command_receiver: ServoCommandReceiver,
    ) {
        let token = run_servo_control(servos, ledc, command_receiver)
            .expect("Failed to create ServoControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
