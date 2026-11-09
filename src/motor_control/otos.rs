// OTOS odometry sensor on I2C0, polled by MotorControl every tick.
// Robotronik board running the SparkFun OTOS firmware, see
// https://github.com/robotronik/Optical_Tracking_Odometry_Sensor (Firmware/SFE/Inc/registers.h)

use super::robot_state::RobotPos;
use crate::board::I2cPins;
use core::f32::consts::PI;
use embassy_time::Timer;
use esp_hal::Async;
use esp_hal::i2c::master::{Config, Error, I2c};
use esp_hal::peripherals::I2C0;
use esp_hal::time::Rate;
use log::{info, warn};

const ADDRESS: u8 = 0x17;
const PRODUCT_ID: u8 = 0x5F;

const REG_PRODUCT_ID: u8 = 0x00;
const REG_IMU_CALIBRATION: u8 = 0x06;
const REG_RESET: u8 = 0x07;
// Position x, y, h then velocity x, y, h: six i16 LE
const REG_POSITION: u8 = 0x20;

// ~2.4 ms per sample, so ~0.6 s with the robot still
const IMU_CALIBRATION_SAMPLES: u8 = 255;
const RESET_TRACKING: u8 = 0x01;

// i16 full scale: ±10 m, ±180°, ±5 m/s, ±2000 °/s
const MM_PER_LSB: f32 = 10_000.0 / 32768.0;
const RAD_PER_LSB: f32 = PI / 32768.0;
const MM_S_PER_LSB: f32 = 5_000.0 / 32768.0;
const RAD_S_PER_LSB: f32 = 2000.0 * PI / 180.0 / 32768.0;

// World frame: x forward and y left at reset, theta counter-clockwise
pub fn decode_pose(raw: &[u8; 12]) -> RobotPos {
    let at = |i: usize| i16::from_le_bytes([raw[i], raw[i + 1]]) as f32;
    RobotPos {
        x: at(0) * MM_PER_LSB,
        y: at(2) * MM_PER_LSB,
        theta: at(4) * RAD_PER_LSB,
        vx: at(6) * MM_S_PER_LSB,
        vy: at(8) * MM_S_PER_LSB,
        omega: at(10) * RAD_S_PER_LSB,
    }
}

pub struct Otos {
    i2c: I2c<'static, Async>,
}

impl Otos {
    // Call on Core 1: async esp-hal drivers bind their interrupt to the creating core
    pub fn new(pins: I2cPins, i2c0: I2C0<'static>) -> Self {
        let config = Config::default().with_frequency(Rate::from_khz(400));
        let i2c = I2c::new(i2c0, config)
            .unwrap()
            .with_sda(pins.sda)
            .with_scl(pins.scl)
            .into_async();
        Self { i2c }
    }

    // Waits for the sensor, calibrates the IMU and zeroes the pose. Robot must stay still.
    // Mounting offset and scale registers keep their 0 defaults: sensor is centered, uncalibrated.
    pub async fn init(&mut self) {
        loop {
            match self.read_reg(REG_PRODUCT_ID).await {
                Ok(PRODUCT_ID) => break,
                Ok(id) => warn!("OTOS: unexpected product id {:#04x}, retrying", id),
                Err(err) => warn!("OTOS: not responding ({:?}), retrying", err),
            }
            Timer::after_millis(500).await;
        }

        while let Err(err) = self.calibrate_and_reset().await {
            warn!("OTOS: init failed ({:?}), retrying", err);
            Timer::after_millis(500).await;
        }
        info!("OTOS: ready");
    }

    async fn calibrate_and_reset(&mut self) -> Result<(), Error> {
        self.write_reg(REG_IMU_CALIBRATION, IMU_CALIBRATION_SAMPLES).await?;
        // Bounded: if the IMU stops answering, the firmware aborts with the counter stuck above 0
        for _ in 0..200 {
            if self.read_reg(REG_IMU_CALIBRATION).await? == 0 {
                return self.write_reg(REG_RESET, RESET_TRACKING).await;
            }
            Timer::after_millis(10).await;
        }
        warn!("OTOS: IMU calibration did not finish, heading may drift");
        self.write_reg(REG_RESET, RESET_TRACKING).await
    }

    pub async fn read_pos(&mut self) -> Result<RobotPos, Error> {
        let mut raw = [0u8; 12];
        self.i2c.write_read_async(ADDRESS, &[REG_POSITION], &mut raw).await?;
        Ok(decode_pose(&raw))
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
        let mut value = [0u8];
        self.i2c.write_read_async(ADDRESS, &[reg], &mut value).await?;
        Ok(value[0])
    }

    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), Error> {
        self.i2c.write_async(ADDRESS, &[reg, value]).await
    }
}
