// OTOS odometry sensor on I2C0, polled by MotorControl every tick

use super::robot_state::RobotPos;
use crate::board::I2cPins;
use esp_hal::peripherals::I2C0;

pub struct Otos {
    pins: I2cPins,
    i2c0: I2C0<'static>,
}

impl Otos {
    // Call on Core 1: async esp-hal drivers bind their interrupt to the creating core
    pub fn new(pins: I2cPins, i2c0: I2C0<'static>) -> Self {
        Self { pins, i2c0 }
    }

    // Position and velocity
    pub fn read_pos(&mut self) -> RobotPos {
        let _ = (&mut self.pins, &mut self.i2c0);
        // TODO: I2C read
        RobotPos::default()
    }
}
