// MotorControl outbox (latest value only). The watch instance lives in main.rs.

use crate::math_utils::wrap_angle;
use core::ops::Sub;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{self, Watch};

#[derive(Clone, Copy, Debug, Default)]
pub struct RobotPos {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
}

// Theta is wrapped so the error takes the shortest rotation
impl Sub for RobotPos {
    type Output = RobotPos;

    fn sub(self, rhs: RobotPos) -> RobotPos {
        RobotPos {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            theta: wrap_angle(self.theta - rhs.theta),
            vx: self.vx - rhs.vx,
            vy: self.vy - rhs.vy,
            omega: self.omega - rhs.omega,
        }
    }
}

pub const ROBOT_POS_RECEIVERS: usize = 2;

pub type RobotPosWatch = Watch<CriticalSectionRawMutex, RobotPos, ROBOT_POS_RECEIVERS>;
pub type RobotPosSender =
    watch::Sender<'static, CriticalSectionRawMutex, RobotPos, ROBOT_POS_RECEIVERS>;
pub type RobotPosReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, RobotPos, ROBOT_POS_RECEIVERS>;
