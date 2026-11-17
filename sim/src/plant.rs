use crate::control_law::{PWM_PERIOD, WHEEL_ANGLES};
use crate::robot_state::{RobotPos, RobotVel};

pub struct Plant {
    // Wheel surface speed at full duty, in mm/s
    pub max_speed: f32,
    // Motor time constant, in s
    pub tau: f32,
    // Duty fraction below which the motor doesn't turn
    pub deadband: f32,
    // Robot centre to wheel contact, in mm
    pub wheel_distance: f32,
    pub pos: RobotPos,
    wheel_speeds: [f32; 3],
}

impl Plant {
    pub fn new(
        max_speed: f32,
        tau: f32,
        deadband: f32,
        wheel_distance: f32,
        pos: RobotPos,
    ) -> Self {
        Self {
            max_speed,
            tau,
            deadband,
            wheel_distance,
            pos,
            wheel_speeds: [0.0; 3],
        }
    }

    pub fn velocity(&self) -> RobotVel {
        forward_kinematics(self.wheel_speeds, self.pos.theta, self.wheel_distance)
    }

    // shortcut: wheels lag independently (no shared robot inertia), upgrade to a rigid-body model if tuning on hardware disagrees
    pub fn step(&mut self, commands: [i16; 3], dt: f32) {
        let vel = self.velocity();
        self.pos.x += vel.vx * dt;
        self.pos.y += vel.vy * dt;
        self.pos.theta += vel.omega * dt;
        for (speed, command) in self.wheel_speeds.iter_mut().zip(commands) {
            let u = command as f32 / PWM_PERIOD as f32;
            let u_eff = if u.abs() < self.deadband { 0.0 } else { u };
            *speed += (u_eff * self.max_speed - *speed) / self.tau * dt;
        }
    }
}

// Inverse of the controller's projection: wheel_i = -sin(phi_i) vx + cos(phi_i) vy + R omega
pub fn forward_kinematics(wheel_speeds: [f32; 3], theta: f32, wheel_distance: f32) -> RobotVel {
    let mut vel = RobotVel::default();
    for (wheel_angle, v) in WHEEL_ANGLES.iter().zip(wheel_speeds) {
        let phi = wheel_angle + theta;
        vel.vx += -phi.sin() * v * 2.0 / 3.0;
        vel.vy += phi.cos() * v * 2.0 / 3.0;
        vel.omega += v / (3.0 * wheel_distance);
    }
    vel
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control_law::{CONTROL_PERIOD, ControlLoop, ControlParams};

    #[test]
    fn equal_wheel_speeds_rotate_in_place() {
        let vel = forward_kinematics([200.0; 3], 0.4, 80.0);
        assert!(vel.vx.abs() < 1e-3 && vel.vy.abs() < 1e-3);
        assert!((vel.omega - 200.0 / 80.0).abs() < 1e-5);
    }

    #[test]
    fn controller_projection_round_trips() {
        let mut control = ControlLoop::default();
        control.params = ControlParams {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            // Duty then equals wheel speed in mm/s
            max_wheel_speed: PWM_PERIOD as f32,
            ..ControlParams::DEFAULT
        };
        let error = RobotPos {
            x: 100.0,
            y: 0.0,
            theta: 0.0,
        };
        let commands =
            control.calculate_motor_commands(error, RobotVel::default(), 0.0, CONTROL_PERIOD);
        let vel = forward_kinematics(commands.map(f32::from), 0.0, control.params.wheel_distance);
        // Commands are rounded to whole duty units
        assert!((vel.vx - 100.0).abs() < 1.0, "vx {}", vel.vx);
        assert!(vel.vy.abs() < 1.0 && vel.omega.abs() < 1e-3, "{vel:?}");
    }
}
