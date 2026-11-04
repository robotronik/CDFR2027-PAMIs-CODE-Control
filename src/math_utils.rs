use core::f32::consts::{PI, TAU};

// Radians, wrapped to [-PI, PI)
pub fn wrap_angle(angle: f32) -> f32 {
    let wrapped = (angle + PI) % TAU;
    if wrapped < 0.0 {
        wrapped + TAU - PI
    } else {
        wrapped - PI
    }
}
