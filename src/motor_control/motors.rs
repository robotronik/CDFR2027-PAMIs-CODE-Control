use crate::board::DcMotorPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_println::println;

#[embassy_executor::task]
async fn run_motor_control(mut motors: [DcMotorPins; 3]) {
    /*
     * Drivetrain motor control loop for 3 DC motors via DRV8871 H-bridges.
     * Manages wheel velocity, direction, and trajectory tracking.
     */
    loop {
        Timer::after(Duration::from_millis(20)).await;
        // Periodic motion loop tick (50Hz)
        let _ = &mut motors;
        println!("MotorControl task tick");
    }
}

pub struct MotorControlTask;

impl MotorControlTask {
    pub fn spawn(spawner: &Spawner, motors: [DcMotorPins; 3]) {
        let token = run_motor_control(motors).expect("Failed to create MotorControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
