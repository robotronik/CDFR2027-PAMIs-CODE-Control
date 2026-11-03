use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;
use esp_println::println;

#[embassy_executor::task]
async fn run_servo_control(mut servos: [Output<'static>; 4]) {
    /*
     * Actuator servo control loop.
     * Manages position sequencing and timing for the 4 robot servos.
     */
    loop {
        Timer::after(Duration::from_millis(50)).await;
        // Periodic servo loop tick (20Hz)
        let _ = &mut servos;
        println!("ServoControl task tick");
    }
}

pub struct ServoControlTask;

impl ServoControlTask {
    pub fn spawn(spawner: &Spawner, servos: [Output<'static>; 4]) {
        let token = run_servo_control(servos).expect("Failed to create ServoControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
