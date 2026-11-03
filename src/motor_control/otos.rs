use crate::board::I2cPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::I2C0;
use esp_println::println;

#[embassy_executor::task]
async fn run_otos(i2c: I2cPins, i2c0: I2C0<'static>) {
    /*
     * Optical Tracking Odometry Sensor (OTOS) task on I2C0.
     * Periodically polls position and orientation updates.
     */
    let _ = (&i2c, &i2c0);
    loop {
        Timer::after(Duration::from_millis(50)).await;
        // Periodic odometry poll tick
        println!("OTOS task tick");
    }
}

pub struct OtosTask;

impl OtosTask {
    pub fn spawn(spawner: &Spawner, i2c: I2cPins, i2c0: I2C0<'static>) {
        let token = run_otos(i2c, i2c0).expect("Failed to create OtosTask (token exhaustion)");
        spawner.spawn(token);
    }
}
