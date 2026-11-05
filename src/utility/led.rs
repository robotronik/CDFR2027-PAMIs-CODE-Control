use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Output;

#[embassy_executor::task]
async fn run_led(mut led: Output<'static>, interval_ms: u64) {
    loop {
        led.toggle();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

pub struct LedTask;

impl LedTask {
    pub fn spawn(spawner: &Spawner, led: Output<'static>, interval_ms: u64) {
        let token = run_led(led, interval_ms).expect("Failed to create LedTask (token exhaustion)");
        spawner.spawn(token);
    }
}
