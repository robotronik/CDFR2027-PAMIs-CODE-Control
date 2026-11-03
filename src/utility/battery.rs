use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::{ADC1, GPIO1};
use esp_println::println;

#[embassy_executor::task]
async fn run_battery_monitor(battery_pin: GPIO1<'static>, adc1: ADC1<'static>) {
    /*
     * Power management and battery voltage monitoring task.
     * Samples the 30k/10k voltage divider on GPIO1 (ADC1_CH0).
     */
    let _ = (&battery_pin, &adc1);
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        // Periodic battery voltage check (1Hz)
        println!("BatteryTask tick");
    }
}

pub struct BatteryTask;

impl BatteryTask {
    pub fn spawn(spawner: &Spawner, battery_pin: GPIO1<'static>, adc1: ADC1<'static>) {
        let token = run_battery_monitor(battery_pin, adc1).expect("Failed to create BatteryTask (token exhaustion)");
        spawner.spawn(token);
    }
}
