use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::Input;
use log::info;

#[embassy_executor::task]
async fn run_button(button: Input<'static>) {
    // Intended for match start or mode switching
    loop {
        Timer::after(Duration::from_millis(100)).await;
        if button.is_low() {
            info!("Boot button pressed!");
        }
    }
}

pub struct ButtonTask;

impl ButtonTask {
    pub fn spawn(spawner: &Spawner, button: Input<'static>) {
        let token = run_button(button).expect("Failed to create ButtonTask (token exhaustion)");
        spawner.spawn(token);
    }
}
