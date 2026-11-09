use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::gpio::Input;
use log::info;

#[embassy_executor::task]
async fn run_button(mut button: Input<'static>) {
    // Intended for match start or mode switching
    loop {
        button.wait_for_falling_edge().await;
        info!("Boot button pressed!");
        // Debounce: contact bounce would log one press several times
        Timer::after_millis(50).await;
    }
}

pub struct ButtonTask;

impl ButtonTask {
    pub fn spawn(spawner: &Spawner, button: Input<'static>) {
        let token = run_button(button).expect("Failed to create ButtonTask (token exhaustion)");
        spawner.spawn(token);
    }
}
