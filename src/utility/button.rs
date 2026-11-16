use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::gpio::Input;
use log::info;

// Does nothing for now beyond logging presses; meant to become match start or mode switching
#[embassy_executor::task]
async fn run_button(mut button: Input<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        info!("Boot button pressed!");
        // Debounce press and release: bounce on either edge would log one press several times
        Timer::after_millis(50).await;
        button.wait_for_high().await;
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
