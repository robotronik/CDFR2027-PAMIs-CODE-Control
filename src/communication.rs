use crate::board::SpiPins;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::SPI2;
use esp_println::println;

#[embassy_executor::task]
async fn run_brain_comm(spi: SpiPins, spi2: SPI2<'static>) {
    /*
     * SPI Communication task with the high-level BrainBoard.
     * Handles inbound command packets and outbound telemetry.
     */
    let _ = (&spi, &spi2);
    loop {
        Timer::after(Duration::from_millis(50)).await;
        // Periodic communication check
        println!("BrainComm task tick");
    }
}

pub struct BrainCommTask;

impl BrainCommTask {
    pub fn spawn(spawner: &Spawner, spi: SpiPins, spi2: SPI2<'static>) {
        let token = run_brain_comm(spi, spi2).expect("Failed to create BrainCommTask (token exhaustion)");
        spawner.spawn(token);
    }
}
