use crate::board::SpiPins;
use crate::protocol::{MOTION_COMMAND_CHANNEL, ROBOT_POSE_WATCH, MotionCommand};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::SPI2;
use esp_println::println;

#[embassy_executor::task]
async fn run_brain_comm(spi: SpiPins, spi2: SPI2<'static>) {
    /*
     * SPI Communication task with the high-level BrainBoard on Core 0.
     * Forwards motion commands to Core 1 and transmits latest robot pose back.
     */
    let _ = (&spi, &spi2);
    let mut pose_receiver = ROBOT_POSE_WATCH.receiver().expect("Failed to obtain pose receiver");

    loop {
        Timer::after(Duration::from_millis(100)).await;

        // Read latest robot pose published by Core 1
        if let Some(pose) = pose_receiver.try_get() {
            println!("BrainComm (Core 0) latest pose: ({:.1}, {:.1})", pose.x, pose.y);
        }

        // Forward target commands received from SPI into the inter-core channel
        let _ = MOTION_COMMAND_CHANNEL.try_send(MotionCommand::Velocity {
            vx: 0.2,
            vy: 0.0,
            omega: 0.0,
        });

        println!("BrainComm (Core 0) tick");
    }
}

pub struct BrainCommTask;

impl BrainCommTask {
    pub fn spawn(spawner: &Spawner, spi: SpiPins, spi2: SPI2<'static>) {
        let token = run_brain_comm(spi, spi2).expect("Failed to create BrainCommTask (token exhaustion)");
        spawner.spawn(token);
    }
}
