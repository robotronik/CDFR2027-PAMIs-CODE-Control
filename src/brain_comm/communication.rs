use crate::board::SpiPins;
use crate::motor_control::{MotionCommandSender, RobotPosReceiver};
use crate::servo_control::ServoCommandSender;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::peripherals::SPI2;

pub struct BrainCommLinks {
    pub motion: MotionCommandSender,
    pub servo: ServoCommandSender,
    pub robot_pos: RobotPosReceiver,
}

#[embassy_executor::task]
async fn run_brain_comm(spi: SpiPins, spi2: SPI2<'static>, links: BrainCommLinks) {
    let _ = (&spi, &spi2, &links);

    loop {
        Timer::after(Duration::from_millis(100)).await;
        // TODO: SPI transfer, decode request, route it, prepare response
    }
}

pub struct BrainCommTask;

impl BrainCommTask {
    pub fn spawn(spawner: &Spawner, spi: SpiPins, spi2: SPI2<'static>, links: BrainCommLinks) {
        let token = run_brain_comm(spi, spi2, links)
            .expect("Failed to create BrainCommTask (token exhaustion)");
        spawner.spawn(token);
    }
}
