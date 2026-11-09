use crate::board::SpiPins;
use crate::motor_control::{MotionCommandSender, RobotPosReceiver};
use crate::servo_control::ServoCommandSender;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{DMA_CH0, SPI2};
use esp_hal::spi::Mode;
use esp_hal::spi::slave::Spi;
use esp_hal::{dma_rx_buffer, dma_tx_buffer};

pub struct BrainCommLinks {
    pub motion: MotionCommandSender,
    pub servo: ServoCommandSender,
    pub robot_pos: RobotPosReceiver,
}

#[embassy_executor::task]
async fn run_brain_comm(
    spi: SpiPins,
    spi2: SPI2<'static>,
    dma_channel: DMA_CH0<'static>,
    links: BrainCommLinks,
) {
    // shortcut: 64-byte frames until protocol.rs defines the real frame size
    let dma_rx_buf = dma_rx_buffer!(64).unwrap();
    let dma_tx_buf = dma_tx_buffer!(64).unwrap();
    let spi_slave = Spi::new(spi2, Mode::_0)
        .with_sck(spi.sclk)
        .with_mosi(spi.mosi)
        .with_miso(spi.miso)
        .with_cs(spi.cs)
        .with_dma(dma_channel);

    let handshake = Output::new(spi.handshake, Level::Low, OutputConfig::default());
    let _ = (&spi_slave, &handshake, &links, &dma_rx_buf, &dma_tx_buf);

    loop {
        Timer::after(Duration::from_millis(100)).await;
        // TODO: SPI transfer, decode request, route it, prepare response
    }
}

pub struct BrainCommTask;

impl BrainCommTask {
    pub fn spawn(
        spawner: &Spawner,
        spi: SpiPins,
        spi2: SPI2<'static>,
        dma_channel: DMA_CH0<'static>,
        links: BrainCommLinks,
    ) {
        let token = run_brain_comm(spi, spi2, dma_channel, links)
            .expect("Failed to create BrainCommTask (token exhaustion)");
        spawner.spawn(token);
    }
}
