use super::protocol::{FRAME_LEN, Frame, Request, Status, decode, encode_reply};
use crate::board::SpiPins;
use crate::motor_control::{MotionCommand, MotionCommandSender, RobotPosReceiver};
use crate::servo_control::{ServoCommand, ServoCommandSender};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::Blocking;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{DMA_CH0, SPI2};
use esp_hal::spi::Mode;
use esp_hal::spi::slave::Spi;
use esp_hal::spi::slave::dma::SpiDma;
use esp_hal::{dma_rx_buffer, dma_tx_buffer};
use log::{debug, warn};

pub struct BrainCommLinks {
    pub motion: MotionCommandSender,
    pub servo: ServoCommandSender,
    pub robot_pos: RobotPosReceiver,
}

struct SpiLink {
    spi: SpiDma<'static, Blocking>,
    rx: DmaRxBuf,
    tx: DmaTxBuf,
    handshake: Output<'static>,
}

// One transaction: arm DMA with `reply`, raise handshake, wait for the Brain to clock it
// shortcut: reply is frozen once armed, so pos can be stale if the Brain is slow; re-arm on a timer if that matters
async fn exchange(link: SpiLink, reply: &Frame) -> (SpiLink, Frame) {
    let SpiLink { spi, mut rx, mut tx, mut handshake } = link;
    tx.set_length(FRAME_LEN);
    tx.as_mut_slice().copy_from_slice(reply);
    rx.set_length(FRAME_LEN);

    let transfer = match spi.transfer(FRAME_LEN, rx, FRAME_LEN, tx) {
        Ok(transfer) => transfer,
        Err((err, ..)) => panic!("BrainComm: cannot arm SPI transfer: {:?}", err),
    };
    handshake.set_high();
    // The slave driver has no async API, so poll instead of blocking Core 0
    while !transfer.is_done() {
        Timer::after_micros(100).await;
    }
    let (spi, (rx, tx)) = transfer.wait();
    handshake.set_low();

    let mut request = [0u8; FRAME_LEN];
    request.copy_from_slice(&rx.as_slice()[..FRAME_LEN]);
    (SpiLink { spi, rx, tx, handshake }, request)
}

fn route(request: Request, links: &BrainCommLinks) -> Status {
    let sent = match request {
        Request::Nop => return Status::Ack,
        Request::Stop => links.motion.try_send(MotionCommand::Stop).is_ok(),
        Request::TargetPos { x, y, theta } => {
            links.motion.try_send(MotionCommand::TargetPos { x, y, theta }).is_ok()
        }
        Request::SetServo { id, angle_deg } => links
            .servo
            .try_send(ServoCommand::SetAngle { id, angle_deg: angle_deg as f32 })
            .is_ok(),
    };
    if sent {
        Status::Ack
    } else {
        warn!("BrainComm: inbox full, dropping {:?}", request);
        Status::QueueFull
    }
}

#[embassy_executor::task]
async fn run_brain_comm(
    spi: SpiPins,
    spi2: SPI2<'static>,
    dma_channel: DMA_CH0<'static>,
    mut links: BrainCommLinks,
) {
    let spi_slave = Spi::new(spi2, Mode::_0)
        .with_sck(spi.sclk)
        .with_mosi(spi.mosi)
        .with_miso(spi.miso)
        .with_cs(spi.cs)
        .with_dma(dma_channel);
    let mut link = SpiLink {
        spi: spi_slave,
        rx: dma_rx_buffer!(FRAME_LEN).unwrap(),
        tx: dma_tx_buffer!(FRAME_LEN).unwrap(),
        handshake: Output::new(spi.handshake, Level::Low, OutputConfig::default()),
    };

    // Status of the previous command, sent back with the next reply
    let (mut status, mut seq) = (Status::Ack, 0);
    loop {
        let pos = links.robot_pos.try_get().unwrap_or_default();
        let request;
        (link, request) = exchange(link, &encode_reply(status, seq, &pos)).await;
        let result;
        (seq, result) = decode(&request);
        status = match result {
            Ok(request) => {
                debug!("BrainComm: seq {} {:?}", seq, request);
                route(request, &links)
            }
            Err(err) => {
                warn!("BrainComm: seq {} rejected: {:?}", seq, err);
                err
            }
        };
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
