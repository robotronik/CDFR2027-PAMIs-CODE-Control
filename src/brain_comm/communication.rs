use super::protocol::{FRAME_LEN, Frame, Request, Status, decode, encode_reply};
use crate::board::SpiPins;
use crate::motor_control::{MotionCommand, MotionCommandSender, RobotPosReceiver};
use crate::servo_control::{ServoCommand, ServoCommandSender};
use crate::utility::BatteryLowReceiver;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::Blocking;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
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
    pub battery_low: BatteryLowReceiver,
}

// No complete frame from the Brain for this long stops the motors
const BRAIN_TIMEOUT: Duration = Duration::from_millis(500);

struct SpiLink {
    spi: SpiDma<'static, Blocking>,
    rx: DmaRxBuf,
    tx: DmaTxBuf,
    handshake: Output<'static>,
}

// One transaction: arm DMA with `reply`, raise handshake, wait for the Brain to clock it.
// Stops the motors once if the Brain stays silent for BRAIN_TIMEOUT.
// shortcut: reply is frozen once armed, so pos can be stale if the Brain is slow; re-arm on a timer if that matters
async fn exchange(link: SpiLink, reply: &Frame, motion: &MotionCommandSender) -> (SpiLink, Frame) {
    let SpiLink { spi, rx, mut tx, mut handshake } = link;
    tx.fill(reply);
    let transfer = match spi.transfer(FRAME_LEN, rx, FRAME_LEN, tx) {
        Ok(transfer) => transfer,
        Err((err, ..)) => panic!("BrainComm: cannot arm SPI transfer: {:?}", err),
    };
    handshake.set_high();
    let armed_at = Instant::now();
    let mut stopped = false;
    // The slave driver has no async API, so poll instead of blocking Core 0
    while !transfer.is_done() {
        // Retried each poll while the inbox is full
        if !stopped && armed_at.elapsed() > BRAIN_TIMEOUT && motion.try_send(MotionCommand::Stop).is_ok() {
            warn!("BrainComm: Brain silent for {} ms, motors stopped", BRAIN_TIMEOUT.as_millis());
            stopped = true;
        }
        Timer::after_micros(100).await;
    }
    let (spi, (rx, tx)) = transfer.wait();
    handshake.set_low();

    let request = rx.as_slice()[..FRAME_LEN].try_into().unwrap();
    (SpiLink { spi, rx, tx, handshake }, request)
}

// Decodes a request and forwards it to its task; returns the seq and status for the next reply
pub fn handle(frame: &Frame, links: &BrainCommLinks) -> (u8, Status) {
    let request = match decode(frame) {
        (seq, Ok(request)) => {
            debug!("BrainComm: seq {} {:?}", seq, request);
            request
        }
        (seq, Err(err)) => {
            warn!("BrainComm: seq {} rejected: {:?}", seq, err);
            return (seq, err);
        }
    };
    let sent = match request {
        Request::Nop => true,
        Request::Stop => links.motion.try_send(MotionCommand::Stop).is_ok(),
        Request::TargetPos { x, y, theta } => {
            links.motion.try_send(MotionCommand::TargetPos { x, y, theta }).is_ok()
        }
        Request::SetServo { id, angle_deg } => links
            .servo
            .try_send(ServoCommand::SetAngle { id, angle_deg: angle_deg as f32 })
            .is_ok(),
    };
    if !sent {
        warn!("BrainComm: inbox full, dropping {:?}", request);
    }
    (frame[2], if sent { Status::Ack } else { Status::QueueFull })
}

#[embassy_executor::task]
async fn run_brain_comm(
    spi: SpiPins,
    spi2: SPI2<'static>,
    dma_channel: DMA_CH0<'static>,
    mut links: BrainCommLinks,
) {
    // Pull-up keeps CS high when no Brain is connected, otherwise the floating line clocks in noise
    let cs = Input::new(spi.cs, InputConfig::default().with_pull(Pull::Up));
    let spi_slave = Spi::new(spi2, Mode::_0)
        .with_sck(spi.sclk)
        .with_mosi(spi.mosi)
        .with_miso(spi.miso)
        .with_cs(cs.peripheral_input())
        .with_dma(dma_channel);
    let mut link = SpiLink {
        spi: spi_slave,
        rx: dma_rx_buffer!(FRAME_LEN).unwrap(),
        tx: dma_tx_buffer!(FRAME_LEN).unwrap(),
        handshake: Output::new(spi.handshake, Level::Low, OutputConfig::default()),
    };

    let (mut seq, mut status) = (0, Status::Ack);
    loop {
        let pos = links.robot_pos.try_get().unwrap_or_default();
        let battery_low = links.battery_low.try_get().unwrap_or(false);
        let reply = encode_reply(status, seq, &pos, battery_low);
        let (next_link, request) = exchange(link, &reply, &links.motion).await;
        link = next_link;
        (seq, status) = handle(&request, &links);
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
