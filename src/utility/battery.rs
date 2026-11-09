// BatteryTask: samples the battery voltage and publishes a low-battery flag for BrainComm

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{self, Watch};
use embassy_time::{Duration, Ticker};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::peripherals::{ADC1, GPIO1};
use log::{debug, info, warn};

// 2S LiPo: 3.5 V per cell under load, recover at 3.6 V so motor current sag doesn't flap the flag
const LOW_MV: u32 = 7000;
const RECOVERED_MV: u32 = 7200;
// 30k/10k divider
const DIVIDER: u32 = 4;
const SAMPLES: u32 = 16;
const PERIOD: Duration = Duration::from_secs(1);

pub const BATTERY_LOW_RECEIVERS: usize = 1;
pub type BatteryLowWatch = Watch<CriticalSectionRawMutex, bool, BATTERY_LOW_RECEIVERS>;
pub type BatteryLowSender =
    watch::Sender<'static, CriticalSectionRawMutex, bool, BATTERY_LOW_RECEIVERS>;
pub type BatteryLowReceiver =
    watch::Receiver<'static, CriticalSectionRawMutex, bool, BATTERY_LOW_RECEIVERS>;

// Hysteresis between LOW_MV and RECOVERED_MV
pub fn battery_low(was_low: bool, battery_mv: u32) -> bool {
    battery_mv < if was_low { RECOVERED_MV } else { LOW_MV }
}

#[embassy_executor::task]
async fn run_battery_monitor(battery_pin: GPIO1<'static>, adc1: ADC1<'static>, low_sender: BatteryLowSender) {
    let mut config = AdcConfig::new();
    let mut pin = config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(battery_pin, Attenuation::_11dB);
    let mut adc = Adc::new(adc1, config).into_async();

    let mut low = false;
    let mut ticker = Ticker::every(PERIOD);
    loop {
        let mut pin_mv = 0;
        for _ in 0..SAMPLES {
            pin_mv += adc.read_oneshot(&mut pin).await as u32;
        }
        let battery_mv = pin_mv / SAMPLES * DIVIDER;
        debug!("Battery: {} mV", battery_mv);

        let now_low = battery_low(low, battery_mv);
        match (low, now_low) {
            (false, true) => warn!("Battery low: {} mV", battery_mv),
            (true, false) => info!("Battery recovered: {} mV", battery_mv),
            _ => {}
        }
        low = now_low;
        low_sender.send(low);

        ticker.next().await;
    }
}

pub struct BatteryTask;

impl BatteryTask {
    pub fn spawn(
        spawner: &Spawner,
        battery_pin: GPIO1<'static>,
        adc1: ADC1<'static>,
        low_sender: BatteryLowSender,
    ) {
        let token = run_battery_monitor(battery_pin, adc1, low_sender)
            .expect("Failed to create BatteryTask (token exhaustion)");
        spawner.spawn(token);
    }
}
