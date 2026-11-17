// Maths tied to HAL modules, run on the board with `cargo hil` (HAL-free maths lives in host-tests/)

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use pami_control::servo_control::servo_duty;
    use pami_control::utility::battery_low;

    // Started like main.rs so hardware tests can be added to this suite
    #[init]
    fn init() {
        rtt_target::rtt_init_log!();
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
    }

    // 0/90/180° give 500/1500/2500 us pulses, angles above 180° clamp to 180°
    #[test]
    fn servo_duty_maps_angle_to_pulse_and_clamps() {
        // 14-bit duty over a 20 ms period: 500 us -> 409, 1500 us -> 1228, 2500 us -> 2048
        assert_eq!(servo_duty(0), 409);
        assert_eq!(servo_duty(90), 1228);
        assert_eq!(servo_duty(180), 2048);
        assert_eq!(servo_duty(200), servo_duty(180));
    }

    // Goes low below 7000 mV, only recovers at 7200 mV or more
    #[test]
    fn battery_low_has_hysteresis() {
        assert!(!battery_low(false, 7000));
        assert!(battery_low(false, 6999));
        assert!(battery_low(true, 7199));
        assert!(!battery_low(true, 7200));
    }
}
