// Board safety tests, run on the board with `cargo hil`. No wiring needed.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use embassy_time::Timer;
    use esp_hal::mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode};
    use esp_hal::peripherals::{GPIO, IO_MUX};
    use esp_hal::time::Rate;
    use pami_control::board::Board;

    // Reads the GPIO4 pad itself (motor 0 IN1), whatever peripheral drives it
    fn gpio4_pad_is_high() -> bool {
        IO_MUX::regs().gpio(4).modify(|_, w| w.fun_ie().set_bit());
        GPIO::regs().in_().read().bits() & (1 << 4) != 0
    }

    #[init]
    fn init() -> Board {
        rtt_target::rtt_init_log!();
        Board::init()
    }

    // With PWM driving IN1 high, the panic stop pulls the pin low
    #[test]
    async fn stop_motors_after_panic_overrides_running_pwm(board: Board) {
        let [m0, ..] = board.motors;
        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
        let mut mcpwm = McPwm::new(board.peripherals.mcpwm0, clock_cfg);
        mcpwm.operator0.set_timer(&mcpwm.timer0);
        let mut pwm = mcpwm.operator0.with_pins(
            m0.in1,
            PwmPinConfig::UP_ACTIVE_HIGH,
            m0.in2,
            PwmPinConfig::UP_ACTIVE_HIGH,
        );
        let timer_cfg = clock_cfg
            .timer_clock_with_frequency(999, PwmWorkingMode::Increase, Rate::from_khz(20))
            .unwrap();
        mcpwm.timer0.start(timer_cfg);
        // Same 100 % duty as motor_duty(1000)
        pwm.0.set_timestamp(1000);
        Timer::after_millis(1).await;
        assert!(gpio4_pad_is_high(), "PWM didn't drive IN1 high, test setup is wrong");

        Board::stop_motors_after_panic();
        Timer::after_millis(1).await;
        assert!(!gpio4_pad_is_high(), "IN1 still high after stop_motors_after_panic");
    }
}
