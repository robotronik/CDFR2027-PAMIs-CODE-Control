#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::system::Stack;
use static_cell::{ConstStaticCell, StaticCell};

mod board;
use board::Board;

mod protocol;

mod communication;
use communication::BrainCommTask;

mod motor_control;
use motor_control::{MotorControlTask, OtosTask};

mod servo_control;
use servo_control::ServoControlTask;

mod utility;
use utility::{BatteryTask, ButtonTask, LedTask};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

/*
 * This creates a default app-descriptor required by the esp-idf bootloader.
 * For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
 */
esp_bootloader_esp_idf::esp_app_desc!();

// Core 1 dedicated stack and Embassy executor
static STACK_CORE_1: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
static EXECUTOR_CORE_1: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let board = Board::init();

    /*
     * Launch Core 1 (APP_CPU) dedicated exclusively to MotorControlTask.
     * Core 1 runs its own Embassy executor with an independent 8KB stack.
     */
    let motors = board.motors;
    esp_rtos::start_second_core(
        board.peripherals.cpu_ctrl,
        board.peripherals.intr1,
        STACK_CORE_1.take(),
        move || {
            let executor = EXECUTOR_CORE_1.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner_core1| {
                MotorControlTask::spawn(&spawner_core1, motors);
            });
        },
    );

    /*
     * Core 0 (PRO_CPU) runs all remaining system, communication, and utility tasks:
     * - Status LED, boot button, battery monitor
     * - Odometry (OTOS) on I2C0
     * - Actuator servos
     * - High-level BrainBoard communication on SPI2
     */
    LedTask::spawn(&spawner, board.status_led, 500);
    ButtonTask::spawn(&spawner, board.boot_button);
    BatteryTask::spawn(&spawner, board.battery_level, board.peripherals.adc1);

    OtosTask::spawn(&spawner, board.i2c, board.peripherals.i2c0);
    ServoControlTask::spawn(&spawner, board.servos);

    BrainCommTask::spawn(&spawner, board.spi, board.peripherals.spi2);

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
