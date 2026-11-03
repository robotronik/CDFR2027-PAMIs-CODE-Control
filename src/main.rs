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

mod board;
use board::Board;

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

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let board = Board::init();

    // Utility & health monitoring tasks
    LedTask::spawn(&spawner, board.status_led, 500);
    ButtonTask::spawn(&spawner, board.boot_button);
    BatteryTask::spawn(&spawner, board.battery_level, board.peripherals.adc1);

    // Motion & odometry tasks
    MotorControlTask::spawn(&spawner, board.motors);
    OtosTask::spawn(&spawner, board.i2c, board.peripherals.i2c0);

    // Actuator servo tasks
    ServoControlTask::spawn(&spawner, board.servos);

    // High-level communication task
    BrainCommTask::spawn(&spawner, board.spi, board.peripherals.spi2);

    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}
