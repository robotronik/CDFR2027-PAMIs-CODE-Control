#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use esp_hal::system::Stack;
use static_cell::{ConstStaticCell, StaticCell};

use pami_control::board::Board;
use pami_control::brain_comm::{BrainCommLinks, BrainCommTask};
use pami_control::motor_control::{MotionCommandChannel, MotorControlTask, Otos, RobotPosWatch};
use pami_control::servo_control::{ServoCommandChannel, ServoControlTask};
use pami_control::utility::{BatteryTask, ButtonTask, LedTask};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    Board::stop_motors_after_panic();
    log::error!("{}", info);
    loop {}
}

// App descriptor required by the esp-idf bootloader
esp_bootloader_esp_idf::esp_app_desc!();

static STACK_CORE_1: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());
static EXECUTOR_CORE_1: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

// MOTION_COMMANDS: BrainComm (Core 0) -> MotorControl (Core 1)
// ROBOT_POS:       MotorControl (Core 1) -> BrainComm (Core 0)
// SERVO_COMMANDS:  BrainComm (Core 0) -> ServoControl (Core 0)
static MOTION_COMMANDS: MotionCommandChannel = MotionCommandChannel::new();
static ROBOT_POS: RobotPosWatch = RobotPosWatch::new();
static SERVO_COMMANDS: ServoCommandChannel = ServoCommandChannel::new();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let board = Board::init();

    // Core 1 runs only MotorControl, which owns the motors and the OTOS
    let motors = board.motors;
    let mcpwm0 = board.peripherals.mcpwm0;
    let (otos_pins, i2c0) = (board.i2c, board.peripherals.i2c0);
    esp_rtos::start_second_core(
        board.peripherals.cpu_ctrl,
        board.peripherals.intr1,
        STACK_CORE_1.take(),
        move || {
            let otos = Otos::new(otos_pins, i2c0);
            let executor = EXECUTOR_CORE_1.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner_core1| {
                MotorControlTask::spawn(
                    &spawner_core1,
                    motors,
                    mcpwm0,
                    otos,
                    MOTION_COMMANDS.receiver(),
                    ROBOT_POS.sender(),
                );
            });
        },
    );

    // Core 0 runs everything else
    LedTask::spawn(&spawner, board.status_led, 500);
    ButtonTask::spawn(&spawner, board.boot_button);
    BatteryTask::spawn(&spawner, board.battery_level, board.peripherals.adc1);

    ServoControlTask::spawn(
        &spawner,
        board.servos,
        board.peripherals.ledc,
        SERVO_COMMANDS.receiver(),
    );

    BrainCommTask::spawn(
        &spawner,
        board.spi,
        board.peripherals.spi2,
        board.peripherals.dma0,
        BrainCommLinks {
            motion: MOTION_COMMANDS.sender(),
            servo: SERVO_COMMANDS.sender(),
            robot_pos: ROBOT_POS
                .receiver()
                .expect("RobotPos watch has no free receiver slot"),
        },
    );

    core::future::pending().await
}
