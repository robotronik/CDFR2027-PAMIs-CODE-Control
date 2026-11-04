use super::servo_commands::{ServoCommand, ServoCommandReceiver};
use embassy_executor::Spawner;
use esp_hal::gpio::Output;
use esp_println::println;

const SERVO_COUNT: usize = 4;

#[embassy_executor::task]
async fn run_servo_control(
    mut servos: [Output<'static>; SERVO_COUNT],
    command_receiver: ServoCommandReceiver,
) {
    /*
     * Actuator servo control loop.
     * Waits for servo commands and drives the 4 robot servos.
     */
    loop {
        let command = command_receiver.receive().await;
        match command {
            ServoCommand::SetAngle { id, angle_deg } => {
                let Some(servo) = servos.get_mut(id as usize) else {
                    println!("ServoControl: ignoring command for unknown servo {}", id);
                    continue;
                };
                let _ = servo;
                // TODO: drive servo PWM
                println!("ServoControl: servo {} -> {:.1} deg", id, angle_deg);
            }
        }
    }
}

pub struct ServoControlTask;

impl ServoControlTask {
    pub fn spawn(
        spawner: &Spawner,
        servos: [Output<'static>; SERVO_COUNT],
        command_receiver: ServoCommandReceiver,
    ) {
        let token = run_servo_control(servos, command_receiver)
            .expect("Failed to create ServoControlTask (token exhaustion)");
        spawner.spawn(token);
    }
}
