// Board housekeeping on Core 0: status LED, boot button, battery monitor
mod battery;
mod button;
mod led;

pub use battery::BatteryTask;
pub use button::ButtonTask;
pub use led::LedTask;
