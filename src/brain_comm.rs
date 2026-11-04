/* BrainBoard communication module: SPI slave task that decodes requests
* from the BrainBoard and routes them to the task that owns each command.
*/
mod communication;
mod protocol;

pub use communication::{BrainCommLinks, BrainCommTask};
