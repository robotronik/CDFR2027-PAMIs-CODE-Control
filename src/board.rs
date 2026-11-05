use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
use esp_hal::peripherals::{
    ADC1, CPU_CTRL, FROM_CPU_INTR1, GPIO1, I2C0, LEDC, MCPWM0, SPI2, UART0,
};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

// One DRV8871 H-bridge channel
pub struct DcMotorPins {
    pub in1: Output<'static>,
    pub in2: Output<'static>,
}

// Connector SPI1
pub struct SpiPins {
    pub sclk: AnyPin<'static>,
    pub cs: AnyPin<'static>,
    pub handshake: AnyPin<'static>,
    pub miso: AnyPin<'static>,
    pub mosi: AnyPin<'static>,
}

// Connector I2C1
pub struct I2cPins {
    pub sda: AnyPin<'static>,
    pub scl: AnyPin<'static>,
}

// Connector UART1
pub struct UartPins {
    pub tx: AnyPin<'static>,
    pub rx: AnyPin<'static>,
}

// Peripherals handed to the tasks that build their drivers
pub struct BoardPeripherals {
    pub spi2: SPI2<'static>,
    pub i2c0: I2C0<'static>,
    pub uart0: UART0<'static>,
    pub adc1: ADC1<'static>,
    pub ledc: LEDC<'static>,
    pub mcpwm0: MCPWM0<'static>,
    pub cpu_ctrl: CPU_CTRL<'static>,
    pub intr1: FROM_CPU_INTR1<'static>,
}

pub struct Board {
    // GPIO16, active high
    pub status_led: Output<'static>,
    // SW_BOOT1 on GPIO0, active low, internal pull-up
    pub boot_button: Input<'static>,
    // ADC1_CH0, 30k/10k divider
    pub battery_level: GPIO1<'static>,
    // IN1/IN2: IO4/IO5, IO6/IO7, IO8/IO9
    pub motors: [DcMotorPins; 3],
    // SERVO_1..4: IO47 (pin 24), IO48 (pin 25), IO14 (pin 22), IO15 (pin 8)
    pub servos: [Output<'static>; 4],
    // CLK IO38, CS IO39, handshake IO40, MISO IO41, MOSI IO42 (pins 31-35)
    pub spi: SpiPins,
    // SDA IO17 (pin 10), SCL IO18 (pin 11)
    pub i2c: I2cPins,
    // TX IO43 (pin 37), RX IO44 (pin 36)
    pub uart0: UartPins,
    pub peripherals: BoardPeripherals,
}

impl Board {
    pub fn init() -> Self {
        println!("Board config start");

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

        let output_cfg = OutputConfig::default();

        let status_led = Output::new(peripherals.GPIO16, Level::Low, output_cfg);

        let boot_button = Input::new(
            peripherals.GPIO0,
            InputConfig::default().with_pull(Pull::Up),
        );

        let battery_level = peripherals.GPIO1;

        let motors = [
            DcMotorPins {
                in1: Output::new(peripherals.GPIO4, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO5, Level::Low, output_cfg),
            },
            DcMotorPins {
                in1: Output::new(peripherals.GPIO6, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO7, Level::Low, output_cfg),
            },
            DcMotorPins {
                in1: Output::new(peripherals.GPIO8, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO9, Level::Low, output_cfg),
            },
        ];

        let servos = [
            Output::new(peripherals.GPIO47, Level::Low, output_cfg),
            Output::new(peripherals.GPIO48, Level::Low, output_cfg),
            Output::new(peripherals.GPIO14, Level::Low, output_cfg),
            Output::new(peripherals.GPIO15, Level::Low, output_cfg),
        ];

        let spi = SpiPins {
            sclk: peripherals.GPIO38.degrade(),
            cs: peripherals.GPIO39.degrade(),
            handshake: peripherals.GPIO40.degrade(),
            miso: peripherals.GPIO41.degrade(),
            mosi: peripherals.GPIO42.degrade(),
        };

        let i2c = I2cPins {
            sda: peripherals.GPIO17.degrade(),
            scl: peripherals.GPIO18.degrade(),
        };

        let uart0 = UartPins {
            tx: peripherals.GPIO43.degrade(),
            rx: peripherals.GPIO44.degrade(),
        };

        let board_peripherals = BoardPeripherals {
            spi2: peripherals.SPI2,
            i2c0: peripherals.I2C0,
            uart0: peripherals.UART0,
            adc1: peripherals.ADC1,
            ledc: peripherals.LEDC,
            mcpwm0: peripherals.MCPWM0,
            cpu_ctrl: peripherals.CPU_CTRL,
            intr1: peripherals.FROM_CPU_INTR1,
        };

        println!("Board config done");

        Self {
            status_led,
            boot_button,
            battery_level,
            motors,
            servos,
            spi,
            i2c,
            uart0,
            peripherals: board_peripherals,
        }
    }
}
