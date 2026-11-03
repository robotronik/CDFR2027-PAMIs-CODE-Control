use esp_hal::clock::CpuClock;
use esp_hal::gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pin, Pull};
use esp_hal::peripherals::{ADC1, I2C0, LEDC, SPI2, UART0, GPIO1};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

// Pins for a single DC motor channel driven by a DRV8871 H-bridge
pub struct DcMotorPins {
    pub in1: Output<'static>,
    pub in2: Output<'static>,
}

// 5-pin SPI interface on connector SPI1
pub struct SpiPins {
    pub sclk: AnyPin<'static>,
    pub cs: AnyPin<'static>,
    pub handshake: AnyPin<'static>,
    pub miso: AnyPin<'static>,
    pub mosi: AnyPin<'static>,
}

// 2-pin I2C interface on connector I2C1
pub struct I2cPins {
    pub sda: AnyPin<'static>,
    pub scl: AnyPin<'static>,
}

// 2-pin UART interface on connector UART1 (UART0)
pub struct UartPins {
    pub tx: AnyPin<'static>,
    pub rx: AnyPin<'static>,
}

// Major hardware peripheral singletons retained for driver initialization
pub struct BoardPeripherals {
    pub spi2: SPI2<'static>,
    pub i2c0: I2C0<'static>,
    pub uart0: UART0<'static>,
    pub adc1: ADC1<'static>,
    pub ledc: LEDC<'static>,
}

// Complete board hardware configuration and pin definitions
pub struct Board {
    // Active-high status LED on GPIO16
    pub status_led: Output<'static>,
    // Active-low boot button SW_BOOT1 on GPIO0 with internal pull-up
    pub boot_button: Input<'static>,
    // Battery level analog sensing pin on GPIO1 (ADC1_CH0, 30k/10k divider)
    pub battery_level: GPIO1<'static>,
    // 3 DC motors on DRV8871 drivers (Motor 1: IO4/IO5, Motor 2: IO6/IO7, Motor 3: IO8/IO9)
    pub motors: [DcMotorPins; 3],
    // 4 Servo PWM outputs (SERVO_1: IO47, SERVO_2: IO48, SERVO_3: IO14, SERVO_4: IO15)
    pub servos: [Output<'static>; 4],
    // 5 SPI bus pins (CLK: IO38, CS: IO39, Handshake: IO40, MISO: IO41, MOSI: IO42)
    pub spi: SpiPins,
    // I2C bus pins (SDA: IO17, SCL: IO18)
    pub i2c: I2cPins,
    // UART0 bus pins (TX: IO43, RX: IO44)
    pub uart0: UartPins,
    // Hardware peripheral blocks available for application tasks
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

        // Status LED on GPIO16 (Active High via 470R resistor to LED)
        let status_led = Output::new(peripherals.GPIO16, Level::Low, output_cfg);

        // Boot Button on GPIO0 (Active Low with internal pull-up enabled)
        let boot_button = Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up));

        // Battery level sense pin (GPIO1) retained for ADC1_CH0
        let battery_level = peripherals.GPIO1;

        // 3 DC Motors (DRV8871 H-Bridges)
        let motors = [
            // DC Motor 1: IN1 = GPIO4, IN2 = GPIO5
            DcMotorPins {
                in1: Output::new(peripherals.GPIO4, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO5, Level::Low, output_cfg),
            },
            // DC Motor 2: IN1 = GPIO6, IN2 = GPIO7
            DcMotorPins {
                in1: Output::new(peripherals.GPIO6, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO7, Level::Low, output_cfg),
            },
            // DC Motor 3: IN1 = GPIO8, IN2 = GPIO9
            DcMotorPins {
                in1: Output::new(peripherals.GPIO8, Level::Low, output_cfg),
                in2: Output::new(peripherals.GPIO9, Level::Low, output_cfg),
            },
        ];

        /*
         * 4 Servo PWM Outputs
         * SERVO_1 on Pin 24 (IO47), SERVO_2 on Pin 25 (IO48)
         * SERVO_3 on Pin 22 (IO14), SERVO_4 on Pin 8 (IO15)
         */
        let servos = [
            Output::new(peripherals.GPIO47, Level::Low, output_cfg),
            Output::new(peripherals.GPIO48, Level::Low, output_cfg),
            Output::new(peripherals.GPIO14, Level::Low, output_cfg),
            Output::new(peripherals.GPIO15, Level::Low, output_cfg),
        ];

        /*
         * 5-Pin SPI Interface on Connector SPI1
         * CLK: IO38 (Pin 31), CS: IO39 (Pin 32), Handshake: IO40 (Pin 33)
         * MISO: IO41 (Pin 34), MOSI: IO42 (Pin 35)
         */
        let spi = SpiPins {
            sclk: peripherals.GPIO38.degrade(),
            cs: peripherals.GPIO39.degrade(),
            handshake: peripherals.GPIO40.degrade(),
            miso: peripherals.GPIO41.degrade(),
            mosi: peripherals.GPIO42.degrade(),
        };

        // I2C Interface on Connector I2C1 (SDA: IO17 / Pin 10, SCL: IO18 / Pin 11)
        let i2c = I2cPins {
            sda: peripherals.GPIO17.degrade(),
            scl: peripherals.GPIO18.degrade(),
        };

        // UART0 Interface on Connector UART1 (TX: IO43 / Pin 37, RX: IO44 / Pin 36)
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
