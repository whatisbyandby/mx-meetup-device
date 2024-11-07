#![no_std]
#![no_main]

use core::fmt::Write;
use core::str;
use core::str::from_utf8;
use cyw43_pio::PioSpi;
use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Config as NetConfig, Stack, StackResources};
use embassy_net_driver_channel::Device as D;
use embassy_rp::adc::{
    Adc, Channel as AdcChannel, Config, InterruptHandler as AdcInterruptHandler,
};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::flash::Blocking;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config as UsbConfig};
use embassy_usb_logger::ReceiverHandler;

use rand::RngCore;

// use heapless::String;
use mx_meetup_lib::{parse_command, DemoDeviceBuilder, PicoCommand};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

bind_interrupts!(struct AdcIrqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel as SyncChannel;

static COMMAND_CHANNEL: SyncChannel<ThreadModeRawMutex, Result<PicoCommand, &'static str>, 5> =
    SyncChannel::new();

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<D<'static, 1514>>) -> ! {
    stack.run().await
}

const FLASH_SIZE: usize = 2 * 1024 * 1024;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let watchdog = Watchdog::new(p.WATCHDOG);

    let mut rng = RoscRng;

    let mut flash = embassy_rp::flash::Flash::<_, Blocking, FLASH_SIZE>::new_blocking(p.FLASH);

    let mut uid = [0; 8];
    flash.blocking_unique_id(&mut uid).unwrap();
    let mut id_string = heapless::String::<16>::new();
    write!(
        id_string,
        "{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
        uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]
    )
    .unwrap();

    static ID_STRING: StaticCell<heapless::String<16>> = StaticCell::new();
    let static_id = ID_STRING.init(id_string);

    // Get unique id
    let fw = include_bytes!("../../pico-w-cyw43/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../pico-w-cyw43/cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, PioIrqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    unwrap!(spawner.spawn(cyw43_task(runner)));

    control.init(clm).await;

    let config = NetConfig::dhcpv4(Default::default());

    // Generate random seed
    let seed = rng.next_u64();

    // Init network stack
    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let stack = embassy_net::Stack::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    static STACK: StaticCell<Stack<D<'static, 1514>>> = StaticCell::new();
    let static_stack = STACK.init(stack);

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, UsbIrqs);
    spawner.spawn(logger_task(driver)).unwrap();

    // Create the ADC and the temperature sensor channel.
    let adc = Adc::new(p.ADC, AdcIrqs, Config::default());
    let ts = AdcChannel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // Run the network stack.
    unwrap!(spawner.spawn(net_task(static_stack)));

    let command_receiver: embassy_sync::channel::Receiver<
        '_,
        ThreadModeRawMutex,
        Result<PicoCommand, &str>,
        5,
    > = COMMAND_CHANNEL.receiver();

    let mut demo_device = DemoDeviceBuilder::new()
        .with_id(static_id)
        .with_stack(static_stack)
        .with_command_receiver(command_receiver)
        .with_control(control)
        .with_adc(adc, ts)
        .with_watchdog(watchdog)
        .with_flash(flash)
        .build();

    demo_device.init().await;

    info!("Device Initalized!");

    demo_device.run().await;
}

struct Handler;

impl ReceiverHandler for Handler {
    async fn handle_data(&self, data: &[u8]) {
        if let Ok(data) = str::from_utf8(data) {
            let data = data.trim();

            let mut full_data: heapless::String<256> = heapless::String::new();
            full_data.push_str(data).unwrap();

            let command = parse_command(full_data);

            COMMAND_CHANNEL.send(command).await;
        }
    }

    fn new() -> Self {
        Self
    }
}

#[embassy_executor::task]
async fn logger_task(driver: Driver<'static, USB>) {
    // Create embassy-usb Config
    let mut config = UsbConfig::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Paradigm");
    config.product = Some("Demo Device");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);
    let log_fut = embassy_usb_logger::with_custom_style!(
        1024,
        log::LevelFilter::Info,
        class,
        |record, writer| {
            use core::fmt::Write;
            let level = record.level().as_str();
            write!(writer, "[{level}] {}\r\n", record.args()).unwrap();
        },
        Handler
    );

    let mut usb = builder.build();
    let usb_fut = usb.run();
    join(usb_fut, log_fut).await;
    
}
