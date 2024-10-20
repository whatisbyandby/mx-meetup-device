#![no_std]
#![no_main]

use core::str::from_utf8;
use core::fmt::Write;
use cyw43_pio::PioSpi;
use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_rp::adc::{
    Adc, Channel as AdcChannel, Config, InterruptHandler as AdcInterruptHandler,
};
use embassy_rp::bind_interrupts;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, Instance, InterruptHandler as UsbInterruptHandler};
use embassy_time::Timer;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::UsbDevice;

use embassy_net::{Config as NetConfig, Ipv4Address, Stack, StackResources};
use embassy_net_driver_channel::Device as D;

use rand::RngCore;

use heapless::String;
use mx_meetup_lib::{parse_command, DemoDevice, DemoDeviceBuilder, DeviceState, PicoCommand};
use rust_mqtt::client::client::MqttClient;
use rust_mqtt::client::client_config::ClientConfig;
use rust_mqtt::packet::v5::reason_codes::ReasonCode;
use rust_mqtt::utils::rng_generator::CountingRng;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use mx_meetup_lib::temperature_sensor::{BuiltInTemperatureSensor, TemperatureSensor};



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
use embassy_sync::channel::{Channel as SyncChannel, Sender};

static COMMAND_CHANNEL: SyncChannel<ThreadModeRawMutex, Result<PicoCommand, &'static str>, 64> =
    SyncChannel::new();



#[embassy_executor::task]
async fn command_task(mut receiver: Receiver<'static, Driver<'static, USB>>) {
    receiver.wait_connection().await;
    loop {
        let command = get_command(&mut receiver).await;
        COMMAND_CHANNEL.send(command).await;
    }
}

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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut rng = RoscRng;

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

    // Create the ADC and the temperature sensor channel.
    let adc = Adc::new(p.ADC, AdcIrqs, Config::default());
    let ts = AdcChannel::new_temp_sensor(p.ADC_TEMP_SENSOR);

    // Create embassy-usb Config
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Paradigm");
        config.product = Some("Mx Meetup Demo Device");
        config.serial_number = Some("12345678");
        config.max_power = 100;
        config.max_packet_size_0 = 64;

        // Required for windows compatibility.
        // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    // Create classes on the builder.
    let class = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    // Build the builder.
    let usb = builder.build();

    let (sender, receiver) = class.split();

    // Run the network stack.
    unwrap!(spawner.spawn(net_task(static_stack)));
    // Run the USB device.
    unwrap!(spawner.spawn(usb_task(usb)));
    // Run the command task.
    unwrap!(spawner.spawn(command_task(receiver)));

    let command_receiver: embassy_sync::channel::Receiver<'_, ThreadModeRawMutex, Result<PicoCommand, &str>, 64> = COMMAND_CHANNEL.receiver();

    let mut demo_device = DemoDeviceBuilder::new()
        .with_stack(static_stack)
        .with_command_receiver(command_receiver)
        .with_usb_sender(sender)
        .with_control(control)
        .with_adc(adc, ts)
        .build();


    // let mut rx_buffer = [0; 4096];
    // let mut tx_buffer = [0; 4096];

    // let mut socket = TcpSocket::new(static_stack, &mut rx_buffer, &mut tx_buffer);

    // socket.set_timeout(None);

    // let address = Ipv4Address::from_bytes(&[192, 168, 1, 89]);
    // let remote_endpoint = (address, 1883);

    // let connection = socket.connect(remote_endpoint).await;

    // if let Err(e) = connection {
    //     info!("connect error: {:?}", e);
    // }
    // info!("connected!");

    // let mut config = ClientConfig::new(
    //     rust_mqtt::client::client_config::MqttVersion::MQTTv5,
    //     CountingRng(20000),
    // );
    // config.add_max_subscribe_qos(rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1);
    // config.add_client_id("clientId-8rhWgBODCl");
    // config.add_will("device/1/status", "DISCONNECTED".as_bytes(), true);
    // config.max_packet_size = 100;
    // let mut recv_buffer = [0; 80];
    // let mut write_buffer = [0; 80];

    // let mut client =
    //     MqttClient::<_, 5, _>::new(socket, &mut write_buffer, 80, &mut recv_buffer, 80, config);

    // match client.connect_to_broker().await {
    //     Ok(()) => {}
    //     Err(mqtt_error) => match mqtt_error {
    //         ReasonCode::NetworkError => {
    //             info!("MQTT Network Error");
    //         }
    //     ReasonCode::Success => {
    //             info!("Success");
    //         }
    //         _ => {
    //             info!("Another Error");
    //         }
    //     },
    // }

    demo_device.init().await;

    // The main loop for the device
    loop {
       demo_device.run().await;

        // if demo_device.get_config().is_none() {
        //     Timer::after_millis(1000).await;
        //     info!("Waiting for config");
        //     continue;
        // }

        // Timer::after_millis(100).await;

        // let state_json = demo_device.get_state_json().unwrap();

        // let mut topic: String<64> = String::new(); // Fixed capacity of 64 bytes

        // write!(topic, "device/{}/state", "1").unwrap();
        
        
        // match client
        //     .send_message(
        //         topic.as_str(),
        //         state_json.as_bytes(),
        //         rust_mqtt::packet::v5::publish_packet::QualityOfService::QoS1,
        //         true,
        //     )
        //     .await
        // {
        //     Ok(()) => {}
        //     Err(mqtt_error) => match mqtt_error {
        //         ReasonCode::NetworkError => {
        //             info!("MQTT Network Error");
        //         }
        //         _ => {
        //             info!("Error while sending message");
        //         }
        //     },
        // }
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn get_command<'d, T: Instance + 'd>(
    class: &mut Receiver<'d, Driver<'d, T>>,
) -> Result<PicoCommand, &'static str> {
    let mut full_data: String<256> = String::new();
    let mut done = false;
    while !done {
        let mut buf = [0; 64];
        let num_chars_read = class.read_packet(&mut buf).await.unwrap();
        let packet_data = &buf[..num_chars_read];
        full_data
            .push_str(from_utf8(packet_data).unwrap())
            .map_err(|_| "Command contains non-UTF characters")?;
        info!("{}", full_data.as_str());
        if num_chars_read < 64 {
            done = true;
        }
    }

    let command = parse_command(full_data)?;
    Ok(command)
}
