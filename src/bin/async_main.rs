#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use embassy_executor::{task, Spawner};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio::{Input, Io},
    prelude::*,
};

use log::info;

use esp_println::println;

use defmt_rtt as _;

extern crate alloc;

#[handler(priority = esp_hal::interrupt::Priority::max())]
#[ram]
fn my_handler() {
    let current_time = Instant::now();
    critical_section::with(|cs| {
        let mut channels = CHANNELS.borrow(cs).borrow_mut();
        for (ch, sig) in channels.iter_mut().zip(SIGNALS.iter()) {
            if let Some(input_mut) = ch.0.as_mut() {
                if input_mut.is_interrupt_set() {
                    input_mut.clear_interrupt();
                    if input_mut.is_high() {
                        ch.1.rising(current_time);
                    } else {
                        sig.signal(ch.1.falling(current_time));
                    }
                }
            }
        }
    });
}

pub struct TonReader {
    rising: Instant,
}

impl TonReader {
    pub const fn new(now: Instant) -> Self {
        Self { rising: now }
    }

    pub fn rising(&mut self, rising: Instant) {
        self.rising = rising
    }

    pub fn falling(&self, falling: Instant) -> Duration {
        falling - self.rising
    }
}

pub const CH_COUNT: usize = 2;

static CHANNELS: Mutex<RefCell<[(Option<Input>, TonReader); CH_COUNT]>> = const {
    Mutex::new(RefCell::new(
        [const { (None, TonReader::new(Instant::from_ticks(0))) }; CH_COUNT],
    ))
};

static SIGNALS: [signal::Signal<CriticalSectionRawMutex, Duration>; CH_COUNT] =
    [const { signal::Signal::new() }; CH_COUNT];

#[main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);

    esp_println::logger::init_logger_from_env();

    let timer0 = esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
        .split::<esp_hal::timer::systimer::Target>();
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let mut steering_ch = Input::new(peripherals.GPIO14, esp_hal::gpio::Pull::Down);
    let mut throttle_ch = Input::new(peripherals.GPIO13, esp_hal::gpio::Pull::Down);

    steering_ch.listen(esp_hal::gpio::Event::AnyEdge);
    throttle_ch.listen(esp_hal::gpio::Event::AnyEdge);

    critical_section::with(|cs| {
        let mut channels_borrow = CHANNELS.borrow(cs).borrow_mut();
        channels_borrow[0].0.replace(steering_ch);
        channels_borrow[1].0.replace(throttle_ch);
    });

    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(my_handler);

    info!("spawning receiver task");

    spawner.spawn(run_receiver()).unwrap();

    let mut button = Input::new(peripherals.GPIO0, esp_hal::gpio::Pull::Up);

    loop {
        _ = embedded_hal_async::digital::Wait::wait_for_falling_edge(&mut button).await;
        critical_section::with(|cs| {
            let mut channels = CHANNELS.borrow(cs).borrow_mut();

            for ch in channels.iter_mut() {
                if let Some(ch) = ch.0.as_mut() {
                    ch.listen(esp_hal::gpio::Event::AnyEdge);
                }
            }
        });
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/v0.22.0/examples/src/bin
}

#[task]
async fn run_receiver() {
    let my_futs = SIGNALS.iter().enumerate().map(|(n, sig)| async move {
        loop {
            println!("t {}: {}", n, sig.wait().await.as_micros())
        }
    });

    futures::future::join_all(my_futs).await;
}
