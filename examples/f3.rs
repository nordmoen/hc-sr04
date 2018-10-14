//! Example using the `hc-sr04` create together with the
//! [`STM32F3Discovery`][1].
//!
//! [1]: https://github.com/japaric/f3

#![feature(extern_prelude)]
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f3;
extern crate hc_sr04;
extern crate panic_itm;

use cortex_m::peripheral::ITM;
use f3::hal::delay::Delay;
use f3::hal::gpio;
use f3::hal::gpio::gpioa::PA8;
use f3::hal::prelude::*;
use f3::hal::stm32f30x;
use f3::hal::time::Instant;
use f3::hal::time::MonoTimer;
use f3::led::Leds;
use hc_sr04::{Error, HcSr04};
use rtfm::{app, Resource, Threshold};

app! {
    device: stm32f30x,

    resources: {
        // In this example we use `PA8` expecting the user to connect
        // `echo` from sensor to either PA10-PA15 (due to the interrupt being
        // expected on either one). Check the `init` function to change
        // pin configurations.
        static SENSOR: HcSr04<PA8<gpio::Output<gpio::PushPull>>, Delay>;
        static LEDS: Leds;
        static EXTI: stm32f30x::EXTI;
        static ITM: ITM;
        static TIMER: MonoTimer;
        static TS: Option<Instant> = None;
    },

    idle: {
        resources: [SENSOR, LEDS, ITM],
    },

    tasks: {
        EXTI15_10: {
            path: update,
            resources: [SENSOR, EXTI, TIMER, TS],
        }
    },
}

fn init(p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.ahb);

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Necessary facilities for sensor
    let delay = Delay::new(p.core.SYST, clocks);
    let timer = MonoTimer::new(p.core.DWT, clocks);
    let pin = gpioa
        .pa8
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    // Create sensor!
    let sensor = HcSr04::new(pin, delay, timer.frequency().0);
    // Prepare LEDs
    let leds = Leds::new(p.device.GPIOE.split(&mut rcc.ahb));
    // Setup interrupt on Pin PA15
    p.device.EXTI.imr1.write(|w| w.mr15().set_bit());
    p.device.EXTI.ftsr1.write(|w| w.tr15().set_bit());
    p.device.EXTI.rtsr1.write(|w| w.tr15().set_bit());

    // Return late resources
    init::LateResources {
        SENSOR: sensor,
        LEDS: leds,
        EXTI: p.device.EXTI,
        ITM: p.core.ITM,
        TIMER: timer,
    }
}

fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    let _stim = &mut r.ITM.stim[0];
    loop {
        // Since `idle` has lowest priority we have to use `claim_mut` to
        // access the sensor
        let dist = r.SENSOR.claim_mut(t, |s, _t| s.distance());
        match dist {
            Ok(dist) => {
                match dist {
                    Some(dist) => {
                        // Distance in cm:
                        let cm = dist.cm();
                        iprintln!(_stim, "{:?}", cm);
                        // How many LEDs should we turn on:
                        let num_leds = {
                            if cm <= 2 {
                                // 2cm is the smallest distance the sensor will
                                // reliably report so use that as minimum
                                r.LEDS.len()
                            } else if cm >= 10 {
                                0
                            } else {
                                r.LEDS.len() - (cm as usize - 2)
                            }
                        };
                        r.LEDS.iter_mut().for_each(|l| l.off());
                        // Turn on LEDs
                        r.LEDS.iter_mut()
                            // Take the appropriate number:
                            .take(num_leds)
                            // Turn on:
                            .for_each(|l| l.on());
                    }
                    None => {
                        iprintln!(_stim, "Error");
                    }
                }
            }
            Err(Error::WouldBlock) => {
                // Tried to poll the sensor, but nothing is ready yet.
                // Either the poll started a measurement or an interrupt
                // occurred. We can wait for the sensor to perform
                // measurement and cause further interrupts.
                rtfm::wfi();
            }
            Err(_) => unreachable!(),
        }
    }
}

// Function to notify sensor of external interrupt. If setup correctly
// the interrupt should occur once the echo pin is pulled high or low.
fn update(_t: &mut Threshold, mut r: EXTI15_10::Resources) {
    match *r.TS {
        Some(ts) => {
            let delta = ts.elapsed();
            *r.TS = None;

            r.SENSOR
                .capture(delta)
                .expect("echo handler: sensor in wrong state!");
        }
        None => {
            *r.TS = Some(r.TIMER.now());

            r.SENSOR
                .capture(0)
                .expect("echo handler: sensor in wrong state!");
        }
    }

    r.EXTI.pr1.write(|w| w.pr15().set_bit());
}
