//! Example using the `hc-sr04` create together with the
//! [`STM32F3Discovery`][1].
//!
//! [1]: https://github.com/japaric/f3

#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;
extern crate hc_sr04;

use f3::hal::delay::Delay;
use f3::hal::gpio::gpioa::PA8;
use f3::hal::gpio;
use f3::hal::prelude::*;
use f3::hal::stm32f30x;
use f3::hal::time::MonoTimer;
use f3::led::Leds;
use hc_sr04::{HcSr04, WouldBlock};
use rtfm::{app, Resource, Threshold};

// Maximum distance to show with LEDs
const MAX_DISTANCE: u32 = 10;

app! {
    device: stm32f30x,

    resources: {
        // In this example we use `PA8` expecting the user to connect
        // `echo` from sensor to either PA10-PA15 (due to the interrupt being
        // expected on either one). Check the `init` function to change
        // pin configurations.
        static SENSOR: HcSr04<PA8<gpio::Output<gpio::PushPull>>, Delay>;
        static LEDS: Leds;
    },

    idle: {
        resources: [SENSOR, LEDS],
    },

    tasks: {
        EXTI15_10: {
            path: update,
            resources: [SENSOR],
        }
    },
}

fn init(p: init::Peripherals) -> init::LateResources {
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
    // TODO: Setup interrupt on `EXTI15_10`
    // Create sensor!
    let sensor = HcSr04::new(pin, delay, timer);
    // Prepare LEDs
    let leds = Leds::new(p.device.GPIOE.split(&mut rcc.ahb));

    // Return late resources
    init::LateResources {
        SENSOR: sensor,
        LEDS: leds,
    }
}

fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        // Since `idle` has lowest priority we have to use `claim_mut` to
        // access the sensor
        let dist = r.SENSOR.claim_mut(t, |s, _t| s.distance());
        match dist {
            Ok(dist) => {
                // Distance in cm:
                let cm = dist.cm();
                // How many LEDs should we turn on:
                let num_leds = {
                    if cm <= 2 {
                        // 2cm is the smallest distance the sensor will
                        // reliably report so use that as minimum
                        r.LEDS.len()
                    } else if cm > MAX_DISTANCE {
                        0
                    } else {
                        let num = r.LEDS.len() as f32 * ((MAX_DISTANCE - 2) as f32 / cm as f32);
                        // Poor mans rounding
                        r.LEDS.len() - (num + 0.5) as usize
                    }
                };
                // Turn on LEDs
                r.LEDS.iter_mut()
                    // First turn everyone off
                    .map(|l|{ l.off(); l})
                    // Take the appropriate number:
                    .take(num_leds)
                    // Turn on:
                    .for_each(|l| l.on());
            }
            Err(WouldBlock) => {
                // Tried to poll the sensor, but nothing is ready yet.
                // Either the poll started a measurement or an interrupt
                // occurred. We can then wait for the sensor to perform
                // measurement and cause further interrupts.
                rtfm::wfi();
            }
            Err(_) => panic!("This should never be reached"),
        }
    }
}

// Function to notify sensor of external interrupt. If setup correctly
// the interrupt should occur once the echo pin is pulled high or low.
fn update(_t: &mut Threshold, mut r: EXTI15_10::Resources) {
    r.SENSOR
        .update()
        .expect("Interrupt function called while sensor were in wrong state!");
    // TODO: Remove interrupt from pending?
}
