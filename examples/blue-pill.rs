#![feature(extern_prelude)]
#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
extern crate cortex_m;

extern crate cortex_m as cm;

extern crate cortex_m_rtfm as rtfm;
use rtfm::{app, Resource, Threshold};

extern crate panic_itm;

extern crate stm32f103xx_hal as hal;
use hal::delay::Delay;
use hal::prelude::*;
use hal::time::Instant;
use hal::time::MonoTimer;
use hal::timer::Event;
use hal::timer::Timer;

extern crate hc_sr04;
use hc_sr04::{Error, HcSr04};

app! {
    device: hal::stm32f103xx,

    resources: {
        static SENSOR: HcSr04<hal::gpio::gpioa::PA1<hal::gpio::Output<hal::gpio::PushPull>>, Delay>;
        static TIMER: MonoTimer;
        static EXTI: hal::stm32f103xx::EXTI;
        static LED: hal::gpio::gpioc::PC13<hal::gpio::Output<hal::gpio::PushPull>>;
        static ITM: hal::stm32f103xx::ITM;
        static TMR: hal::timer::Timer<stm32f103xx::TIM3>;
        static TS: Option<Instant> = None;
    },

    idle: {
        resources: [TMR, SENSOR, LED, ITM],
    },

    tasks: {
        TIM3: {
            path: timeout_handler,
            resources: [TMR, ITM, SENSOR, TS],
        },
        EXTI0: {
            path: echo_handler,
            resources: [ITM, SENSOR, EXTI, TIMER, TS],
        }
    },
}

fn init(p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    // configure clocks
    let mut flash = p.device.FLASH.constrain();
    let mut rcc = p.device.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(8.mhz())
        .pclk1(8.mhz())
        .freeze(&mut flash.acr);

    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);
    let mut gpioc = p.device.GPIOC.split(&mut rcc.apb2);

    let delay = Delay::new(p.core.SYST, clocks);
    let monotimer = MonoTimer::new(p.core.DWT, clocks);

    // configure PC13 pin to blink LED
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // configure PA0 pin to capture echo
    gpioa.pa0.into_floating_input(&mut gpioa.crl);

    // configure PA1 pin to trigger pulse
    let trig = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);

    // configure and start TIM3 periodic timer
    let tmr = Timer::tim3(p.device.TIM3, 1.hz(), clocks, &mut rcc.apb1);

    // setup EXTI0 interrupt: pin PA0
    p.device.EXTI.imr.write(|w| w.mr0().set_bit());
    p.device.EXTI.ftsr.write(|w| w.tr0().set_bit());
    p.device.EXTI.rtsr.write(|w| w.tr0().set_bit());

    // create sensor
    let sensor = HcSr04::new(trig, delay, monotimer.frequency().0);

    // init late resources
    init::LateResources {
        SENSOR: sensor,
        LED: led,
        EXTI: p.device.EXTI,
        TIMER: monotimer,
        TMR: tmr,
        ITM: p.core.ITM,
    }
}

fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        r.TMR.claim_mut(t, |s, _t| {
            s.start(1.hz());
            s.listen(Event::Update);
        });

        loop {
            let dist = r.SENSOR.claim_mut(t, |s, _t| s.distance());
            match dist {
                Ok(dist) => {
                    r.TMR.claim_mut(t, |s, _t| {
                        s.unlisten(Event::Update);
                    });

                    match dist {
                        Some(dist) => {
                            let cm = dist.cm();
                            r.ITM.claim_mut(t, |s, _t| {
                                iprintln!(&mut s.stim[0], "{:?}", cm);
                            });
                            break;
                        }
                        None => {
                            r.ITM.claim_mut(t, |s, _t| {
                                iprintln!(&mut s.stim[0], "Err");
                            });
                            break;
                        }
                    }
                }
                Err(Error::WouldBlock) => {
                    rtfm::wfi();
                }
                Err(_) => unreachable!(),
            }
        }

        for _ in 0..10000 {
            cm::asm::nop();
        }
    }
}

fn echo_handler(_t: &mut Threshold, mut r: EXTI0::Resources) {
    let dbg = &mut r.ITM.stim[0];

    match *r.TS {
        Some(ts) => {
            let delta = ts.elapsed();
            *r.TS = None;
            iprintln!(dbg, "stop capture: {:?}", delta);

            r.SENSOR
                .capture(delta)
                .expect("echo handler: sensor in wrong state!");
        }
        None => {
            *r.TS = Some(r.TIMER.now());
            iprintln!(dbg, "start capture");

            r.SENSOR
                .capture(0)
                .expect("echo handler: sensor in wrong state!");
        }
    }

    r.EXTI.pr.write(|w| w.pr0().set_bit());
}

fn timeout_handler(_t: &mut Threshold, mut r: TIM3::Resources) {
    let dbg = &mut r.ITM.stim[0];

    iprintln!(dbg, "timeout");
    r.SENSOR
        .timedout()
        .expect("timeout handler: sensor in wrong state!");
    r.TMR.unlisten(Event::Update);
    *r.TS = None;
}
