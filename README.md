# `hc-sr04`
> A platform agnostic driver to interface with the HC-SR04 (ultrasonic distance)

## What works
- Estimating distance based on interrupt

## Examples
See the [`examples`][3] folder for usage. To find the dependencies of the examples
copy the `dev-dependencies` from `Cargo.toml`.

## TODO
- [x] Test on embedded target (tested on [`f3`][1], see example)
- [ ] Move to timers based purely on [`embedded-hal`][2]
- [ ] Find out why crate only seem to work in `--release` mode
- [ ] Test on single board computer (RPi etc.)
- [ ] Gather feedback on API

## License
Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

[1]: https://github.com/japaric/f3
[2]: https://github.com/japaric/embedded-hal/issues/59
[3]: examples/
