# `BMP085-driver` [![Latest Version]][crates.io]

[Latest Version]: https://img.shields.io/crates/v/bmp085-driver.svg

A platform agnostic driver to interface the BOSCH BMP085 pressure sensor, written in Rust.

Please note that the BMP085 sensor has been discontinued in 2013.

This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/) traits.

## [API reference]

[API reference]: https://docs.rs/bmp085-driver

## What works

- Reading the calibration coefficients from the sensor eeprom
- Reading the uncompensated temperature from the sensor
- Converting the uncompensated temperature to d℃
- Reading the uncompensated pressure from the sensor
- Converting the uncompensated pressure to hPa
- Converting the pressure in hPa to hPa relative to altitude normal null (optional feature)

## License

Licensed under either of
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.
