BMP085-driver
=============

[![Latest version](https://img.shields.io/crates/v/bmp085-driver.svg)](https://crates.io/crates/bmp085-driver)

A platform agnostic driver to interface the [Bosch Sensortec][Bosch Sensortec] BMP085 pressure sensor, written in Rust.

Please note that the BMP085 sensor has been [discontinued](https://media.digikey.com/pdf/PCNs/Bosch/BMP085_Disc.pdf) by Bosch Sensortec in 2013.

This driver is build using the [embedded-hal](https://docs.rs/embedded-hal/) traits.

[API reference]

## Features

- Reading the calibration coefficients from the sensor eeprom
- Reading the uncompensated temperature from the sensor
- Converting the uncompensated temperature to d℃
- Reading the uncompensated pressure from the sensor
- Converting the uncompensated pressure to hPa
- Converting the pressure in hPa to hPa relative to altitude normal null (optional feature)
- Tests for the conversion functions according to the data-sheet of the sensor


## License

Licensed under either of
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
at your option.

[API reference]: https://docs.rs/bmp085-driver
[Bosch Sensortec]: https://www.bosch-sensortec.com
