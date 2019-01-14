//! A platform agnostic driver to interface the BOSCH BMP085 sensor
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2

#![no_std]
#![deny(missing_docs)]
#![allow(clippy::trivially_copy_pass_by_ref, clippy::new_ret_no_self)]

extern crate cast;
extern crate embedded_hal as hal;
extern crate generic_array;

use core::mem;

use cast::{i16, i32, u16, u32};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};
use hal::blocking::delay::DelayUs;
use hal::blocking::i2c::{Write, WriteRead};

/// BMP085 module address. The LSB of the device address
/// distinguishes between read (1) and write (0) operation,
/// corresponding to address 0xEF (read) and 0xEE (write).
// embedded-hal/blocking/i2c uses 7-bit addresses.
#[allow(clippy::unreadable_literal)]
const ADDRESS: u8 = 0b1110111; // 7-bit I2C address, missing least significant r/w bit

/// BMP085 registers
#[allow(missing_docs)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
enum Register {
    COEFF_AC1 = 0xAA,
    COEFF_AC2 = 0xAC,
    COEFF_AC3 = 0xAE,
    COEFF_AC4 = 0xB0,
    COEFF_AC5 = 0xB2,
    COEFF_AC6 = 0xB4,
    COEFF_B1 = 0xB6,
    COEFF_B2 = 0xB8,
    COEFF_MB = 0xBA,
    COEFF_MC = 0xBC,
    COEFF_MD = 0xBE,
    CONTROL_REG = 0xF4,
    VALUE_REG = 0xF6,
}

#[allow(missing_docs)]
impl Register {
    pub fn addr(&self) -> u8 {
        *self as u8
    }
}

/// BMP085 control register values for different internal oversampling settings (osrs)
// Instead of waiting for the maximum conversion time, the output pin
// EOC (end of conversion) can be used to check if the conversion is
// finished (logic 1) or still running (logic 0).
#[allow(missing_docs)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone)]
enum ControlRegisterValue {
    CONTROL_VALUE_TEMPERATURE = 0x2E,     // Max. conversion time 4.5ms
    CONTROL_VALUE_PRESSURE_OSRS_0 = 0x34, // Max. conversion time 4.5ms
    CONTROL_VALUE_PRESSURE_OSRS_1 = 0x74, // Max. conversion time 7.5ms
    CONTROL_VALUE_PRESSURE_OSRS_2 = 0xB4, // Max. conversion time 13.5ms
    CONTROL_VALUE_PRESSURE_OSRS_3 = 0xF4, // Max. conversion time 25.5ms
}

#[allow(missing_docs)]
impl ControlRegisterValue {
    pub fn value(&self) -> u8 {
        *self as u8
    }
}

/// Calibration coefficients from BMP085 eeprom
struct Coefficients {
    pub ac1: i16,
    pub ac2: i16,
    pub ac3: i16,
    pub ac4: u16,
    pub ac5: u16,
    pub ac6: u16,
    pub b1: i16,
    pub b2: i16,
    pub mb: i16,
    pub mc: i16,
    pub md: i16,
}

impl Coefficients {
    fn new() -> Self {
        Coefficients {
            ac1: 0,
            ac2: 0,
            ac3: 0,
            ac4: 0,
            ac5: 0,
            ac6: 0,
            b1: 0,
            b2: 0,
            mb: 0,
            mc: 0,
            md: 0,
        }
    }
}

/// Oversampling modes
#[allow(missing_docs)]
#[derive(Copy, Clone)]
pub enum Oversampling {
    /// Number of samples 1, conversion time max 4.5ms, average current 3µA
    UltraLowPower = 0,
    /// Number of samples 2, conversion time max 7.5ms, average current 5µA
    Standard = 1,
    /// Number of samples 4, conversion time max 13.5ms, average current 7µA
    HighResolution = 2,
    /// Number of samples 8, conversion time max. 22.5ms, average current 12µA
    UltraHighResolution = 3,
}

impl Oversampling {
    fn value(&self) -> u8 {
        *self as u8
    }
}

/// BMP085 driver
pub struct Bmp085<I2C, TIMER: DelayUs<u16>> {
    i2c: I2C,
    timer: TIMER,
    coeff: Coefficients,
    oss: Oversampling,
}

/// Temperature as deci celcius
pub type DeciCelcius = i32;

/// Pressure in pascal
pub type Pascal = i32;

// Type of intermediate value from temperature calculation. Used in
// pressure calculation after sensor read-out.
type B5 = i32;

impl<I2C, TIMER, E> Bmp085<I2C, TIMER>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    TIMER: DelayUs<u16>,
{
    /// Create a new driver from a I2C peripheral
    pub fn new(i2c: I2C, timer: TIMER, oss: Oversampling) -> Result<Self, E> {
        let mut bmp085 = Bmp085 {
            i2c,
            timer,
            coeff: Coefficients::new(),
            oss,
        };

        // Get calibration coefficients from the BMP085 eeprom
        bmp085.coeff.ac1 = bmp085.read_i16(Register::COEFF_AC1)?;
        bmp085.coeff.ac2 = bmp085.read_i16(Register::COEFF_AC2)?;
        bmp085.coeff.ac3 = bmp085.read_i16(Register::COEFF_AC3)?;
        bmp085.coeff.ac4 = bmp085.read_u16(Register::COEFF_AC4)?;
        bmp085.coeff.ac5 = bmp085.read_u16(Register::COEFF_AC5)?;
        bmp085.coeff.ac6 = bmp085.read_u16(Register::COEFF_AC6)?;
        bmp085.coeff.b1 = bmp085.read_i16(Register::COEFF_B1)?;
        bmp085.coeff.b2 = bmp085.read_i16(Register::COEFF_B2)?;
        bmp085.coeff.mb = bmp085.read_i16(Register::COEFF_MB)?;
        bmp085.coeff.mc = bmp085.read_i16(Register::COEFF_MC)?;
        bmp085.coeff.md = bmp085.read_i16(Register::COEFF_MD)?;

        Ok(bmp085)
    }

    fn read_u16(&mut self, reg: Register) -> Result<u16, E> {
        let buf: GenericArray<u8, U2> = self.read_register(reg)?;
        Ok((u16(buf[0]) << 8) + u16(buf[1]))
    }

    fn read_i16(&mut self, reg: Register) -> Result<i16, E> {
        let buf: GenericArray<u8, U2> = self.read_register(reg)?;
        Ok((i16(buf[0]) << 8) + i16(buf[1]))
    }

    fn read_register<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::uninitialized() };

        {
            let buffer: &mut [u8] = &mut buffer;

            self.i2c.write_read(ADDRESS, &[reg.addr()], buffer)?;
        }

        Ok(buffer)
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        self.i2c.write(ADDRESS, &[reg.addr(), byte])
    }

    /// Read temperature and pressure from sensor
    pub fn read(&mut self) -> Result<PT, E> {
        // Read temperature
        self.write_register(
            Register::CONTROL_REG,
            ControlRegisterValue::CONTROL_VALUE_TEMPERATURE.value(),
        )?;
        // Wait 4.5ms or wait for the output pin EOC is finished (logic 1)
        self.timer.delay_us(4500);
        let ut: i16 = self.read_i16(Register::VALUE_REG)?; // TODO i32 or i16?
        let (temperature, b5) = calculate_temperature(i32(ut), &self.coeff);

        // Read pressure
        let pressure_kind = match self.oss {
            Oversampling::UltraLowPower => ControlRegisterValue::CONTROL_VALUE_PRESSURE_OSRS_0,
            Oversampling::Standard => ControlRegisterValue::CONTROL_VALUE_PRESSURE_OSRS_1,
            Oversampling::HighResolution => ControlRegisterValue::CONTROL_VALUE_PRESSURE_OSRS_2,
            Oversampling::UltraHighResolution => {
                ControlRegisterValue::CONTROL_VALUE_PRESSURE_OSRS_3
            }
        };
        self.write_register(Register::CONTROL_REG, pressure_kind.value())?;
        self.timer.delay_us(1000 * (2 + (3 << self.oss.value())));

        let up: GenericArray<u8, U3> = self.read_register(Register::VALUE_REG)?;
        let up: i32 = (i32(up[0]) << 16) + (i32(up[1]) << 8) + i32(up[2]);
        let pressure: Pascal = calculate_true_pressure(up, b5, self.oss, &self.coeff);

        Ok(PT {
            temperature,
            pressure,
        })
    }
}

/// Result of the BMP085 sensor read-out
#[allow(dead_code)]
pub struct PT {
    temperature: DeciCelcius,
    pressure: Pascal,
}

fn calculate_temperature(ut: i32, coeff: &Coefficients) -> (DeciCelcius, B5) {
    let x1: i32 = ((ut - i32(coeff.ac6)) * i32(coeff.ac5)) >> 15;
    let x2: i32 = (i32(coeff.mc) << 11) / (x1 + i32(coeff.md));
    let b5: i32 = x1 + x2; // Value b5 is used in pressure calculation
    let t: DeciCelcius = (b5 + 8) >> 4;
    (t, b5)
}

fn calculate_true_pressure(up: i32, b5: B5, oss: Oversampling, coeff: &Coefficients) -> Pascal {
    let b6: i32 = b5 - 4000;

    // B3
    let x1: i32 = ((i32(coeff.b2) * (b6 * b6)) >> 12) >> 11;
    let x2: i32 = (i32(coeff.ac2) * b6) >> 11;
    let x3: i32 = x1 + x2;
    let b3: i32 = (((i32(coeff.ac1) * 4 + x3) << oss.value()) + 2) >> 2;

    // B4
    let x1: i32 = (i32(coeff.ac3) * b6) >> 13;
    let x2: i32 = (i32(coeff.b1) * ((b6 * b6) >> 12)) >> 16;
    let x3: i32 = (x1 + x2 + 2) >> 2;
    let b4: u32 = (u32(coeff.ac4) * ((x3 as u32) + 32768)) >> 15;

    // B7 (use u32 instead of long, differs from datasheet)
    let b7: u32 = ((up - b3) as u32) * (50000u32 >> oss.value());

    let p: i32 = if b7 < 0x8000_0000 {
        b7 * 2 / b4
    } else {
        b7 / b4 * 2
    } as i32;

    let x1: i32 = (p >> 8) * (p >> 8);
    let x1: i32 = (x1 * 3038) >> 16;
    let x2: i32 = (-7357 * p) >> 16;
    p + ((x1 + x2 + 3791) >> 4)
}

#[cfg(test)]
mod tests {

    use super::{
        calculate_temperature, calculate_true_pressure, Coefficients, Oversampling, Pascal,
    };

    #[test]
    fn calculate_temp_pressure() {
        // Coefficient values according datasheet
        let coeff = Coefficients {
            ac1: 408,
            ac2: -72,
            ac3: -14383,
            ac4: 32741,
            ac5: 32757,
            ac6: 23153,
            b1: 6190,
            b2: 4,
            mb: -32767,
            mc: -8711,
            md: 2868,
        };

        let ut: i32 = 27898; // Uncompensated temperature from sensor
        let up: i32 = 23843; // Uncompensated pressure from sensor
        let oss = Oversampling::UltraLowPower;
        assert_eq!(oss.value(), 0);

        let (deci_c, b5) = calculate_temperature(ut, &coeff);
        assert_eq!(deci_c, 150);
        assert_eq!(b5, 2400); // Differs from datasheet example, but imho ok

        let pressure: Pascal = calculate_true_pressure(up, b5, oss, &coeff);
        assert_eq!(pressure, 69964);
    }
}
