//! A platform agnostic driver to interface with the
//! [LIS3MDL](https://www.st.com/en/mems-and-sensors/lis3mdl.html) (3-axis magnetic sensor).
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/0.2\

#![no_std]

use core::mem;
use embedded_hal as hal;
use crate::hal::blocking::i2c::{Write, WriteRead};

use bitflags::bitflags;

/// The full scale for measurement from 4 Gauss to 16 Gauss; Sensitivity of the sensor
#[derive(Debug)]
pub enum FullScale {
    Fs4g,
    Fs8g,
    Fs12g,
    Fs16g
}

/// The mode to operate in, impacts noise, power consumption, and speed
#[derive(Debug)]
pub enum OperatingMode {
    LowPower,
    MediumPerformance,
    HighPerformance,
    UltraHighPerformance
}

/// State of the LIS3MDL
#[derive(Debug)]
pub enum MeasurementMode {
    Idle,
    SingleMeasurement,
    Continuous
}

/// Possible data rates at which the xyz data can be provided
#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum DataRate {
    ODR_0_625Hz,
    ODR_1_25Hz,
    ODR_2_5Hz,
    ODR_5Hz,
    ODR_10Hz,
    ODR_20Hz,
    ODR_40Hz,
    ODR_80Hz,
    /// Fastest obtainable data rate for the given operating mode
    ODR_Fast
}

/// Driver Errors
#[derive(Debug)]
pub enum Error {
    /// Any issue with the I<sup>2</sup>C connection
    CommunicationError,
    /// An invalid value was found, should not happen
    InvalidValue,
    /// The wrong device was present when queried, checked once on driver setup
    IncorrectDeviceIdFound,
}

fn i2c_error<E>(_: E) -> Error {
    Error::CommunicationError
}

/// XYZ triple for raw values (int16)
#[derive(Debug)]
pub struct I16xyz {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// XYZ triple 32-bit float
#[derive(Debug)]
pub struct I32xyz {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

pub enum Address {
    Addr1E = 0x1E,
    Addr1C = 0x1C,
}

const LIS3MDL_DEVICE_ID: u8 = 0x3D;

/// LIS3MDL driver
pub struct Lis3mdl<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Lis3mdl<I2C> where I2C: WriteRead<Error=E> + Write<Error=E>,
{
    /// Create a new driver from an I<sup>2</sup>C peripheral and configures default settings:
    ///
    /// * Full Scale Range: 12G
    /// * Measurement Mode: Continuous Measurement
    /// * Operating Mode: Ultra High Performance
    /// * Enable the temperature sensor
    /// * Set data rate to fast
    /// * Enables Block Data Update
    ///
    /// These defaults may be changed after initialization with `set_full_scale`,
    /// `set_measurement_mode`, `set_operating_mode` `set_temperature_sensor`,
    /// `set_data_rate`, and `set_block_data_update`, respectively.
    pub fn new (i2c: I2C, addr: Address) -> Result<Self,Error> {
        let mut lis3mdl = Lis3mdl {
            i2c,
            address: addr as u8,
        };

        if lis3mdl.who_am_i()? != LIS3MDL_DEVICE_ID {
            return Err(Error::IncorrectDeviceIdFound)
        }

        lis3mdl.set_full_scale(FullScale::Fs12g)?;
        lis3mdl.set_operating_mode(OperatingMode::UltraHighPerformance)?;
        lis3mdl.set_measurement_mode(MeasurementMode::Continuous)?;
        lis3mdl.set_temperature_sensor_enable(true)?;
        lis3mdl.set_data_rate(DataRate::ODR_Fast)?;
        lis3mdl.set_block_data_update(true)?;

        Ok(lis3mdl)
    }

    /// Reads the WHO_AM_I register; should return `0x3D`
    pub fn who_am_i(&mut self) -> Result<u8, Error> {
        self.read_register(Register::WHO_AM_I).map_err(i2c_error)
    }

    /// Reads the XYZ components values of the magnetic field and returns the raw signed 16-bit
    /// integer value of each axis. This will return whatever is present in the data registers for
    /// each axis, so it is recommend to ensure that you are in the proper measurement mode and
    /// either synchronizing the read with interrupt/`xyz_data_available` or using
    /// `set_block_data_update`.
    ///
    /// To obtain the value in Gauss or milliGauss either use `get_mag_axes_mgauss` or divide by
    /// column 2 in the table below (obtained from Table 2 in AN4602 Rev 1):
    ///
    /// | Full-scale (G) | Gain@16-bit (LSB/Gauss) |
    /// |----------------|-------------------------|
    /// |        4       |           6842          |
    /// |        8       |           3421          |
    /// |        12      |           2281          |
    /// |        16      |           1711          |
    pub fn get_raw_mag_axes(&mut self) -> Result<I16xyz, Error> {

        let x = self.read_x_raw()?;
        let y = self.read_y_raw()?;
        let z = self.read_z_raw()?;

        Ok(I16xyz {
            x,
            y,
            z
        })
    }

    /// True if the XYZ data is available to be read
    pub fn xyz_data_available(&mut self) -> Result<bool, Error> {
        Ok(self.read_device_status()?.contains(StatusRegisterBits::ZYXDA))
    }

    /// Provide the magnetic field strength in each axis in milliGauss. Uses `get_raw_mag_axes` to
    /// obtain the value.
    pub fn get_mag_axes_mgauss(&mut self) -> Result<I32xyz, Error> {
        let mag_data = self.get_raw_mag_axes()?;

        let fullscale = FullScaleBits::from_bits(self.read_register(Register::CTRL_REG2)?).unwrap();

        // Gain values from Table 2 in AN4602 Rev 1
        let sensitivity: f64 = match fullscale {
            FullScaleBits::FS4G => Ok(1000_f64/6842_f64),
            FullScaleBits::FS8G => Ok(1000_f64/3421_f64),
            FullScaleBits::FS12G => Ok(1000_f64/2281_f64),
            FullScaleBits::FS16G => Ok(1000_f64/1711_f64),
            _ => Err(Error::InvalidValue)
        }?;

        Ok(I32xyz {
            x: (mag_data.x as f64 * sensitivity) as i32,
            y: (mag_data.y as f64 * sensitivity) as i32,
            z: (mag_data.z as f64 * sensitivity) as i32
        })
    }

    /// Set the Full Scale from between 4 Gauss and 16 Gauss to adjust the input dynamic range,
    /// based on the magnetic field to be measured. This will affect the output of
    /// `get_raw_mag_axes` so use `get_mag_axes_mgauss` unless you intend to adjust the values
    /// yourself.
    pub fn set_full_scale(&mut self, scale: FullScale) -> Result<(),Error> {
        // Mask for just the full scale bits.
        let fs_mask = !(ControlRegister2Bits::FS1 | ControlRegister2Bits::FS0);

        let fs = match scale {
            FullScale::Fs4g => FullScaleBits::FS4G,
            FullScale::Fs8g => FullScaleBits::FS8G,
            FullScale::Fs12g => FullScaleBits::FS12G,
            FullScale::Fs16g => FullScaleBits::FS16G,
        };

        // Zero out the full scale bits, we will replace them with the bitwise OR for the new setting
        let existing_settings = self.read_control_register_2()? & fs_mask;

        let reg2bits = ControlRegister2Bits::from_bits_truncate(fs.bits());

        // Update the full scale setting, preserving the other values
        self.set_control_register_2(reg2bits | existing_settings)
    }

    /// Adjust the operating mode. This will have an impact on current consumption, the max data
    /// rate, and the output noise with Ultra High Performance (UHP) being the slowest and highest
    /// current consumption with the lowest noise, and Low Power (LP) being the highest level of
    /// noise, but offering up to 1000 Hz data rate and lowest current consumption. See AN4602 for
    /// more details.
    pub fn set_operating_mode(&mut self, mode: OperatingMode) -> Result<(), Error> {
        // Masks for just the operating mode bits for X, Y, and Z axes
        let reg1_mask = !(ControlRegister1Bits::OM1 | ControlRegister1Bits::OM0);
        let reg4_mask = !(ControlRegister4Bits::OMZ1 | ControlRegister4Bits::OMZ0);

        let om = match mode {
            OperatingMode::LowPower => (ControlRegister1Bits::empty(), ControlRegister4Bits::empty()),
            OperatingMode::MediumPerformance => (ControlRegister1Bits::OM0, ControlRegister4Bits::OMZ0),
            OperatingMode::HighPerformance => (ControlRegister1Bits::OM1, ControlRegister4Bits::OMZ1),
            OperatingMode::UltraHighPerformance => (ControlRegister1Bits::OM1
                                                        | ControlRegister1Bits::OM0,
                                                    ControlRegister4Bits::OMZ1
                                                        | ControlRegister4Bits::OMZ0),
        };

        // zero out the entries for OM1/OM0 and OMZ1/OMZ0
        let existing_reg_1_settings = self.read_control_register_1()? & reg1_mask;
        let existing_reg_4_settings = self.read_control_register_4()? & reg4_mask;

        // Update the operating mode settings, preserving the other values
        self.set_control_register_1(existing_reg_1_settings | om.0)?;
        self.set_control_register_4(existing_reg_4_settings | om.1)?;
        Ok(())
    }

    /// Select between 3 measurement modes: Idle, Single Measurement, and Continuous. Configure to
    /// Idle if not being used, Single Measurement if only one measurement is desired, and Continuous
    /// if a constant stream of data is needed.
    pub fn set_measurement_mode(&mut self, mode: MeasurementMode) -> Result<(), Error> {
        // Mask for the measurement mode setting
        let mm_mask = !(ControlRegister3Bits::MD1 | ControlRegister3Bits::MD0);

        let mm = match mode {
            MeasurementMode::Idle => ControlRegister3Bits::MD1,
            MeasurementMode::SingleMeasurement => ControlRegister3Bits::MD0,
            MeasurementMode::Continuous => ControlRegister3Bits::empty(),
        };

        // zero out the entries for MD1 and MD0
        let existing_reg_3_settings = self.read_control_register_3()? & mm_mask;

        // Update the measurement mode settings, preserving the other values
        self.set_control_register_3(existing_reg_3_settings | mm)
    }

    /// Set the output data rate. Specific data rates from 0.625 Hz to 80 Hz can be configured for
    /// any given operating mode, and Fast will be the highest achievable data rate for the given
    /// operating mode at 1000 Hz for Low Power and 155 Hz for Ultra High Performance. See AN4602
    /// for more details.
    pub fn set_data_rate(&mut self, rate: DataRate) -> Result<(), Error> {
        // Mask for the data rate setting
        let odr_mask = !(ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1
            | ControlRegister1Bits::DO0 | ControlRegister1Bits::FAST_ODR);

        let odr = match rate {
            DataRate::ODR_0_625Hz => ControlRegister1Bits::empty(),
            DataRate::ODR_1_25Hz => ControlRegister1Bits::DO0,
            DataRate::ODR_2_5Hz => ControlRegister1Bits::DO1,
            DataRate::ODR_5Hz => ControlRegister1Bits::DO1 | ControlRegister1Bits::DO0,
            DataRate::ODR_10Hz => ControlRegister1Bits::DO2,
            DataRate::ODR_20Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO0,
            DataRate::ODR_40Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1,
            DataRate::ODR_80Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1
                                  | ControlRegister1Bits::DO0,
            DataRate::ODR_Fast => ControlRegister1Bits::FAST_ODR,
        };

        // zero out the entries for DO2, DO1, DO0, and FAST_ODR
        let existing_reg_1_settings = self.read_control_register_1()? & odr_mask;

        // Update the measurement mode settings, preserving other values
        self.set_control_register_1(existing_reg_1_settings | odr)
    }

    /// Blocks the refresh of data for a given axis until the initiated read for that axis
    /// completes. Strongly recommended if the reading of the magnetic data cannot be synchronized
    /// with the XYZDA in the status register. Ensures that data registers for each channel always
    /// contain the most recent magnetic data.
    pub fn set_block_data_update(&mut self, block: bool) -> Result<(), Error> {
        let bdu_mask = !ControlRegister5Bits::BDU;

        let existing_reg_5_settings = self.read_control_register_5()? & bdu_mask;

        if block {
            self.set_control_register_5(existing_reg_5_settings | ControlRegister5Bits::BDU)
        }
        else {
            self.set_control_register_5(existing_reg_5_settings)
        }
    }

    /// Enables the temperature sensor
    pub fn set_temperature_sensor_enable(&mut self, enabled: bool) -> Result<(), Error> {
        let temp_mask = !ControlRegister1Bits::TEMP_EN;

        let existing_reg_1_settings = self.read_control_register_1()? & temp_mask;

        if enabled {
            self.set_control_register_1(existing_reg_1_settings | ControlRegister1Bits::TEMP_EN)
        }
        else {
            self.set_control_register_1(existing_reg_1_settings)
        }
    }

    fn read_x_raw(&mut self) -> Result<i16,Error> {
        self.read_register_i16(Register::OUT_X_H,Register::OUT_X_L)
    }

    fn read_y_raw(&mut self) -> Result<i16,Error> {
        self.read_register_i16(Register::OUT_Y_H,Register::OUT_Y_L)
    }

    fn read_z_raw(&mut self) -> Result<i16,Error> {
        self.read_register_i16(Register::OUT_Z_H,Register::OUT_Z_L)
    }

    fn read_device_status(&mut self) -> Result<StatusRegisterBits, Error> {
        Ok(StatusRegisterBits::from_bits_truncate(self.read_register(Register::STATUS_REG)?))
    }

    fn set_control_register_1(&mut self, bits: ControlRegister1Bits) -> Result<(),Error> {
        Ok(self.write_register(Register::CTRL_REG1, bits.bits())?)
    }

    fn set_control_register_2(&mut self, bits: ControlRegister2Bits) -> Result<(),Error> {
        Ok(self.write_register(Register::CTRL_REG2, bits.bits())?)
    }

    fn set_control_register_3(&mut self, bits: ControlRegister3Bits) -> Result<(),Error> {
        Ok(self.write_register(Register::CTRL_REG3, bits.bits())?)
    }

    fn set_control_register_4(&mut self, bits: ControlRegister4Bits) -> Result<(),Error> {
        Ok(self.write_register(Register::CTRL_REG4, bits.bits())?)
    }

    fn set_control_register_5(&mut self, bits: ControlRegister5Bits) -> Result<(),Error> {
        Ok(self.write_register(Register::CTRL_REG5, bits.bits())?)
    }

    fn read_control_register_1(&mut self) -> Result<ControlRegister1Bits, Error> {
        Ok(ControlRegister1Bits::from_bits_truncate(self.read_register(Register::CTRL_REG1)?))
    }

    fn read_control_register_2(&mut self) -> Result<ControlRegister2Bits, Error> {
        Ok(ControlRegister2Bits::from_bits_truncate(self.read_register(Register::CTRL_REG2)?))
    }

    fn read_control_register_3(&mut self) -> Result<ControlRegister3Bits, Error> {
        Ok(ControlRegister3Bits::from_bits_truncate(self.read_register(Register::CTRL_REG3)?))
    }

    fn read_control_register_4(&mut self) -> Result<ControlRegister4Bits, Error> {
        Ok(ControlRegister4Bits::from_bits_truncate(self.read_register(Register::CTRL_REG4)?))
    }

    fn read_control_register_5(&mut self) -> Result<ControlRegister5Bits, Error> {
        Ok(ControlRegister5Bits::from_bits_truncate(self.read_register(Register::CTRL_REG5)?))
    }

    fn read_register_i16(&mut self, reg_low: Register, reg_high: Register) -> Result<i16, Error> {
        let low = self.read_register(reg_low)?;
        let high = self.read_register(reg_high)?;

        // Convert the low and high bytes to signed 16-bit integer
        let signed = i16::from_le_bytes([high, low]);
        Ok(signed)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Error> {
        let mut buffer: [u8; 1] = unsafe { mem::uninitialized() };
        self.i2c.write_read(self.address, &[reg.addr()], &mut buffer).map_err(i2c_error)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), Error> {
        self.i2c.write(self.address, &[reg.addr(), byte]).map_err(i2c_error)
    }
}

#[allow(non_camel_case_types, dead_code)]
#[derive(Debug, Copy, Clone)]
enum Register {
    OFFSET_X_REG_L_M = 0x05,
    OFFSET_X_REG_H_M = 0x06,
    OFFSET_Y_REG_L_M = 0x07,
    OFFSET_Y_REG_H_M = 0x08,
    OFFSET_Z_REG_L_M = 0x09,
    OFFSET_Z_REG_H_M = 0x0A,

    WHO_AM_I = 0x0F,

    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,

    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    TEMP_OUT_L = 0x2E,
    TEMP_OUT_H = 0x2F,
    INT_CFG = 0x30,
    INT_SRC = 0x31,
    INT_THS_L = 0x32,
    INT_THS_H = 0x33
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister1Bits: u8 {
    const TEMP_EN = 0b1000_0000;
    const OM1 = 0b0100_0000;
    const OM0 = 0b0010_0000;
    const DO2 = 0b0001_0000;
    const DO1 = 0b0000_1000;
    const DO0 = 0b0000_0100;
    const FAST_ODR = 0b0000_0010;
    const ST = 0b0000_0001;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister2Bits: u8 {
    const FS1 = 0b0100_0000;
    const FS0 = 0b0010_0000;
    const REBOOT = 0b0000_1000;
    const SOFT_RST = 0b0000_0100;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister3Bits: u8 {
    const LP = 0b0010_0000;
    const SIM = 0b0000_0100;
    const MD1 = 0b0000_0010;
    const MD0 = 0b0000_0001;
}
}

bitflags! {
struct ControlRegister4Bits: u8 {
    const OMZ1 = 0b0000_1000;
    const OMZ0 = 0b0000_0100;
    const BLE = 0b0000_0010;
    const UHP = 0b0000_1100;
}
}

bitflags! {
struct ControlRegister5Bits: u8 {
    const FAST_READ = 0b1000_0000;
    const BDU = 0b0100_0000;
}
}

bitflags! {
struct StatusRegisterBits: u8 {
    const ZYXOR = 0b1000_0000;
    const ZOR = 0b0100_0000;
    const YOR = 0b0010_0000;
    const XOR = 0b0001_0000;
    const ZYXDA = 0b0000_1000;
    const ZDA = 0b0000_0100;
    const YDA = 0b0000_0010;
    const XDA = 0b0000_0001;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct FullScaleBits: u8 {
    const FS4G = 0b0000_0000;
    const FS8G = 0b0010_0000;
    const FS12G = 0b0100_0000;
    const FS16G = 0b0110_0000;
}
}