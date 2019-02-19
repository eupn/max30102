#![no_std]
#![allow(dead_code)]

///! High-Sensitivity Pulse Oximeter and Heart-Rate Sensor for Wearable Health
///! Datasheet: https://datasheets.maximintegrated.com/en/ds/MAX30102.pdf
use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),

    /// Invalid slot number.
    InvalidSlotNumber,
}

pub struct Max30102<I> {
    i2c: I,
}

impl<I, E> Max30102<I>
    where
        I: WriteRead<Error = E> + Write<Error = E>,
{
    /// Side-effect-free constructor.
    /// Nothing will be read or written before `init()` call.
    pub fn new(i2c: I) -> Self {
        let max = Max30102 {
            i2c,
        };

        max
    }

    /// Initializes the MAX30102 device.
    pub fn init(&mut self) -> Result<(), Error<E>> {
        let id = self.chip_id()?;
        if id != MAX30102_EXPECTED_CHIP_ID {
            return Err(Error::InvalidChipId(id))
        }

        //self.soft_reset()?;

        self.set_fifo_average(MAX30102_SAMPLEAVG_4)?;

        self.set_fifo_rollover(true)?;

        self.set_led_mode(MAX30102_MODE_MULTILED)?;

        self.set_adc_range(MAX30102_ADCRANGE_4096)?;

        self.set_sample_rate(MAX30102_SAMPLERATE_400)?;

        self.set_pulse_width(MAX30102_PULSEWIDTH_411)?;

        self.set_pulse_amplitude_red(0x7f)?;
        self.set_pulse_amplitude_ir(0x7f)?;

        self.enable_slot(1, SLOT_RED_LED)?;
        self.enable_slot(2, SLOT_IR_LED)?;

        self.clear_fifo()?;

        Ok(())
    }

    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET)
    }

    /// Returns ID of the chip.
    pub fn chip_id(&mut self) -> Result<u8, Error<E>> {
        let id = self
            .read_u8(MAX30102_PARTID)
            .map_err(Error::I2c)?;

        Ok(id)
    }

    /// Sets sample average (Table 3, Page 18).
    pub fn set_fifo_average(&mut self, num_samples: u8) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, num_samples)
    }

    /// Enables or disables FIFO roll-over on overflow.
    pub fn set_fifo_rollover(&mut self, enable: bool) -> Result<(), Error<E>> {
        let value = if enable { MAX30102_ROLLOVER_ENABLE } else { MAX30102_ROLLOVER_DISABLE };
        self.rmw_mask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, value)
    }

    /// Sets which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    /// See datasheet, page 19.
    pub fn set_led_mode(&mut self, mode: u8) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode)
    }

    /// Sets ADC range.
    pub fn set_adc_range(&mut self, range: u8) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, range)
    }

    /// Sets ADC sample rate.
    pub fn set_sample_rate(&mut self, rate: u8) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, rate)
    }

    /// Sets LED pulse width.
    pub fn set_pulse_width(&mut self, width: u8) -> Result<(), Error<E>> {
        self.rmw_mask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, width)
    }

    /// Sets red LED pulse amplitude.
    pub fn set_pulse_amplitude_red(&mut self, amplitude: u8) -> Result<(), Error<E>> {
        self
            .write_u8(MAX30102_LED1_PULSEAMP, amplitude)
            .map_err(Error::I2c)
    }

    /// Sets IR LED pulse amplitude.
    pub fn set_pulse_amplitude_ir(&mut self, amplitude: u8) -> Result<(), Error<E>> {
        self
            .write_u8(MAX30102_LED2_PULSEAMP, amplitude)
            .map_err(Error::I2c)
    }

    /// Sets time slot for certain LED device.
    pub fn enable_slot(&mut self, slot: u8, device: u8) -> Result<(), Error<E>> {
        let (reg, mask, value) = match slot {
            0 => (MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device),
            1 => (MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4),
            2 => (MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device),
            3 => (MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4),

            _ => return Err(Error::InvalidSlotNumber)
        };

        self.rmw_mask(reg, mask, value)
    }

    /// Clears on-chip samples FIFO.
    pub fn clear_fifo(&mut self) -> Result<(), Error<E>> {
        self.write_u8(MAX30102_FIFOWRITEPTR, 0).map_err(Error::I2c)?;
        self.write_u8( MAX30102_FIFOOVERFLOW, 0).map_err(Error::I2c)?;
        self.write_u8( MAX30102_FIFOREADPTR, 0).map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns last sample from on-chip FIFO or None.
    pub fn read_fifo(&mut self) -> Result<Option<(u32, u32)>, Error<E>> {
        Ok(if self.fifo_samples_available()? > 0 {
            let mut buf = [0u8; 6];

            self
                .read_bytes(MAX30102_FIFODATA, &mut buf)
                .map_err(Error::I2c)?;

            let led1 = (buf[0] as u32) << 16 | (buf[1] as u32) << 8 | (buf[2] as u32);
            let led2 = (buf[3] as u32) << 16 | (buf[4] as u32) << 8 | (buf[5] as u32);

            // Use only 18 bits
            Some((led1 & 0x3FFFF, led2 & 0x3FFFF))
        } else {
            None
        })
    }

    /// Returns number of samples available in FIFO.
    pub fn fifo_samples_available(&mut self) -> Result<u8, Error<E>> {
        // Read FIFO read and write pointer
        let rp = self.read_u8(MAX30102_FIFOREADPTR).map_err(Error::I2c)?;
        let wp = self.read_u8(MAX30102_FIFOWRITEPTR).map_err(Error::I2c)?;

        let mut num_samples = (wp - rp) as isize;
        if num_samples < 0 {
            num_samples += 32;
        }

        Ok(num_samples as u8)
    }

    /// Reads-Modifies-Writes a register by given mask and value.
    fn rmw_mask(&mut self, reg: u8, mask: u8, value: u8) -> Result<(), Error<E>> {
        let mut data = self.read_u8(reg).map_err(Error::I2c)?;

        data &= mask;
        data |= value;

        self
            .write_u8(reg, data)
            .map_err(Error::I2c)?;

        Ok(())
    }

    fn read_u8(&mut self, reg: u8) -> Result<u8, E> {
        let mut byte: [u8; 1] = [0; 1];

        match self.i2c.write_read(MAX30102_I2C_ADDR, &[reg], &mut byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(MAX30102_I2C_ADDR, &[reg], buf)
    }

    fn write_u8(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(MAX30102_I2C_ADDR, &[reg, value])?;

        Ok(())
    }
}

// -- Constants --

const MAX30102_I2C_ADDR: u8 = 0x57;

// Status Registers
const MAX30102_INTSTAT1: u8 =0x00;
const MAX30102_INTSTAT2: u8 =0x01;
const MAX30102_INTENABLE1: u8 =0x02;
const MAX30102_INTENABLE2: u8 =0x03;

// FIFO Registers
const MAX30102_FIFOWRITEPTR: u8 = 0x04;
const MAX30102_FIFOOVERFLOW: u8 = 0x05;
const MAX30102_FIFOREADPTR: u8 = 0x06;
const MAX30102_FIFODATA: u8 =0x07;

// Configuration Registers
const MAX30102_FIFOCONFIG: u8 = 0x08;
const MAX30102_MODECONFIG: u8 = 0x09;
const MAX30102_PARTICLECONFIG: u8 = 0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (page. 11)
const MAX30102_LED1_PULSEAMP: u8 = 0x0C;
const MAX30102_LED2_PULSEAMP: u8 = 0x0D;
const MAX30102_LED3_PULSEAMP: u8 = 0x0E;
const MAX30102_LED_PROX_AMP: u8 = 0x10;
const MAX30102_MULTILEDCONFIG1: u8 = 0x11;
const MAX30102_MULTILEDCONFIG2: u8 = 0x12;

// Die Temperature Registers
const MAX30102_DIETEMPINT: u8 = 0x1F;
const MAX30102_DIETEMPFRAC: u8 = 0x20;
const MAX30102_DIETEMPCONFIG: u8 = 0x21;

// Proximity Function Registers
const MAX30102_PROXINTTHRESH: u8 = 0x30;

// Part ID Registers
const MAX30102_REVISIONID: u8 = 0xFE;
const MAX30102_PARTID: u8 = 0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
const MAX30102_INT_A_FULL_MASK: u8 = !0b10000000;
const MAX30102_INT_A_FULL_ENABLE: u8 = 0x80;
const MAX30102_INT_A_FULL_DISABLE: u8 = 0x00;

const MAX30102_INT_DATA_RDY_MASK: u8 = !0b01000000;
const MAX30102_INT_DATA_RDY_ENABLE: u8 =0x40;
const MAX30102_INT_DATA_RDY_DISABLE: u8 = 0x00;

const MAX30102_INT_ALC_OVF_MASK: u8 = !0b00100000;
const MAX30102_INT_ALC_OVF_ENABLE: u8 = 0x20;
const MAX30102_INT_ALC_OVF_DISABLE: u8 = 0x00;

const MAX30102_INT_PROX_INT_MASK: u8 = !0b00010000;
const MAX30102_INT_PROX_INT_ENABLE: u8 = 0x10;
const MAX30102_INT_PROX_INT_DISABLE: u8 = 0x00;

const MAX30102_INT_DIE_TEMP_RDY_MASK: u8 = !0b00000010;
const MAX30102_INT_DIE_TEMP_RDY_ENABLE: u8 = 0x02;
const MAX30102_INT_DIE_TEMP_RDY_DISABLE: u8 = 0x00;

const MAX30102_SAMPLEAVG_MASK: u8 =!0b11100000;
const MAX30102_SAMPLEAVG_1: u8 = 0x00;
const MAX30102_SAMPLEAVG_2: u8 = 0x20;
const MAX30102_SAMPLEAVG_4: u8 = 0x40;
const MAX30102_SAMPLEAVG_8: u8 = 0x60;
const MAX30102_SAMPLEAVG_16: u8 = 0x80;
const MAX30102_SAMPLEAVG_32: u8 = 0xA0;

const MAX30102_ROLLOVER_MASK: u8 = 0xEF;
const MAX30102_ROLLOVER_ENABLE: u8 = 0x10;
const MAX30102_ROLLOVER_DISABLE: u8 = 0x00;

const MAX30102_A_FULL_MASK: u8 = 0xF0;

// Mode configuration commands (page 19)
const MAX30102_SHUTDOWN_MASK: u8 = 0x7F;
const MAX30102_SHUTDOWN: u8 = 0x80;
const MAX30102_WAKEUP: u8 = 0x00;

const MAX30102_RESET_MASK: u8 = 0xBF;
const MAX30102_RESET: u8 = 0x40;

const MAX30102_MODE_MASK: u8 = 0xF8;
const MAX30102_MODE_REDONLY: u8 = 0x02;
const MAX30102_MODE_REDIRONLY: u8 = 0x03;
const MAX30102_MODE_MULTILED: u8 = 0x07;

// Particle sensing configuration commands (pages 19-20)
const MAX30102_ADCRANGE_MASK: u8 = 0x9F;
const MAX30102_ADCRANGE_2048: u8 = 0x00;
const MAX30102_ADCRANGE_4096: u8 = 0x20;
const MAX30102_ADCRANGE_8192: u8 = 0x40;
const MAX30102_ADCRANGE_16384: u8 = 0x60;

const MAX30102_SAMPLERATE_MASK: u8 = 0xE3;
const MAX30102_SAMPLERATE_50: u8 = 0x00;
const MAX30102_SAMPLERATE_100: u8 = 0x04;
const MAX30102_SAMPLERATE_200: u8 = 0x08;
const MAX30102_SAMPLERATE_400: u8 = 0x0C;
const MAX30102_SAMPLERATE_800: u8 = 0x10;
const MAX30102_SAMPLERATE_1000: u8 = 0x14;
const MAX30102_SAMPLERATE_1600: u8 = 0x18;
const MAX30102_SAMPLERATE_3200: u8 = 0x1C;

const MAX30102_PULSEWIDTH_MASK: u8 = 0xFC;
const MAX30102_PULSEWIDTH_69: u8 = 0x00;
const MAX30102_PULSEWIDTH_118: u8 = 0x01;
const MAX30102_PULSEWIDTH_215: u8 = 0x02;
const MAX30102_PULSEWIDTH_411: u8 = 0x03;

// Multi-LED Mode configuration (page 22)
const MAX30102_SLOT1_MASK: u8 = 0xF8;
const MAX30102_SLOT2_MASK: u8 = 0x8F;
const MAX30102_SLOT3_MASK: u8 = 0xF8;
const MAX30102_SLOT4_MASK: u8 = 0x8F;

const SLOT_NONE: u8 = 0x00;
const SLOT_RED_LED: u8 = 0x01;
const SLOT_IR_LED: u8 = 0x02;
const SLOT_GREEN_LED: u8 = 0x03;
const SLOT_NONE_PILOT: u8 = 0x04;
const SLOT_RED_PILOT: u8 =0x05;
const SLOT_IR_PILOT: u8 = 0x06;
const SLOT_GREEN_PILOT: u8 = 0x07;

const MAX30102_EXPECTED_CHIP_ID: u8 = 0x15;