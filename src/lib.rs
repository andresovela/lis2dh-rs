//! Driver crate for the ST [LIS2DH12] accelerometer.
//! Compatible with [embedded-hal] and [embedded-hal-async] traits.
//!
//! # Example usage
//! ```ignore
//! let mut accelerometer = Lis2dh::new(i2c, Sa0Pad::High);
//!
//! accelerometer.set_mode(Mode::Normal).await.unwrap();
//! accelerometer.set_output_data_rate(OutputDataRate::Hz100).await.unwrap();
//!
//! let int1_config = Int1Config::FifoWatermark;
//! accelerometer.configure_int1(&int1_config).await.unwrap();
//!
//! accelerometer.configure_fifo(FifoConfig::Stream { watermark: 9 }).await.unwrap();
//! accelerometer.enable_fifo(true).await.unwrap();
//!
//! let mut data = [AccelerationData::default(); 10];
//! loop {
//!     // Wait until the accelerometer fills the FIFO
//!     accelerometer_int1.wait_for_high().await;
//!     accelerometer.read_data(&mut data).await.unwrap();
//! }
//! ```
//!
//! [LIS2DH12]: https://www.st.com/en/mems-and-sensors/lis2dh12.html
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/

#![no_std]
#![deny(missing_docs)]

mod register;
pub use register::*;

/// Driver for the LIS2DH accelerometer
pub struct Lis2dh<I2C> {
    i2c: I2C,
    addr: u8,
}

/// Error type
#[derive(Debug)]
pub enum Error<I2cError> {
    /// I2C bus error
    I2c(I2cError),
    /// Attempted to write to a read-only register
    WriteToReadOnly,
    /// Invalid parameter
    InvalidParameter,
}

impl<I2C, E> Lis2dh<I2C>
where
    I2C: embedded_hal_async::i2c::I2c + embedded_hal::i2c::ErrorType<Error = E>,
{
    /// Creates a new driver instance for the LIS2DH
    ///
    /// The device's I2C address is dependent on how the
    /// SA0 pad of the device is connected on the board
    pub fn new(i2c: I2C, addr_pad: Sa0Pad) -> Self {
        let addr = 0x18 | addr_pad as u8;

        Self { i2c, addr }
    }

    /// Reads the accelerometer's  device ID
    /// It should read 0x33
    pub async fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::WhoAmI).await
    }

    /// Reboots the accelometer's memory content
    pub async fn reboot_memory_content(&mut self) -> Result<(), Error<E>> {
        self.write_register(Register::CtrlReg5, BOOT).await
    }

    /// Sets the device's operating mode
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::LowPower => {
                self.set_register_bits(Register::CtrlReg1, LPEN).await?;
                self.clear_register_bits(Register::CtrlReg4, HR).await?;
            }
            Mode::Normal => {
                self.clear_register_bits(Register::CtrlReg1, LPEN).await?;
                self.clear_register_bits(Register::CtrlReg4, HR).await?;
            }
            Mode::HighResolution => {
                self.clear_register_bits(Register::CtrlReg1, LPEN).await?;
                self.set_register_bits(Register::CtrlReg4, HR).await?;
            }
        };
        Ok(())
    }

    /// Sets the accelerometer's full scale
    pub async fn set_full_scale(&mut self, fs: FullScale) -> Result<(), Error<E>> {
        self.clear_register_bits(Register::CtrlReg4, FS_MASK)
            .await?;
        self.set_register_bits(Register::CtrlReg4, fs as u8).await
    }

    /// Sets the output data rate (ODR) of the accelerometer
    /// The ODR determines how often the accelerometer updates its readings
    pub async fn set_output_data_rate(&mut self, odr: OutputDataRate) -> Result<(), Error<E>> {
        self.modify_register(Register::CtrlReg1, |v| (v & !ODR_MASK) | odr as u8)
            .await?;

        // From the datasheet:
        // By design, when the device from high-resolution configuration (HR) is set to power-down
        // mode (PD), it is recommended to read register REFERENCE (26h) for a complete reset of
        // the filtering block before switching to normal/high-performance mode again for proper
        // device functionality
        if matches!(odr, OutputDataRate::PowerDown) {
            self.read_register(Register::Reference).await?;
        }

        Ok(())
    }

    /// Reads the status registers of the accelerometer
    pub async fn read_status(&mut self) -> Result<Status, Error<E>> {
        let sr = self.read_register(Register::StatusReg).await?;
        let sr_aux = self.read_register(Register::StatusRegAux).await?;

        Ok(Status {
            xyz_overrun: (sr & ZYXOR != 0),
            z_overrun: (sr & ZOR != 0),
            y_overrun: (sr & YOR != 0),
            x_overrun: (sr & XOR != 0),
            xyz_data_available: (ZYXDA & 0x08 != 0),
            z_data_available: (ZDA & 0x04 != 0),
            y_data_available: (YDA & 0x02 != 0),
            x_data_available: (XDA & 0x01 != 0),
            temp_overrun: (sr_aux & TOR != 0),
            temp_data_available: (sr_aux & TDA != 0),
        })
    }

    /// Enables/disables the accelerometer's FIFO
    pub async fn enable_fifo(&mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.set_register_bits(Register::CtrlReg5, FIFO_EN).await
        } else {
            self.clear_register_bits(Register::CtrlReg5, FIFO_EN).await
        }
    }

    /// Configures the accelerometer's FIFO
    pub async fn configure_fifo(&mut self, config: FifoConfig) -> Result<(), Error<E>> {
        let value = match config {
            FifoConfig::Bypass => 0x00,
            FifoConfig::Fifo => 0x40,
            FifoConfig::StreamToFifo { pin } => match pin {
                IntPin::Int1 => 0xC0,
                IntPin::Int2 => 0xE0,
            },
            FifoConfig::Stream { watermark } => {
                if watermark >= 0x20 {
                    return Err(Error::InvalidParameter);
                }
                0x80 + watermark
            }
        };
        self.write_register(Register::FifoCtrlReg, value).await
    }

    /// Reads the status of the FIFO
    pub async fn read_fifo_status(&mut self) -> Result<FifoStatus, Error<E>> {
        let fsr = self.read_register(Register::FifoSrcReg).await?;

        Ok(FifoStatus {
            watermark_exceeded: (fsr & WTM != 0),
            full: (fsr & OVRN_FIFO != 0),
            empty: (fsr & EMPTY != 0),
            sample_count: (fsr & FSS_MASK),
        })
    }

    /// Reads samples from the accelerometer's FIFO into the given array of data
    ///
    /// Care must be taken to ensure that this function does not read more data than
    /// the FIFO has available. This function provides no protection against that
    pub async fn read_data(&mut self, data: &mut [AccelerationData]) -> Result<(), Error<E>> {
        for i in 0..data.len() {
            self.read_from_fifo(&mut data[i]).await?;
        }

        Ok(())
    }

    /// Read all 6 OUT registers and write the data to the given sample
    async fn read_from_fifo(&mut self, sample: &mut AccelerationData) -> Result<(), Error<E>> {
        // From the datasheet:
        // An 8-bit sub-address (SUB) is transmitted:
        // the 7 LSb represent the actual register address while the MSb enables address auto increment.
        // If the MSb of the SUB field is ‘1’, the SUB (register address) is automatically increased to
        // allow multiple data read/writes.
        let reg_addr = Register::OutXL as u8 | 0x80;
        let mut buffer = [0; 6];
        self.i2c
            .write_read(self.addr, &[reg_addr], &mut buffer)
            .await
            .map_err(Error::I2c)?;

        sample.x = i16::from_le_bytes([buffer[0], buffer[1]]);
        sample.y = i16::from_le_bytes([buffer[2], buffer[3]]);
        sample.z = i16::from_le_bytes([buffer[4], buffer[5]]);

        Ok(())
    }

    /// Configures the functionality of pin INT1
    pub async fn configure_int1(&mut self, config: &Int1Config) -> Result<(), Error<E>> {
        match config {
            Int1Config::Unused => {
                self.write_register(Register::CtrlReg3, 0x00).await?;
            }
            Int1Config::Click(config) => {
                self.enable_click_interrupt(config).await?;
                self.write_register(Register::CtrlReg3, I1_CLICK).await?;
            }
            Int1Config::MovementInt1(config) => {
                self.enable_movement_interrupt(Int1, config).await?;
                self.write_register(Register::CtrlReg3, I1_IA1).await?;
            }
            Int1Config::MovementInt2(config) => {
                self.enable_movement_interrupt(Int2, config).await?;
                self.write_register(Register::CtrlReg3, I1_IA2).await?;
            }
            Int1Config::DataAvailableXYZ => {
                self.write_register(Register::CtrlReg3, I1_ZYXDA).await?;
            }
            Int1Config::FifoWatermark => {
                self.write_register(Register::CtrlReg3, I1_WTM).await?;
            }
            Int1Config::FifoOverrun => {
                self.write_register(Register::CtrlReg3, I1_OVERRUN).await?;
            }
        }

        Ok(())
    }

    /// Configures the functionality of pin INT2
    pub async fn configure_int2(&mut self, config: &Int2Config) -> Result<(), Error<E>> {
        self.clear_register_bits(Register::CtrlReg6, I2_MASK)
            .await?;
        match config {
            Int2Config::Unused => {
                self.set_register_bits(Register::CtrlReg6, 0x00).await?;
            }
            Int2Config::Click(config) => {
                self.enable_click_interrupt(config).await?;
                self.set_register_bits(Register::CtrlReg6, I2_CLICK).await?;
            }
            Int2Config::MovementInt1(config) => {
                self.enable_movement_interrupt(Int1, config).await?;
                self.set_register_bits(Register::CtrlReg6, I2_IA1).await?;
            }
            Int2Config::MovementInt2(config) => {
                self.enable_movement_interrupt(Int2, config).await?;
                self.set_register_bits(Register::CtrlReg6, I2_IA2).await?;
            }
            Int2Config::Boot => {
                self.set_register_bits(Register::CtrlReg6, I2_BOOT).await?;
            }
            Int2Config::Activity(config) => {
                self.enable_activity_interrupt(config).await?;
                self.set_register_bits(Register::CtrlReg6, I2_ACT).await?;
            }
        }

        Ok(())
    }

    /// Sets the interrupt polarity for both INT1 and INT2
    pub async fn set_interrupt_polarity(&mut self, polarity: IntPolarity) -> Result<(), Error<E>> {
        match polarity {
            IntPolarity::ActiveHigh => {
                self.clear_register_bits(Register::CtrlReg6, INT_POLARITY)
                    .await
            }
            IntPolarity::ActiveLow => {
                self.set_register_bits(Register::CtrlReg6, INT_POLARITY)
                    .await
            }
        }
    }

    /// Enables the movement interrupt
    async fn enable_movement_interrupt<T: IntRegisters>(
        &mut self,
        int: T,
        config: &MovementIntConfig,
    ) -> Result<(), Error<E>> {
        if config.threshold >= 0x80 || config.duration >= 0x80 {
            return Err(Error::InvalidParameter);
        }

        let mut cfg = config.mode as u8;
        cfg |= (config.enable.z_high as u8) << 5;
        cfg |= (config.enable.z_low as u8) << 4;
        cfg |= (config.enable.y_high as u8) << 3;
        cfg |= (config.enable.y_low as u8) << 2;
        cfg |= (config.enable.x_high as u8) << 1;
        cfg |= config.enable.x_low as u8;

        if config.latch {
            self.set_register_bits(Register::CtrlReg5, T::LIR_BIT)
                .await?;
        } else {
            self.clear_register_bits(Register::CtrlReg5, T::LIR_BIT)
                .await?;
        }

        if config.only_4d {
            self.set_register_bits(Register::CtrlReg5, T::D4D_BIT)
                .await?;
        } else {
            self.clear_register_bits(Register::CtrlReg5, T::D4D_BIT)
                .await?;
        }

        self.write_register(int.cfg(), cfg).await?;
        self.write_register(int.ths(), config.threshold).await?;
        self.write_register(int.dur(), config.duration).await?;

        Ok(())
    }

    /// Reads the source of the movement interrupt
    pub async fn read_movement_interrupt_source(
        &mut self,
        int: impl IntRegisters,
    ) -> Result<MovementInterrupts, Error<E>> {
        let src = self.read_register(int.src()).await?;

        Ok(MovementInterrupts {
            z_high: (src & ZH != 0),
            z_low: (src & ZL != 0),
            y_high: (src & YH != 0),
            y_low: (src & YL != 0),
            x_high: (src & XH != 0),
            x_low: (src & XL != 0),
        })
    }

    /// Enables the click interrupt
    async fn enable_click_interrupt(&mut self, config: &ClickIntConfig) -> Result<(), Error<E>> {
        if config.threshold >= 0x80 || config.time_limit >= 0x80 {
            return Err(Error::InvalidParameter);
        }

        let mut cfg = 0;
        cfg |= (config.enable.z_double as u8) << 5;
        cfg |= (config.enable.z_single as u8) << 4;
        cfg |= (config.enable.y_double as u8) << 3;
        cfg |= (config.enable.y_single as u8) << 2;
        cfg |= (config.enable.x_double as u8) << 1;
        cfg |= config.enable.x_single as u8;

        let mut ths = config.threshold;
        ths |= (config.latch as u8) << 7;

        self.write_register(Register::ClickCfg, cfg).await?;
        self.write_register(Register::ClickThs, ths).await?;
        self.write_register(Register::TimeLimit, config.time_limit)
            .await?;
        self.write_register(Register::TimeLatency, config.time_latency)
            .await?;
        self.write_register(Register::TimeWindow, config.time_window)
            .await?;

        Ok(())
    }

    /// Reads the source of the click interrupt
    pub async fn read_click_interrupt_source(&mut self) -> Result<ClickIntSrc, Error<E>> {
        let src = self.read_register(Register::ClickSrc).await?;

        Ok(ClickIntSrc {
            double_click_enabled: (src & DCLICK != 0),
            single_click_enabled: (src & SCLICK != 0),
            positive: (src & SIGN != 0),
            z: (src & Z != 0),
            y: (src & Y != 0),
            x: (src & X != 0),
        })
    }

    async fn enable_activity_interrupt(
        &mut self,
        config: &ActivityIntConfig,
    ) -> Result<(), Error<E>> {
        if config.threshold >= 0x80 {
            return Err(Error::InvalidParameter);
        }

        self.write_register(Register::ActThs, config.threshold)
            .await?;
        self.write_register(Register::ActDur, config.duration).await
    }

    /// Enables the internal temperature sensor
    ///
    /// Note that enabling the temperature sensor disables continuous updates
    /// of the output registers
    pub async fn enable_temperature_sensor(&mut self, enable: bool) -> Result<(), Error<E>> {
        if enable {
            self.write_register(Register::TempCfgReg, TEMP_EN_MASK)
                .await?;
            self.set_register_bits(Register::CtrlReg4, BDU).await?;
        } else {
            self.write_register(Register::TempCfgReg, 0x00).await?;
        }

        Ok(())
    }

    /// Reads the output of the temperature sensor
    pub async fn read_temperature(&mut self) -> Result<u16, Error<E>> {
        // Set MSB to enable auto-incrementing the read address
        let reg_addr = Register::OutTempL as u8 | 0x80;
        self.i2c
            .write(self.addr, &[reg_addr])
            .await
            .map_err(Error::I2c)?;

        let mut buffer = [0; 2];
        self.i2c
            .read(self.addr, &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(u16::from_le_bytes([buffer[0], buffer[1]]))
    }

    /// Writes a value to a given register
    async fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        if register.is_read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.addr, &[register.addr(), value])
            .await
            .map_err(Error::I2c)?;
        Ok(())
    }

    /// Reads a value from a given register
    async fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[register.addr()], &mut buffer)
            .await
            .map_err(Error::I2c)?;
        Ok(buffer[0])
    }

    /// Modifies the value of a given register
    async fn modify_register<F>(&mut self, register: Register, f: F) -> Result<(), Error<E>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register).await?;
        self.write_register(register, f(value)).await
    }

    /// Sets some bits of a given register
    async fn set_register_bits(&mut self, register: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(register, |v| v | bits).await
    }

    /// Clears some bits of a given register
    async fn clear_register_bits(&mut self, register: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(register, |v| v & !bits).await
    }
}

/// SA0 pad connection on the board
#[derive(Copy, Clone, Debug)]
pub enum Sa0Pad {
    /// SA0 pad is connected to GND
    Low = 0,
    /// SA0 pad is connected to VDD
    High = 1,
}

/// Operating mode
#[derive(Copy, Clone, Debug)]
pub enum Mode {
    /// Low power mode (8-bit data output)
    LowPower,
    /// Normal mode (10-bit data output)
    Normal,
    /// High resolution mode (12-bit data output)
    HighResolution,
}

/// Full scale selection
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum FullScale {
    /// ±2 g
    Fs2g = 0x00,
    /// ±4 g
    Fs4g = 0x10,
    /// ±8 g
    Fs8g = 0x20,
    /// ±16 g
    Fs16g = 0x30,
}

/// Output data rate
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum OutputDataRate {
    /// Power-down mode
    PowerDown = 0x00,
    /// 1 Hz
    Hz1 = 0x10,
    /// 10 Hz
    Hz10 = 0x20,
    /// 25 Hz
    Hz25 = 0x30,
    /// 50 Hz
    Hz50 = 0x40,
    /// 100 Hz
    Hz100 = 0x50,
    /// 200 Hz
    Hz200 = 0x60,
    /// 400 Hz
    Hz400 = 0x70,
    /// 1.620 kHz in low-power mode
    /// Normal and high-resolution modes are not defined
    /// in the datasheet for this configuration
    HighSpeed = 0x80,
    /// 1.344 kHz in normal/high-resolution modes
    /// 5.376 kHz in low-power mode
    HighestSpeed = 0x90,
}

/// Status data
#[derive(Copy, Clone, Debug)]
pub struct Status {
    /// X-, Y- and Z-axis overrun
    pub xyz_overrun: bool,
    /// Z-axis data overrun
    pub z_overrun: bool,
    /// Y-axis data overrun
    pub y_overrun: bool,
    /// X-axis data overrun
    pub x_overrun: bool,
    /// X-, Y- and Z-axis new data available
    pub xyz_data_available: bool,
    /// Z-axis new data available
    pub z_data_available: bool,
    /// Y-axis new data available
    pub y_data_available: bool,
    /// Z-axis new data available
    pub x_data_available: bool,
    /// Temperature data overrun
    pub temp_overrun: bool,
    /// Temperature data available
    pub temp_data_available: bool,
}

/// Interrupt pin
#[derive(Copy, Clone, Debug)]
pub enum IntPin {
    /// Pin INT1
    Int1,
    /// Pin INT2
    Int2,
}

/// FIFO configuration
#[derive(Copy, Clone, Debug)]
pub enum FifoConfig {
    /// Bypass mode
    ///
    /// FIFO remains non-operational and for this reason it
    /// remains empty
    Bypass,
    /// FIFO mode
    ///
    /// Buffer continues filling data from the accelerometer channels
    /// until it is full (a set of 32 samples stored). When the FIFO
    /// is full, it stops collecting data from the input channels and
    /// the FIFO contents remain unchanged
    Fifo,
    /// Stream mode
    ///
    /// The FIFO continues filling data from the X, Y, and Z accelerometer channels
    /// until the buffer is full (a set of 32 samples stored) at which point
    /// the FIFO buffer index restarts from the beginning and older data is replaced
    /// by the current data. The oldest values continue to be overwritten until a
    /// read operation frees the FIFO slots
    Stream {
        /// Watermark level for the watermark interrupt
        /// When the interrupt is triggered there will be N+1 values to read
        watermark: u8,
    },
    /// Stream-to-FIFO mode
    ///
    /// Data from the X, Y and Z accelerometer channels are collected in a combination
    /// of Stream mode and FIFO mode. The FIFO buffer starts operating in Stream
    /// mode and switches to FIFO mode when the selected interrupt occurs
    StreamToFifo {
        /// Interrupt pin selected to trigger the switch to FIFO mode
        pin: IntPin,
    },
}

/// FIFO status data
#[derive(Copy, Clone, Debug)]
pub struct FifoStatus {
    /// FIFO content exceeds watermark level
    pub watermark_exceeded: bool,
    /// FIFO is full
    pub full: bool,
    /// FIFO is empty
    pub empty: bool,
    /// Current number of unread samples in the FIFO buffer
    pub sample_count: u8,
}

/// Interrupt mode for movement interrupts
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
pub enum MovementIntMode {
    /// OR combination of interrupt events
    OrCombination = 0x00,
    /// 6-direction movement recognition
    ///
    /// An interrupt is generated when the orientation moves from an unknown
    /// zone to a known zone. The interrupt signal remains for a duration ODR
    MovementRecognition = 0x40,
    /// AND combination of interrupt events
    AndCombination = 0x80,
    /// 6-direction position recognition
    ///
    /// An interrupt is generated when the orientation is inside a known zone
    /// The interrupt signal remains while the orientation is inside the zone
    PositionRecognition = 0xC0,
}

/// Movement interrupts
#[derive(Copy, Clone, Debug, Default)]
pub struct MovementInterrupts {
    /// X-axis acceleration higher than threshold value
    pub x_high: bool,
    /// X-axis acceleration lower than threshold value
    pub x_low: bool,
    /// Y-axis acceleration higher than threshold value
    pub y_high: bool,
    /// Y-axis acceleration lower than threshold value
    pub y_low: bool,
    /// Z-axis acceleration higher than threshold value
    pub z_high: bool,
    /// Z-axis acceleration lower than threshold value
    pub z_low: bool,
}

/// Movement interrupt configuration
#[derive(Copy, Clone, Debug)]
pub struct MovementIntConfig {
    /// Movement interrupt mode
    pub mode: MovementIntMode,
    /// Interrupts to enable
    pub enable: MovementInterrupts,
    /// Interrupt threshold
    pub threshold: u8,
    /// Minimum duration of the interrupt event for it to be recognized
    pub duration: u8,
    /// Latch interrupt request
    /// Cleared by reading movement interrupt source
    pub latch: bool,
    /// Enable 4D-detection: Disables the Z-axis
    pub only_4d: bool,
}

/// Click interrupts
#[derive(Copy, Clone, Debug, Default)]
pub struct ClickInterrupts {
    /// Single click on X-axis
    pub x_single: bool,
    /// Double click on X-axis
    pub x_double: bool,
    /// Single click on Y-axis
    pub y_single: bool,
    /// Double click on Y-axis
    pub y_double: bool,
    /// Single click on Z-axis
    pub z_single: bool,
    /// Double click on Z-axis
    pub z_double: bool,
}

/// Click interrupt configuration
#[derive(Copy, Clone, Debug)]
pub struct ClickIntConfig {
    /// Interrupts to enable
    pub enable: ClickInterrupts,
    /// Interrupt threshold
    pub threshold: u8,
    /// Click time limit
    pub time_limit: u8,
    /// Click time latency
    pub time_latency: u8,
    /// Click time window
    pub time_window: u8,
    /// Latch interrupt request
    /// Cleared by reading click interrupt source
    pub latch: bool,
}

/// Click interrupt source
#[derive(Copy, Clone, Debug)]
pub struct ClickIntSrc {
    /// X-axis click detected
    pub x: bool,
    /// Y-axis click detected
    pub y: bool,
    /// Z-axis click detected
    pub z: bool,
    /// Click sign
    pub positive: bool,
    /// Single click detection enabled
    pub single_click_enabled: bool,
    /// Double click detection enabled
    pub double_click_enabled: bool,
}

/// Activity interrupt configuration
#[derive(Copy, Clone, Debug)]
pub struct ActivityIntConfig {
    /// Sleep-to-wake, return-to-sleep activation threshold in low-power mode
    /// 1 LSb = 16 mg @ FS = 2 g
    /// 1 LSb = 32 mg @ FS = 4 g
    /// 1 LSb = 62 mg @ FS = 8 g
    /// 1 LSb = 186 mg @ FS = 16 g
    pub threshold: u8,
    /// Sleep-to-wake, return-to-sleep duration
    pub duration: u8,
}

/// INT1 configuration
#[derive(Copy, Clone, Debug)]
pub enum Int1Config {
    /// Interrupt pin unused
    Unused,
    /// Click interrupt
    Click(ClickIntConfig),
    /// Movement interrupt 1
    MovementInt1(MovementIntConfig),
    /// Movement interrupt 2
    MovementInt2(MovementIntConfig),
    /// X-, Y- and Z-axis data available interrupt
    DataAvailableXYZ,
    /// FIFO watermark interrupt
    FifoWatermark,
    /// FIFO overrun interrupt
    FifoOverrun,
}

/// INT2 configuration
#[derive(Copy, Clone, Debug)]
pub enum Int2Config {
    /// Interrupt pin unused
    Unused,
    /// Click interrupt
    Click(ClickIntConfig),
    /// Movement interrupt 1
    MovementInt1(MovementIntConfig),
    /// Movement interrupt 2
    MovementInt2(MovementIntConfig),
    /// Boot
    Boot,
    /// Activity interrupt
    Activity(ActivityIntConfig),
}

/// Interrupt polarity
#[derive(Copy, Clone, Debug)]
pub enum IntPolarity {
    /// Active high
    ActiveHigh,
    /// Active low
    ActiveLow,
}

/// Acceleration data
#[derive(Copy, Clone, Debug, Default)]
pub struct AccelerationData {
    /// X-axis
    pub x: i16,
    /// Y-axis
    pub y: i16,
    /// Z-axis
    pub z: i16,
}
