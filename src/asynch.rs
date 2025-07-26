use super::*;

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
            xyz_data_available: (sr & ZYXDA != 0),
            z_data_available: (sr & ZDA != 0),
            y_data_available: (sr & YDA != 0),
            x_data_available: (sr & XDA != 0),
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
        for sample in data {
            self.read_from_fifo(sample).await?;
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
