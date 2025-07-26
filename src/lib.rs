#![doc = include_str!("../README.md")]
#![no_std]
#![deny(missing_docs)]

#[cfg(feature = "async")]
mod asynch;
mod register;
#[cfg(not(feature = "async"))]
mod sync;

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
