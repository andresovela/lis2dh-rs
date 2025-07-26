//! Register utilities.

/// LIS2DH registers
#[derive(Copy, Clone, Debug)]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Register {
    StatusRegAux = 0x07,
    OutTempL = 0x0C,
    OutTempH = 0x0D,
    WhoAmI = 0x0F,
    CtrlReg0 = 0x1E,
    TempCfgReg = 0x1F,
    CtrlReg1 = 0x20,
    CtrlReg2 = 0x21,
    CtrlReg3 = 0x22,
    CtrlReg4 = 0x23,
    CtrlReg5 = 0x24,
    CtrlReg6 = 0x25,
    Reference = 0x26,
    StatusReg = 0x27,
    OutXL = 0x28,
    OutXH = 0x29,
    OutYL = 0x2A,
    OutYH = 0x2B,
    OutZL = 0x2C,
    OutZH = 0x2D,
    FifoCtrlReg = 0x2E,
    FifoSrcReg = 0x2F,
    Int1Cfg = 0x30,
    Int1Src = 0x31,
    Int1Ths = 0x32,
    Int1Duration = 0x33,
    Int2Cfg = 0x34,
    Int2Src = 0x35,
    Int2Ths = 0x36,
    Int2Duration = 0x37,
    ClickCfg = 0x38,
    ClickSrc = 0x39,
    ClickThs = 0x3A,
    TimeLimit = 0x3B,
    TimeLatency = 0x3C,
    TimeWindow = 0x3D,
    ActThs = 0x3E,
    ActDur = 0x3F,
}

impl Register {
    /// Get the address of the register
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Checks if the register is read-only
    pub fn is_read_only(self) -> bool {
        matches!(
            self,
            Register::StatusRegAux
                | Register::OutTempL
                | Register::OutTempH
                | Register::WhoAmI
                | Register::OutXL
                | Register::OutXH
                | Register::OutYL
                | Register::OutYH
                | Register::OutZL
                | Register::OutZH
                | Register::FifoSrcReg
                | Register::Int1Src
                | Register::Int2Src
                | Register::ClickSrc
        )
    }
}

// STATUS_REG_AUX
pub(crate) const TOR: u8 = 0x40;
pub(crate) const TDA: u8 = 0x04;

// TEMP_CFG_REG
pub(crate) const TEMP_EN_MASK: u8 = 0xC0;

// CTRL_REG_1
pub(crate) const ODR_MASK: u8 = 0xF0;
pub(crate) const LPEN: u8 = 0x08;

// CTRL_REG_4
pub(crate) const BDU: u8 = 0x80;
pub(crate) const FS_MASK: u8 = 0x30;
pub(crate) const HR: u8 = 0x08;

// CTRL_REG_3
pub(crate) const I1_CLICK: u8 = 0x80;
pub(crate) const I1_IA1: u8 = 0x40;
pub(crate) const I1_IA2: u8 = 0x20;
pub(crate) const I1_ZYXDA: u8 = 0x10;
pub(crate) const I1_WTM: u8 = 0x04;
pub(crate) const I1_OVERRUN: u8 = 0x02;

// CTRL_REG_5
pub(crate) const BOOT: u8 = 0x80;
pub(crate) const FIFO_EN: u8 = 0x40;
pub(crate) const LIR_INT1: u8 = 0x08;
pub(crate) const D4D_INT1: u8 = 0x04;
pub(crate) const LIR_INT2: u8 = 0x02;
pub(crate) const D4D_INT2: u8 = 0x01;

// CTRL_REG_6
pub(crate) const I2_MASK: u8 = 0xF8;
pub(crate) const I2_CLICK: u8 = 0x80;
pub(crate) const I2_IA1: u8 = 0x40;
pub(crate) const I2_IA2: u8 = 0x20;
pub(crate) const I2_BOOT: u8 = 0x10;
pub(crate) const I2_ACT: u8 = 0x08;
pub(crate) const INT_POLARITY: u8 = 0x02;

// STATUS_REG
pub(crate) const ZYXOR: u8 = 0x80;
pub(crate) const ZOR: u8 = 0x40;
pub(crate) const YOR: u8 = 0x20;
pub(crate) const XOR: u8 = 0x10;
pub(crate) const ZYXDA: u8 = 0x08;
pub(crate) const ZDA: u8 = 0x04;
pub(crate) const YDA: u8 = 0x02;
pub(crate) const XDA: u8 = 0x01;

// FIFO_SRC_REG
pub(crate) const WTM: u8 = 0x80;
pub(crate) const OVRN_FIFO: u8 = 0x40;
pub(crate) const EMPTY: u8 = 0x20;
pub(crate) const FSS_MASK: u8 = 0x1F;

// INTX_SRC
pub(crate) const ZH: u8 = 0x20;
pub(crate) const ZL: u8 = 0x10;
pub(crate) const YH: u8 = 0x08;
pub(crate) const YL: u8 = 0x04;
pub(crate) const XH: u8 = 0x02;
pub(crate) const XL: u8 = 0x01;

// CLICK_SRC
pub(crate) const DCLICK: u8 = 0x20;
pub(crate) const SCLICK: u8 = 0x10;
pub(crate) const SIGN: u8 = 0x08;
pub(crate) const Z: u8 = 0x04;
pub(crate) const Y: u8 = 0x02;
pub(crate) const X: u8 = 0x01;

/// INT_* registers
pub trait IntRegisters {
    /// LIR bit position in register CTRL_REG_5
    const LIR_BIT: u8;
    /// D4D bit position in register CTRL_REG_5
    const D4D_BIT: u8;
    /// Get the CFG register for the INT block
    fn cfg(&self) -> Register;
    /// Get the SRC register for the INT block
    fn src(&self) -> Register;
    /// Get the THS register for the INT block
    fn ths(&self) -> Register;
    /// Get the DUR register for the INT block
    fn dur(&self) -> Register;
}

/// Representation of INT1 registers
pub struct Int1;

/// Representation of INT2 registers
pub struct Int2;

macro_rules! impl_int_registers {
    ($IntType:ident: $Cfg:ident, $Src:ident, $Ths:ident, $Dur:ident, $Lir:expr, $D4d:expr) => {
        impl IntRegisters for $IntType {
            const LIR_BIT: u8 = $Lir;
            const D4D_BIT: u8 = $D4d;
            fn cfg(&self) -> Register {
                Register::$Cfg
            }
            fn src(&self) -> Register {
                Register::$Src
            }
            fn ths(&self) -> Register {
                Register::$Ths
            }
            fn dur(&self) -> Register {
                Register::$Dur
            }
        }
    };
}

impl_int_registers!(Int1: Int1Cfg, Int1Src, Int1Ths, Int1Duration, LIR_INT1, D4D_INT1);
impl_int_registers!(Int2: Int2Cfg, Int2Src, Int2Ths, Int2Duration, LIR_INT2, D4D_INT2);
