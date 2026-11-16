#![no_std]
#![allow(unknown_lints)]
#![allow(clippy::chunks_exact_to_as_chunks)]

//! Native ST VL53L5CX 3D multi-zone Time-of-Flight driver.
//!
//! A `#![no_std]` direct, byte-exact implementation of STMicroelectronics' Ultra Lite Driver (ULD)
//! for the VL53L5CX 8x8 / 4x4 multizone Time-of-Flight sensor.
//!
//! Provides:
//! - Blocking driver `Vl53l5cx<BUS, DELAY>` based on `embedded-hal` 1.0 (`I2c` and `DelayNs`).
//! - Optional async driver `asynch::Vl53l5cx<BUS>` based on `embedded-hal-async` 1.0 (`feature = "async"`).
//! - Feature-gated ST firmware binary blobs (`feature = "firmware-blob"`).
//! - Byte-exact pure ULD codec (`codec` module) with 4x4 matrix extraction, status decoding, and sigma validation.

pub mod codec;

pub use codec::{Frame4x4, Frame8x8, FrameError};

/// Factory I2C address (7-bit).
pub const DEFAULT_ADDR: u8 = 0x29;

pub const DEVICE_ID: u8 = 0xF0;
pub const REVISION_ID: u8 = 0x02;

#[cfg(feature = "firmware-blob")]
const FIRMWARE: &[u8] = include_bytes!("vl53l5cx_fw.bin");
#[cfg(feature = "firmware-blob")]
const DEFAULT_CONFIGURATION: &[u8] = include_bytes!("vl53l5cx_cfg.bin");
#[cfg(feature = "firmware-blob")]
const DEFAULT_XTALK: &[u8] = include_bytes!("vl53l5cx_xtalk.bin");
#[cfg(feature = "firmware-blob")]
const GET_NVM_CMD: &[u8] = include_bytes!("vl53l5cx_nvm_cmd.bin");

// Compile-time guarantees on the ST blobs (fails the build, never at runtime).
#[cfg(feature = "firmware-blob")]
const _: () = assert!(FIRMWARE.len() == 0x15000);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(DEFAULT_CONFIGURATION.len() == 972);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(DEFAULT_XTALK.len() == codec::XTALK_BUFFER_SIZE);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(GET_NVM_CMD.len() == 40);

#[cfg(feature = "firmware-blob")]
const FW_PAGE_09: &[u8] = FIRMWARE.split_at(0x8000).0;
#[cfg(feature = "firmware-blob")]
const FW_REST: &[u8] = FIRMWARE.split_at(0x8000).1;
#[cfg(feature = "firmware-blob")]
const FW_PAGE_0A: &[u8] = FW_REST.split_at(0x8000).0;
#[cfg(feature = "firmware-blob")]
const FW_PAGE_0B: &[u8] = FW_REST.split_at(0x8000).1;
#[cfg(feature = "firmware-blob")]
const _: () = assert!(FW_PAGE_0B.len() == 0x5000);

pub const UI_CMD_STATUS: u16 = 0x2C00;
pub const UI_CMD_START: u16 = 0x2C04;
pub const UI_CMD_END: u16 = 0x2FFF;

/// Address for writing DCI commands to VL53L5CX.
///
/// Per ST ULD vl53l5cx_api.c, this is defined as `UI_CMD_END - 11` = 0x2FF4.
pub const DCI_CMD_ADDR: u16 = 0x2FF4;

pub const DCI_ZONE_CONFIG: u16 = 0x5450;
pub const DCI_RANGING_MODE: u16 = 0xAD30;
pub const DCI_DSS_CONFIG: u16 = 0xAD38;
pub const DCI_FREQ_HZ: u16 = 0x5458;
pub const DCI_SINGLE_RANGE: u16 = 0xD964;
pub const DCI_OUTPUT_CONFIG: u16 = 0xD968;
pub const DCI_OUTPUT_ENABLES: u16 = 0xD970;
pub const DCI_OUTPUT_LIST: u16 = 0xD980;
pub const DCI_PIPE_CONTROL: u16 = 0xDB80;
pub const DCI_GLARE_FILTER: u16 = 0xE108;
pub const DCI_UI_RANGE_DATA: u16 = 0x5440;
pub const DCI_TARGET_ORDER: u16 = 0xAE64;
pub const DCI_SHARPENER: u16 = 0xAED8;

/// Target ordering within each multi-target detection zone (ST ULD `vl53l5cx_set_target_order`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TargetOrder {
    /// Return the closest target first (ideal for fingertips and obstacle avoidance).
    Closest = 1,
    /// Return the target with strongest signal amplitude first (default factory mode).
    Strongest = 2,
}

/// Scratch buffer size (ULD `VL53L5CX_TEMPORARY_BUFFER_SIZE` for <= 4x4 data).
pub const TEMP_SIZE: usize = 1024;

/// ULD `_vl53l5cx_poll_for_answer`: 200 reads x 10 ms = 2 s.
pub const POLL_ATTEMPTS: u16 = 200;
pub const POLL_INTERVAL_MS: u32 = 10;
/// ULD `_vl53l5cx_poll_for_mcu_boot`: 500 reads x 2 ms = 1 s.
pub const MCU_BOOT_ATTEMPTS: u16 = 500;

/// Highest ranging frequency supported in 4x4 mode.
pub const MAX_FREQ_HZ_4X4: u8 = 60;

/// Driver error. `E` is the I2C bus error type.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<E> {
    Bus(E),
    /// Wrong Device/Revision ID at the configured address.
    NotAlive {
        device_id: u8,
        revision_id: u8,
    },
    /// Page register `0x7FFF` did not latch the expected page before a bank write.
    UnexpectedPage {
        expected: u8,
        actual: u8,
    },
    /// A bounded poll expired without the expected value.
    Timeout {
        register: u16,
        last: u8,
    },
    /// The sensor MCU reported an error status (>= 0x7F).
    McuError(u8),
    /// Non-zero GO2 status (MCU boot failure or runtime error).
    Go2Status(u8),
    /// The firmware reports a frame size different from the configured one.
    OutputSizeMismatch {
        expected: u32,
        reported: u16,
    },
    /// Only 4x4 resolution is supported by this driver.
    UnsupportedResolution(u16),
    /// Ranging data requested before `start_ranging`.
    NotRanging,
    InvalidParam,
    Frame(FrameError),
}

/// Returns `true` if `(device_id, revision_id)` identifies a VL53L5CX.
pub const fn is_vl53l5cx_id(device_id: u8, revision_id: u8) -> bool {
    device_id == DEVICE_ID && revision_id == REVISION_ID
}

// ============================================================================
// Standard Blocking Driver (embedded-hal 1.0)
// ============================================================================

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, Operation};

/// Reads Device ID / Revision ID without modifying operational state
/// (page register 0x7FFF is restored to page 2 afterwards).
pub fn probe<B: I2c, D: DelayNs>(
    bus: &mut B,
    delay: &mut D,
    addr: u8,
) -> Result<(u8, u8), B::Error> {
    let mut page_err = None;
    for attempt in 0..3 {
        match bus.write(addr, &[0x7F, 0xFF, 0x00]) {
            Ok(()) => {
                page_err = None;
                break;
            }
            Err(e) => {
                page_err = Some(e);
                delay.delay_ms(15 * (attempt + 1));
            }
        }
    }
    if let Some(err) = page_err {
        return Err(err);
    }
    delay.delay_ms(5);

    let mut id = [0u8; 2];
    let res = bus.write_read(addr, &[0x00, 0x00], &mut id);

    let restore = bus.write(addr, &[0x7F, 0xFF, 0x02]);
    res?;
    restore?;
    Ok((id[0], id[1]))
}

fn write_multi_blocking<B: I2c, D: DelayNs>(
    bus: &mut B,
    delay: &mut D,
    addr: u8,
    reg: u16,
    data: &[u8],
) -> Result<(), B::Error> {
    let reg_bytes = reg.to_be_bytes();
    let mut last_err = None;
    for attempt in 0..5 {
        let mut ops = [Operation::Write(&reg_bytes), Operation::Write(data)];
        match bus.transaction(addr, &mut ops) {
            Ok(()) => return Ok(()),
            Err(e) => {
                last_err = Some(e);
                delay.delay_ms(10 * (attempt + 1));
            }
        }
    }
    match last_err {
        Some(err) => Err(err),
        None => Ok(()),
    }
}

fn read_multi_blocking<B: I2c, D: DelayNs>(
    bus: &mut B,
    delay: &mut D,
    addr: u8,
    reg: u16,
    buf: &mut [u8],
) -> Result<(), B::Error> {
    let reg_bytes = reg.to_be_bytes();
    let mut last_err = None;
    for attempt in 0..5 {
        match bus.write_read(addr, &reg_bytes, buf) {
            Ok(()) => return Ok(()),
            Err(e) => {
                last_err = Some(e);
                delay.delay_ms(15 * (attempt + 1));
            }
        }
    }
    match last_err {
        Some(err) => Err(err),
        None => Ok(()),
    }
}

#[cfg(feature = "firmware-blob")]
fn write_fw_bank_blocking<B: I2c, D: DelayNs>(
    bus: &mut B,
    delay: &mut D,
    addr: u8,
    page: u8,
    data: &[u8],
) -> Result<(), Error<B::Error>> {
    let mut page_err = None;
    for attempt in 0..20 {
        match bus.write(addr, &[0x7F, 0xFF, page]) {
            Ok(()) => {
                page_err = None;
                break;
            }
            Err(e) => {
                page_err = Some(e);
                delay.delay_ms(15 * (attempt + 1));
            }
        }
    }
    if let Some(err) = page_err {
        return Err(Error::Bus(err));
    }

    delay.delay_ms(10);

    let reg_bytes = [0x00, 0x00];
    let mut ops = [Operation::Write(&reg_bytes), Operation::Write(data)];
    let mut bank_err = None;
    for attempt in 0..5 {
        match bus.transaction(addr, &mut ops) {
            Ok(()) => {
                bank_err = None;
                break;
            }
            Err(e) => {
                bank_err = Some(e);
                delay.delay_ms(20 * (attempt + 1));
            }
        }
    }
    if let Some(err) = bank_err {
        return Err(Error::Bus(err));
    }

    delay.delay_ms(50);
    Ok(())
}

/// Standard blocking VL53L5CX driver instance.
pub struct Vl53l5cx<BUS, DELAY> {
    bus: BUS,
    delay: DELAY,
    addr: u8,
    temp: [u8; TEMP_SIZE],
    #[allow(dead_code)]
    offset_data: [u8; codec::OFFSET_BUFFER_SIZE],
    streamcount: u8,
    data_read_size: usize,
}

impl<BUS: I2c, DELAY: DelayNs> Vl53l5cx<BUS, DELAY> {
    /// Creates a new driver instance.
    pub const fn new(bus: BUS, delay: DELAY, addr: u8) -> Self {
        Self {
            bus,
            delay,
            addr,
            temp: [0u8; TEMP_SIZE],
            offset_data: [0u8; codec::OFFSET_BUFFER_SIZE],
            streamcount: 255,
            data_read_size: 0,
        }
    }

    /// Returns the configured 7-bit I2C address.
    pub const fn address(&self) -> u8 {
        self.addr
    }

    /// Releases the inner bus and delay peripherals.
    pub fn into_inner(self) -> (BUS, DELAY) {
        (self.bus, self.delay)
    }

    fn wr(&mut self, reg: u16, value: u8) -> Result<(), Error<BUS::Error>> {
        let [hi, lo] = reg.to_be_bytes();
        let mut last_err = None;
        for attempt in 0..5 {
            match self.bus.write(self.addr, &[hi, lo, value]) {
                Ok(()) => {
                    if reg == 0x7FFF {
                        self.delay.delay_ms(2);
                    }
                    return Ok(());
                }
                Err(e) => {
                    last_err = Some(e);
                    self.delay.delay_ms(5 * (attempt + 1));
                }
            }
        }
        if let Some(err) = last_err {
            Err(Error::Bus(err))
        } else {
            Err(Error::InvalidParam)
        }
    }

    fn rd(&mut self, reg: u16) -> Result<u8, Error<BUS::Error>> {
        let mut b = [0u8; 1];
        read_multi_blocking(&mut self.bus, &mut self.delay, self.addr, reg, &mut b)
            .map_err(Error::Bus)?;
        Ok(b[0])
    }

    fn write_slice(&mut self, reg: u16, data: &[u8]) -> Result<(), Error<BUS::Error>> {
        write_multi_blocking(&mut self.bus, &mut self.delay, self.addr, reg, data)
            .map_err(Error::Bus)
    }

    fn write_temp(&mut self, reg: u16, len: usize) -> Result<(), Error<BUS::Error>> {
        let buf = self.temp.get(..len).ok_or(Error::InvalidParam)?;
        write_multi_blocking(&mut self.bus, &mut self.delay, self.addr, reg, buf)
            .map_err(Error::Bus)
    }

    fn read_temp(&mut self, reg: u16, len: usize) -> Result<(), Error<BUS::Error>> {
        let buf = self.temp.get_mut(..len).ok_or(Error::InvalidParam)?;
        read_multi_blocking(&mut self.bus, &mut self.delay, self.addr, reg, buf).map_err(Error::Bus)
    }

    fn poll_for_answer(
        &mut self,
        size: usize,
        pos: usize,
        reg: u16,
        mask: u8,
        expected: u8,
    ) -> Result<(), Error<BUS::Error>> {
        let mut buf = [0u8; 4];
        let buf = buf.get_mut(..size).ok_or(Error::InvalidParam)?;
        let mut last_bus_err = None;
        for _attempt in 0..POLL_ATTEMPTS {
            self.delay.delay_ms(POLL_INTERVAL_MS);
            match read_multi_blocking(&mut self.bus, &mut self.delay, self.addr, reg, buf) {
                Ok(()) => {
                    last_bus_err = None;
                    match codec::poll_outcome(buf, pos, mask, expected) {
                        codec::PollOutcome::Done => return Ok(()),
                        codec::PollOutcome::McuError(s) => return Err(Error::McuError(s)),
                        codec::PollOutcome::Pending => {}
                    }
                }
                Err(e) => {
                    last_bus_err = Some(e);
                }
            }
        }
        if let Some(err) = last_bus_err {
            Err(Error::Bus(err))
        } else {
            let last = match buf.get(pos) {
                Some(&val) => val,
                None => 0,
            };
            Err(Error::Timeout {
                register: reg,
                last,
            })
        }
    }

    #[cfg(feature = "firmware-blob")]
    fn poll_for_mcu_boot(&mut self) -> Result<(), Error<BUS::Error>> {
        let mut status0 = 0xFF;
        let mut last_err = None;
        for _attempt in 0..MCU_BOOT_ATTEMPTS {
            match self.rd(0x06) {
                Ok(s) => {
                    last_err = None;
                    status0 = s;
                    if status0 & 0x80 != 0 {
                        let status1 = match self.rd(0x07) {
                            Ok(v) => v,
                            Err(_) => status0,
                        };
                        return Err(Error::Go2Status(status1));
                    }
                    if status0 & 0x01 != 0 || status0 == 0x00 {
                        return Ok(());
                    }
                }
                Err(e) => {
                    last_err = Some(e);
                }
            }
            self.delay.delay_ms(2);
        }

        if let Some(err) = last_err {
            Err(err)
        } else {
            Err(Error::Timeout {
                register: 0x06,
                last: status0,
            })
        }
    }

    fn dci_read(&mut self, index: u16, out: &mut [u8]) -> Result<(), Error<BUS::Error>> {
        let n = out.len();
        if !n.is_multiple_of(4) || n + 12 > TEMP_SIZE {
            return Err(Error::InvalidParam);
        }
        let cmd = codec::dci_read_cmd(index, n as u16);
        self.write_slice(DCI_CMD_ADDR, &cmd)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;
        self.read_temp(UI_CMD_START, n + 12)?;
        codec::swap_words(&mut self.temp[..n + 12]);
        out.copy_from_slice(&self.temp[4..4 + n]);
        Ok(())
    }

    fn dci_write(&mut self, index: u16, data: &[u8]) -> Result<(), Error<BUS::Error>> {
        let (address, len) =
            codec::dci_write_frame(index, data, &mut self.temp).ok_or(Error::InvalidParam)?;
        self.write_temp(address, len)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
    }

    fn dci_replace(
        &mut self,
        index: u16,
        size: usize,
        new_data: &[u8],
        pos: usize,
    ) -> Result<(), Error<BUS::Error>> {
        let mut local = [0u8; 64];
        if size > local.len() || pos + new_data.len() > size {
            return Err(Error::InvalidParam);
        }
        self.dci_read(index, &mut local[..size])?;
        local[pos..pos + new_data.len()].copy_from_slice(new_data);
        self.dci_write(index, &local[..size])
    }

    /// Verifies Device ID (0xF0) and Revision ID (0x02).
    pub fn is_alive(&mut self) -> Result<(), Error<BUS::Error>> {
        let (device_id, revision_id) =
            probe(&mut self.bus, &mut self.delay, self.addr).map_err(Error::Bus)?;
        if is_vl53l5cx_id(device_id, revision_id) {
            Ok(())
        } else {
            Err(Error::NotAlive {
                device_id,
                revision_id,
            })
        }
    }

    /// Changes the 7-bit I2C slave address.
    pub fn set_i2c_address(&mut self, new_addr: u8) -> Result<(), Error<BUS::Error>> {
        if new_addr == 0 || new_addr > 0x7F {
            return Err(Error::InvalidParam);
        }
        self.wr(0x7FFF, 0x00)?;
        self.wr(0x0004, new_addr)?;
        self.addr = new_addr;
        self.wr(0x7FFF, 0x02)
    }

    /// Boots the sensor: SW reboot, firmware download (84 KiB), NVM offset
    /// calibration, Xtalk and default configuration (ULD `vl53l5cx_init`).
    #[cfg(feature = "firmware-blob")]
    pub fn init(&mut self) -> Result<(), Error<BUS::Error>> {
        self.data_read_size = 0;
        self.streamcount = 255;

        // SW reboot sequence
        let _ = self.wr(0x7FFF, 0x00);
        let _ = self.wr(0x0009, 0x04);
        let _ = self.wr(0x000F, 0x40);
        let _ = self.wr(0x000A, 0x03);
        let _ = self.rd(0x7FFF);
        let _ = self.wr(0x000C, 0x01);

        let _ = self.wr(0x0101, 0x00);
        let _ = self.wr(0x0102, 0x00);
        let _ = self.wr(0x010A, 0x01);
        let _ = self.wr(0x4002, 0x01);
        let _ = self.wr(0x4002, 0x00);
        let _ = self.wr(0x010A, 0x03);
        let _ = self.wr(0x0103, 0x01);
        let _ = self.wr(0x000C, 0x00);
        let _ = self.wr(0x000F, 0x43);
        self.delay.delay_ms(1);

        let _ = self.wr(0x000F, 0x40);
        let _ = self.wr(0x000A, 0x01);
        self.delay.delay_ms(100);

        self.wr(0x7FFF, 0x00)?;
        self.poll_for_answer(1, 0, 0x06, 0xFF, 0x01)?;

        self.wr(0x000E, 0x01)?;
        self.wr(0x7FFF, 0x02)?;

        self.wr(0x03, 0x0D)?;
        self.wr(0x7FFF, 0x01)?;
        self.poll_for_answer(1, 0, 0x21, 0x10, 0x10)?;
        self.wr(0x7FFF, 0x00)?;

        let _ = self.rd(0x7FFF)?;
        self.wr(0x0C, 0x01)?;

        // Power ON status
        self.wr(0x7FFF, 0x00)?;
        self.wr(0x101, 0x00)?;
        self.wr(0x102, 0x00)?;
        self.wr(0x010A, 0x01)?;
        self.wr(0x4002, 0x01)?;
        self.wr(0x4002, 0x00)?;
        self.wr(0x010A, 0x03)?;
        self.wr(0x103, 0x01)?;
        self.wr(0x400F, 0x00)?;
        self.wr(0x21A, 0x43)?;
        self.wr(0x21A, 0x03)?;
        self.wr(0x21A, 0x01)?;
        self.wr(0x21A, 0x00)?;
        self.wr(0x219, 0x00)?;
        self.wr(0x21B, 0x00)?;

        // Wake up MCU
        self.wr(0x7FFF, 0x00)?;
        let _ = self.rd(0x7FFF)?;
        self.wr(0x0C, 0x00)?;
        self.wr(0x7FFF, 0x01)?;
        self.delay.delay_ms(5);
        self.wr(0x20, 0x07)?;
        self.wr(0x20, 0x06)?;

        // Download firmware (3 pages)
        write_fw_bank_blocking(&mut self.bus, &mut self.delay, self.addr, 0x09, FW_PAGE_09)?;
        write_fw_bank_blocking(&mut self.bus, &mut self.delay, self.addr, 0x0A, FW_PAGE_0A)?;
        write_fw_bank_blocking(&mut self.bus, &mut self.delay, self.addr, 0x0B, FW_PAGE_0B)?;
        self.wr(0x7FFF, 0x01)?;
        self.delay.delay_ms(5);

        // Check FW correctly downloaded
        self.wr(0x7FFF, 0x02)?;
        self.delay.delay_ms(2);
        self.wr(0x03, 0x0D)?;
        self.wr(0x7FFF, 0x01)?;
        self.delay.delay_ms(5);
        self.poll_for_answer(1, 0, 0x21, 0x10, 0x10)?;

        self.wr(0x7FFF, 0x00)?;
        let _ = self.rd(0x7FFF)?;
        self.wr(0x0C, 0x01)?;

        // Reset MCU and wait boot
        self.wr(0x7FFF, 0x00)?;
        self.wr(0x114, 0x00)?;
        self.wr(0x115, 0x00)?;
        self.wr(0x116, 0x42)?;
        self.wr(0x117, 0x00)?;
        self.wr(0x0B, 0x00)?;
        let _ = self.rd(0x7FFF)?;
        self.wr(0x0C, 0x00)?;
        self.wr(0x0B, 0x01)?;
        self.delay.delay_ms(150);
        self.poll_for_mcu_boot()?;

        self.wr(0x7FFF, 0x02)?;
        self.delay.delay_ms(20);

        // Offset calibration from NVM
        self.write_slice(0x2FD8, GET_NVM_CMD)?;
        self.poll_for_answer(4, 0, UI_CMD_STATUS, 0xFF, 0x02)?;
        self.read_temp(UI_CMD_START, codec::NVM_DATA_SIZE)?;
        self.offset_data
            .copy_from_slice(&self.temp[..codec::OFFSET_BUFFER_SIZE]);
        codec::prepare_offset_4x4(&mut self.temp).map_err(|_| Error::InvalidParam)?;
        self.write_temp(0x2E18, codec::OFFSET_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        // Default Xtalk
        self.temp[..codec::XTALK_BUFFER_SIZE].copy_from_slice(DEFAULT_XTALK);
        codec::prepare_xtalk_4x4(&mut self.temp).map_err(|_| Error::InvalidParam)?;
        self.write_temp(0x2CF8, codec::XTALK_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        // Default configuration
        self.write_slice(0x2C34, DEFAULT_CONFIGURATION)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        // Pipe control, single range, glare filter
        self.dci_write(DCI_PIPE_CONTROL, &[1, 0x00, 0x01, 0x00])?;
        self.dci_write(DCI_SINGLE_RANGE, &1u32.to_le_bytes())?;
        self.dci_replace(DCI_GLARE_FILTER, 40, &[1], 0x26)?;
        self.dci_replace(DCI_GLARE_FILTER, 40, &[1], 0x25)?;

        self.set_resolution_4x4()?;
        Ok(())
    }

    /// Sets sensor resolution to 4x4.
    pub fn set_resolution_4x4(&mut self) -> Result<(), Error<BUS::Error>> {
        let mut dss = [0u8; 16];
        self.dci_read(DCI_DSS_CONFIG, &mut dss)?;
        dss[0x04] = 64;
        dss[0x06] = 64;
        dss[0x09] = 4;
        self.dci_write(DCI_DSS_CONFIG, &dss)?;

        let mut zone = [0u8; 8];
        self.dci_read(DCI_ZONE_CONFIG, &mut zone)?;
        zone[0x00] = 4;
        zone[0x01] = 4;
        zone[0x04] = 8;
        zone[0x05] = 8;
        self.dci_write(DCI_ZONE_CONFIG, &zone)?;

        Ok(())
    }

    /// Sets sensor resolution to 8x8 (64 zones).
    #[cfg(feature = "firmware-blob")]
    pub fn set_resolution_8x8(&mut self) -> Result<(), Error<BUS::Error>> {
        let mut dss = [0u8; 16];
        self.dci_read(DCI_DSS_CONFIG, &mut dss)?;
        dss[0x04] = 16;
        dss[0x06] = 16;
        dss[0x09] = 1;
        self.dci_write(DCI_DSS_CONFIG, &dss)?;

        let mut zone = [0u8; 8];
        self.dci_read(DCI_ZONE_CONFIG, &mut zone)?;
        zone[0x00] = 8;
        zone[0x01] = 8;
        zone[0x04] = 4;
        zone[0x05] = 4;
        self.dci_write(DCI_ZONE_CONFIG, &zone)?;

        // Send 8x8 offset calibration data per ST ULD lines 953 and 143-152
        self.temp[..codec::OFFSET_BUFFER_SIZE].copy_from_slice(&self.offset_data);
        codec::prepare_offset_8x8(&mut self.temp).map_err(|_| Error::InvalidParam)?;
        self.write_temp(0x2E18, codec::OFFSET_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        // Send 8x8 default Xtalk data per ST ULD lines 954 and 209-212
        self.temp[..codec::XTALK_BUFFER_SIZE].copy_from_slice(DEFAULT_XTALK);
        self.write_temp(0x2CF8, codec::XTALK_BUFFER_SIZE)?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        Ok(())
    }

    /// Sets continuous ranging mode.
    pub fn set_ranging_mode_continuous(&mut self) -> Result<(), Error<BUS::Error>> {
        let mut buf = [0u8; 8];
        self.dci_read(DCI_RANGING_MODE, &mut buf)?;
        buf[0x01] = 0x01;
        buf[0x03] = 0x03;
        self.dci_write(DCI_RANGING_MODE, &buf)?;
        self.dci_write(DCI_SINGLE_RANGE, &[0x00, 0x00, 0x00, 0x00])
    }

    /// Sets ranging frequency in Hz (1..=60).
    pub fn set_ranging_frequency_hz(&mut self, hz: u8) -> Result<(), Error<BUS::Error>> {
        if hz == 0 || hz > MAX_FREQ_HZ_4X4 {
            return Err(Error::InvalidParam);
        }
        self.dci_replace(DCI_FREQ_HZ, 4, &[hz], 1)
    }

    /// Reads back configured ranging frequency in Hz.
    pub fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut b = [0u8; 4];
        self.dci_read(DCI_FREQ_HZ, &mut b)?;
        Ok(b[1])
    }

    /// Configures the target order reported by the sensor (Closest vs Strongest).
    pub fn set_target_order(&mut self, order: TargetOrder) -> Result<(), Error<BUS::Error>> {
        self.dci_replace(DCI_TARGET_ORDER, 4, &[order as u8], 0)
    }

    /// Reads back the current target order configuration.
    pub fn get_target_order(&mut self) -> Result<TargetOrder, Error<BUS::Error>> {
        let mut b = [0u8; 4];
        self.dci_read(DCI_TARGET_ORDER, &mut b)?;
        match b[0] {
            1 => Ok(TargetOrder::Closest),
            2 => Ok(TargetOrder::Strongest),
            _ => Err(Error::InvalidParam),
        }
    }

    /// Sets the sharpener percentage (0% to 99%) to filter optical crosstalk and veiling glare.
    ///
    /// Corresponds to ST ULD `vl53l5cx_set_sharpener_percent`.
    pub fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), Error<BUS::Error>> {
        if percent >= 100 {
            return Err(Error::InvalidParam);
        }
        let sharpener = ((percent as u16 * 255) / 100) as u8;
        self.dci_replace(DCI_SHARPENER, 16, &[sharpener], 0x0D)
    }

    /// Reads back the active sharpener percentage (0% to 99%).
    ///
    /// Corresponds to ST ULD `vl53l5cx_get_sharpener_percent`.
    pub fn get_sharpener_percent(&mut self) -> Result<u8, Error<BUS::Error>> {
        let mut b = [0u8; 16];
        self.dci_read(DCI_SHARPENER, &mut b)?;
        let percent = (b[0x0D] as u16 * 100 / 255) as u8;
        Ok(percent)
    }

    /// Starts continuous ranging with 4x4 or 8x8 configuration.
    pub fn start_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        let mut zone = [0u8; 8];
        self.dci_read(DCI_ZONE_CONFIG, &mut zone)?;
        let resolution = u16::from(zone[0]) * u16::from(zone[1]);
        if resolution as usize != codec::ZONES_4X4 && resolution as usize != codec::ZONES_8X8 {
            return Err(Error::UnsupportedResolution(resolution));
        }

        self.data_read_size = 0;
        self.streamcount = 255;

        let cfg = codec::build_output_config(resolution);
        let frame_size = cfg.data_read_size as usize;
        if frame_size > TEMP_SIZE {
            return Err(Error::InvalidParam);
        }

        let list: [u8; 48] = codec::words_to_le(&cfg.list);
        self.dci_write(DCI_OUTPUT_LIST, &list)?;
        let header: [u8; 8] = codec::words_to_le(&[cfg.data_read_size, cfg.entries]);
        self.dci_write(DCI_OUTPUT_CONFIG, &header)?;
        let enables: [u8; 16] = codec::words_to_le(&codec::OUTPUT_ENABLES);
        self.dci_write(DCI_OUTPUT_ENABLES, &enables)?;

        self.wr(0x7FFF, 0x00)?;
        self.wr(0x09, 0x05)?;
        self.wr(0x7FFF, 0x02)?;

        self.write_slice(UI_CMD_END - 3, &[0x00, 0x03, 0x00, 0x00])?;
        self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)?;

        let mut check = [0u8; 12];
        self.dci_read(DCI_UI_RANGE_DATA, &mut check)?;
        let reported = u16::from_le_bytes([check[8], check[9]]);
        if u32::from(reported) != cfg.data_read_size {
            return Err(Error::OutputSizeMismatch {
                expected: cfg.data_read_size,
                reported,
            });
        }
        self.data_read_size = frame_size;
        self.streamcount = 255;
        Ok(())
    }

    /// Stops ranging.
    pub fn stop_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
        self.data_read_size = 0;
        let mut first_error = None;

        let mut flag = [0u8; 4];
        read_multi_blocking(&mut self.bus, &mut self.delay, self.addr, 0x2FFC, &mut flag)
            .map_err(Error::Bus)?;
        if u32::from_le_bytes(flag) != 0x4FF {
            self.wr(0x7FFF, 0x00)?;
            self.wr(0x15, 0x16)?;
            self.wr(0x14, 0x01)?;
            let mut stopped = false;
            let mut last = 0;
            for _ in 0..500u16 {
                last = self.rd(0x06)?;
                if last & 0x80 != 0 {
                    stopped = true;
                    break;
                }
                self.delay.delay_ms(POLL_INTERVAL_MS);
            }
            if !stopped {
                first_error = Some(Error::Timeout {
                    register: 0x06,
                    last,
                });
            }
        }

        let status0 = self.rd(0x06)?;
        if status0 & 0x80 != 0 {
            let status1 = match self.rd(0x07) {
                Ok(v) => v,
                Err(_) => status0,
            };
            if status1 != 0x84 && status1 != 0x85 && first_error.is_none() {
                first_error = Some(Error::Go2Status(status1));
            }
        }

        self.wr(0x7FFF, 0x00)?;
        self.wr(0x14, 0x00)?;
        self.wr(0x15, 0x00)?;
        self.wr(0x09, 0x04)?;
        self.wr(0x7FFF, 0x02)?;

        match first_error {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Checks if a new frame is available.
    pub fn check_data_ready(&mut self) -> Result<bool, Error<BUS::Error>> {
        let mut regs = [0u8; 4];
        read_multi_blocking(&mut self.bus, &mut self.delay, self.addr, 0x0000, &mut regs)
            .map_err(Error::Bus)?;

        match codec::decode_data_ready(regs, self.streamcount) {
            codec::ReadyState::Ready(new_sc) => {
                self.streamcount = new_sc;
                Ok(true)
            }
            codec::ReadyState::NotReady => Ok(false),
            codec::ReadyState::Abnormal(code) => Err(Error::Go2Status(code)),
        }
    }

    /// Reads and parses the current 4x4 ranging frame.
    pub fn get_ranging_data(&mut self) -> Result<Frame4x4, Error<BUS::Error>> {
        let n = self.data_read_size;
        if n == 0 {
            return Err(Error::NotRanging);
        }
        self.read_temp(0x0000, n)?;
        let new_streamcount = self.temp[0];
        let frame = codec::parse_frame(&mut self.temp[..n]).map_err(Error::Frame)?;
        self.streamcount = new_streamcount;
        Ok(frame)
    }

    /// Reads and parses the current 8x8 ranging frame (64 zones).
    pub fn get_ranging_data_8x8(&mut self) -> Result<Frame8x8, Error<BUS::Error>> {
        let n = self.data_read_size;
        if n == 0 {
            return Err(Error::NotRanging);
        }
        self.read_temp(0x0000, n)?;
        let new_streamcount = self.temp[0];
        let frame = codec::parse_frame_8x8(&mut self.temp[..n]).map_err(Error::Frame)?;
        self.streamcount = new_streamcount;
        Ok(frame)
    }
}

// ============================================================================
// Optional Async Driver (embedded-hal-async 1.0)
// ============================================================================

#[cfg(feature = "async")]
pub mod asynch {
    use super::*;
    use embassy_time::Timer;
    use embedded_hal_async::i2c::{I2c as AsyncI2c, Operation as AsyncOperation};

    /// Reads Device ID / Revision ID asynchronously without side effects.
    pub async fn probe<B: AsyncI2c>(bus: &mut B, addr: u8) -> Result<(u8, u8), B::Error> {
        let mut page_err = None;
        for attempt in 0..3 {
            match bus.write(addr, &[0x7F, 0xFF, 0x00]).await {
                Ok(()) => {
                    page_err = None;
                    break;
                }
                Err(e) => {
                    page_err = Some(e);
                    Timer::after_millis(15 * (attempt + 1) as u64).await;
                }
            }
        }
        if let Some(err) = page_err {
            return Err(err);
        }
        Timer::after_millis(5).await;

        let mut id = [0u8; 2];
        let res = bus.write_read(addr, &[0x00, 0x00], &mut id).await;

        let restore = bus.write(addr, &[0x7F, 0xFF, 0x02]).await;
        res?;
        restore?;
        Ok((id[0], id[1]))
    }

    async fn write_multi_async<B: AsyncI2c>(
        bus: &mut B,
        addr: u8,
        reg: u16,
        data: &[u8],
    ) -> Result<(), B::Error> {
        let reg_bytes = reg.to_be_bytes();
        let mut last_err = None;
        for attempt in 0..5 {
            let mut ops = [
                AsyncOperation::Write(&reg_bytes),
                AsyncOperation::Write(data),
            ];
            match bus.transaction(addr, &mut ops).await {
                Ok(()) => return Ok(()),
                Err(e) => {
                    last_err = Some(e);
                    Timer::after_millis(10 * (attempt + 1) as u64).await;
                }
            }
        }
        match last_err {
            Some(err) => Err(err),
            None => Ok(()),
        }
    }

    async fn read_multi_async<B: AsyncI2c>(
        bus: &mut B,
        addr: u8,
        reg: u16,
        buf: &mut [u8],
    ) -> Result<(), B::Error> {
        let reg_bytes = reg.to_be_bytes();
        let mut last_err = None;
        for attempt in 0..5 {
            match bus.write_read(addr, &reg_bytes, buf).await {
                Ok(()) => return Ok(()),
                Err(e) => {
                    last_err = Some(e);
                    Timer::after_millis(15 * (attempt + 1) as u64).await;
                }
            }
        }
        match last_err {
            Some(err) => Err(err),
            None => Ok(()),
        }
    }

    #[cfg(feature = "firmware-blob")]
    async fn write_fw_bank_async<B: AsyncI2c>(
        bus: &mut B,
        addr: u8,
        page: u8,
        data: &[u8],
    ) -> Result<(), Error<B::Error>> {
        let mut page_err = None;
        for attempt in 0..20 {
            match bus.write(addr, &[0x7F, 0xFF, page]).await {
                Ok(()) => {
                    page_err = None;
                    break;
                }
                Err(e) => {
                    page_err = Some(e);
                    Timer::after_millis(15 * (attempt + 1) as u64).await;
                }
            }
        }
        if let Some(err) = page_err {
            return Err(Error::Bus(err));
        }

        Timer::after_millis(10).await;

        let reg_bytes = [0x00, 0x00];
        let mut ops = [
            AsyncOperation::Write(&reg_bytes),
            AsyncOperation::Write(data),
        ];
        let mut bank_err = None;
        for attempt in 0..5 {
            match bus.transaction(addr, &mut ops).await {
                Ok(()) => {
                    bank_err = None;
                    break;
                }
                Err(e) => {
                    bank_err = Some(e);
                    Timer::after_millis(20 * (attempt + 1) as u64).await;
                }
            }
        }
        if let Some(err) = bank_err {
            return Err(Error::Bus(err));
        }

        Timer::after_millis(50).await;
        Ok(())
    }

    /// Asynchronous VL53L5CX driver instance.
    pub struct Vl53l5cx<BUS> {
        bus: BUS,
        addr: u8,
        temp: [u8; TEMP_SIZE],
        #[allow(dead_code)]
        offset_data: [u8; codec::OFFSET_BUFFER_SIZE],
        streamcount: u8,
        data_read_size: usize,
    }

    impl<BUS: AsyncI2c> Vl53l5cx<BUS> {
        /// Creates a new async driver instance.
        pub const fn new(bus: BUS, addr: u8) -> Self {
            Self {
                bus,
                addr,
                temp: [0u8; TEMP_SIZE],
                offset_data: [0u8; codec::OFFSET_BUFFER_SIZE],
                streamcount: 255,
                data_read_size: 0,
            }
        }

        pub const fn address(&self) -> u8 {
            self.addr
        }

        pub fn into_inner(self) -> BUS {
            self.bus
        }

        async fn wr(&mut self, reg: u16, value: u8) -> Result<(), Error<BUS::Error>> {
            let [hi, lo] = reg.to_be_bytes();
            let mut last_err = None;
            for attempt in 0..5 {
                match self.bus.write(self.addr, &[hi, lo, value]).await {
                    Ok(()) => {
                        if reg == 0x7FFF {
                            Timer::after_millis(2).await;
                        }
                        return Ok(());
                    }
                    Err(e) => {
                        last_err = Some(e);
                        Timer::after_millis(5 * (attempt + 1) as u64).await;
                    }
                }
            }
            if let Some(err) = last_err {
                Err(Error::Bus(err))
            } else {
                Err(Error::InvalidParam)
            }
        }

        async fn rd(&mut self, reg: u16) -> Result<u8, Error<BUS::Error>> {
            let mut b = [0u8; 1];
            read_multi_async(&mut self.bus, self.addr, reg, &mut b)
                .await
                .map_err(Error::Bus)?;
            Ok(b[0])
        }

        async fn write_slice(&mut self, reg: u16, data: &[u8]) -> Result<(), Error<BUS::Error>> {
            write_multi_async(&mut self.bus, self.addr, reg, data)
                .await
                .map_err(Error::Bus)
        }

        async fn write_temp(&mut self, reg: u16, len: usize) -> Result<(), Error<BUS::Error>> {
            let buf = self.temp.get(..len).ok_or(Error::InvalidParam)?;
            write_multi_async(&mut self.bus, self.addr, reg, buf)
                .await
                .map_err(Error::Bus)
        }

        async fn read_temp(&mut self, reg: u16, len: usize) -> Result<(), Error<BUS::Error>> {
            let buf = self.temp.get_mut(..len).ok_or(Error::InvalidParam)?;
            read_multi_async(&mut self.bus, self.addr, reg, buf)
                .await
                .map_err(Error::Bus)
        }

        async fn poll_for_answer(
            &mut self,
            size: usize,
            pos: usize,
            reg: u16,
            mask: u8,
            expected: u8,
        ) -> Result<(), Error<BUS::Error>> {
            let mut buf = [0u8; 4];
            let buf = buf.get_mut(..size).ok_or(Error::InvalidParam)?;
            let mut last_bus_err = None;
            for _attempt in 0..POLL_ATTEMPTS {
                Timer::after_millis(POLL_INTERVAL_MS as u64).await;
                match read_multi_async(&mut self.bus, self.addr, reg, buf).await {
                    Ok(()) => {
                        last_bus_err = None;
                        match codec::poll_outcome(buf, pos, mask, expected) {
                            codec::PollOutcome::Done => return Ok(()),
                            codec::PollOutcome::McuError(s) => return Err(Error::McuError(s)),
                            codec::PollOutcome::Pending => {}
                        }
                    }
                    Err(e) => {
                        last_bus_err = Some(e);
                    }
                }
            }
            if let Some(err) = last_bus_err {
                Err(Error::Bus(err))
            } else {
                let last = match buf.get(pos) {
                    Some(&val) => val,
                    None => 0,
                };
                Err(Error::Timeout {
                    register: reg,
                    last,
                })
            }
        }

        #[cfg(feature = "firmware-blob")]
        async fn poll_for_mcu_boot(&mut self) -> Result<(), Error<BUS::Error>> {
            let mut status0 = 0xFF;
            let mut last_err = None;
            for _attempt in 0..MCU_BOOT_ATTEMPTS {
                match self.rd(0x06).await {
                    Ok(s) => {
                        last_err = None;
                        status0 = s;
                        if status0 & 0x80 != 0 {
                            let status1 = match self.rd(0x07).await {
                                Ok(v) => v,
                                Err(_) => status0,
                            };
                            return Err(Error::Go2Status(status1));
                        }
                        if status0 & 0x01 != 0 || status0 == 0x00 {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        last_err = Some(e);
                    }
                }
                Timer::after_millis(2).await;
            }

            if let Some(err) = last_err {
                Err(err)
            } else {
                Err(Error::Timeout {
                    register: 0x06,
                    last: status0,
                })
            }
        }

        async fn dci_read(&mut self, index: u16, out: &mut [u8]) -> Result<(), Error<BUS::Error>> {
            let n = out.len();
            if !n.is_multiple_of(4) || n + 12 > TEMP_SIZE {
                return Err(Error::InvalidParam);
            }
            let cmd = codec::dci_read_cmd(index, n as u16);
            self.write_slice(DCI_CMD_ADDR, &cmd).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;
            self.read_temp(UI_CMD_START, n + 12).await?;
            codec::swap_words(&mut self.temp[..n + 12]);
            out.copy_from_slice(&self.temp[4..4 + n]);
            Ok(())
        }

        async fn dci_write(&mut self, index: u16, data: &[u8]) -> Result<(), Error<BUS::Error>> {
            let (address, len) =
                codec::dci_write_frame(index, data, &mut self.temp).ok_or(Error::InvalidParam)?;
            self.write_temp(address, len).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03).await
        }

        async fn dci_replace(
            &mut self,
            index: u16,
            size: usize,
            new_data: &[u8],
            pos: usize,
        ) -> Result<(), Error<BUS::Error>> {
            let mut local = [0u8; 64];
            if size > local.len() || pos + new_data.len() > size {
                return Err(Error::InvalidParam);
            }
            self.dci_read(index, &mut local[..size]).await?;
            local[pos..pos + new_data.len()].copy_from_slice(new_data);
            self.dci_write(index, &local[..size]).await
        }

        pub async fn is_alive(&mut self) -> Result<(), Error<BUS::Error>> {
            let (device_id, revision_id) =
                probe(&mut self.bus, self.addr).await.map_err(Error::Bus)?;
            if is_vl53l5cx_id(device_id, revision_id) {
                Ok(())
            } else {
                Err(Error::NotAlive {
                    device_id,
                    revision_id,
                })
            }
        }

        pub async fn set_i2c_address(&mut self, new_addr: u8) -> Result<(), Error<BUS::Error>> {
            if new_addr == 0 || new_addr > 0x7F {
                return Err(Error::InvalidParam);
            }
            self.wr(0x7FFF, 0x00).await?;
            self.wr(0x0004, new_addr).await?;
            self.addr = new_addr;
            self.wr(0x7FFF, 0x02).await
        }

        #[cfg(feature = "firmware-blob")]
        pub async fn init(&mut self) -> Result<(), Error<BUS::Error>> {
            self.data_read_size = 0;
            self.streamcount = 255;

            let _ = self.wr(0x7FFF, 0x00).await;
            let _ = self.wr(0x0009, 0x04).await;
            let _ = self.wr(0x000F, 0x40).await;
            let _ = self.wr(0x000A, 0x03).await;
            let _ = self.rd(0x7FFF).await;
            let _ = self.wr(0x000C, 0x01).await;

            let _ = self.wr(0x0101, 0x00).await;
            let _ = self.wr(0x0102, 0x00).await;
            let _ = self.wr(0x010A, 0x01).await;
            let _ = self.wr(0x4002, 0x01).await;
            let _ = self.wr(0x4002, 0x00).await;
            let _ = self.wr(0x010A, 0x03).await;
            let _ = self.wr(0x0103, 0x01).await;
            let _ = self.wr(0x000C, 0x00).await;
            let _ = self.wr(0x000F, 0x43).await;
            Timer::after_millis(1).await;

            let _ = self.wr(0x000F, 0x40).await;
            let _ = self.wr(0x000A, 0x01).await;
            Timer::after_millis(100).await;

            self.wr(0x7FFF, 0x00).await?;
            self.poll_for_answer(1, 0, 0x06, 0xFF, 0x01).await?;

            self.wr(0x000E, 0x01).await?;
            self.wr(0x7FFF, 0x02).await?;

            self.wr(0x03, 0x0D).await?;
            self.wr(0x7FFF, 0x01).await?;
            self.poll_for_answer(1, 0, 0x21, 0x10, 0x10).await?;
            self.wr(0x7FFF, 0x00).await?;

            let _ = self.rd(0x7FFF).await?;
            self.wr(0x0C, 0x01).await?;

            self.wr(0x7FFF, 0x00).await?;
            self.wr(0x101, 0x00).await?;
            self.wr(0x102, 0x00).await?;
            self.wr(0x010A, 0x01).await?;
            self.wr(0x4002, 0x01).await?;
            self.wr(0x4002, 0x00).await?;
            self.wr(0x010A, 0x03).await?;
            self.wr(0x103, 0x01).await?;
            self.wr(0x400F, 0x00).await?;
            self.wr(0x21A, 0x43).await?;
            self.wr(0x21A, 0x03).await?;
            self.wr(0x21A, 0x01).await?;
            self.wr(0x21A, 0x00).await?;
            self.wr(0x219, 0x00).await?;
            self.wr(0x21B, 0x00).await?;

            self.wr(0x7FFF, 0x00).await?;
            let _ = self.rd(0x7FFF).await?;
            self.wr(0x0C, 0x00).await?;
            self.wr(0x7FFF, 0x01).await?;
            Timer::after_millis(5).await;
            self.wr(0x20, 0x07).await?;
            self.wr(0x20, 0x06).await?;

            write_fw_bank_async(&mut self.bus, self.addr, 0x09, FW_PAGE_09).await?;
            write_fw_bank_async(&mut self.bus, self.addr, 0x0A, FW_PAGE_0A).await?;
            write_fw_bank_async(&mut self.bus, self.addr, 0x0B, FW_PAGE_0B).await?;
            self.wr(0x7FFF, 0x01).await?;
            Timer::after_millis(5).await;

            self.wr(0x7FFF, 0x02).await?;
            Timer::after_millis(2).await;
            self.wr(0x03, 0x0D).await?;
            self.wr(0x7FFF, 0x01).await?;
            Timer::after_millis(5).await;
            self.poll_for_answer(1, 0, 0x21, 0x10, 0x10).await?;

            self.wr(0x7FFF, 0x00).await?;
            let _ = self.rd(0x7FFF).await?;
            self.wr(0x0C, 0x01).await?;

            self.wr(0x7FFF, 0x00).await?;
            self.wr(0x114, 0x00).await?;
            self.wr(0x115, 0x00).await?;
            self.wr(0x116, 0x42).await?;
            self.wr(0x117, 0x00).await?;
            self.wr(0x0B, 0x00).await?;
            let _ = self.rd(0x7FFF).await?;
            self.wr(0x0C, 0x00).await?;
            self.wr(0x0B, 0x01).await?;
            Timer::after_millis(150).await;
            self.poll_for_mcu_boot().await?;

            self.wr(0x7FFF, 0x02).await?;
            Timer::after_millis(20).await;

            self.write_slice(0x2FD8, GET_NVM_CMD).await?;
            self.poll_for_answer(4, 0, UI_CMD_STATUS, 0xFF, 0x02)
                .await?;
            self.read_temp(UI_CMD_START, codec::NVM_DATA_SIZE).await?;
            self.offset_data
                .copy_from_slice(&self.temp[..codec::OFFSET_BUFFER_SIZE]);
            codec::prepare_offset_4x4(&mut self.temp).map_err(|_| Error::InvalidParam)?;
            self.write_temp(0x2E18, codec::OFFSET_BUFFER_SIZE).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            self.temp[..codec::XTALK_BUFFER_SIZE].copy_from_slice(DEFAULT_XTALK);
            codec::prepare_xtalk_4x4(&mut self.temp).map_err(|_| Error::InvalidParam)?;
            self.write_temp(0x2CF8, codec::XTALK_BUFFER_SIZE).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            self.write_slice(0x2C34, DEFAULT_CONFIGURATION).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            self.dci_write(DCI_PIPE_CONTROL, &[1, 0x00, 0x01, 0x00])
                .await?;
            self.dci_write(DCI_SINGLE_RANGE, &1u32.to_le_bytes())
                .await?;
            self.dci_replace(DCI_GLARE_FILTER, 40, &[1], 0x26).await?;
            self.dci_replace(DCI_GLARE_FILTER, 40, &[1], 0x25).await?;

            self.set_resolution_4x4().await?;
            Ok(())
        }

        pub async fn set_resolution_4x4(&mut self) -> Result<(), Error<BUS::Error>> {
            let mut dss = [0u8; 16];
            self.dci_read(DCI_DSS_CONFIG, &mut dss).await?;
            dss[0x04] = 64;
            dss[0x06] = 64;
            dss[0x09] = 4;
            self.dci_write(DCI_DSS_CONFIG, &dss).await?;

            let mut zone = [0u8; 8];
            self.dci_read(DCI_ZONE_CONFIG, &mut zone).await?;
            zone[0x00] = 4;
            zone[0x01] = 4;
            zone[0x04] = 8;
            zone[0x05] = 8;
            self.dci_write(DCI_ZONE_CONFIG, &zone).await?;

            Ok(())
        }

        /// Sets sensor resolution to 8x8 (64 zones).
        #[cfg(feature = "firmware-blob")]
        pub async fn set_resolution_8x8(&mut self) -> Result<(), Error<BUS::Error>> {
            let mut dss = [0u8; 16];
            self.dci_read(DCI_DSS_CONFIG, &mut dss).await?;
            dss[0x04] = 16;
            dss[0x06] = 16;
            dss[0x09] = 1;
            self.dci_write(DCI_DSS_CONFIG, &dss).await?;

            let mut zone = [0u8; 8];
            self.dci_read(DCI_ZONE_CONFIG, &mut zone).await?;
            zone[0x00] = 8;
            zone[0x01] = 8;
            zone[0x04] = 4;
            zone[0x05] = 4;
            self.dci_write(DCI_ZONE_CONFIG, &zone).await?;

            // Send 8x8 offset calibration data per ST ULD lines 953 and 143-152
            self.temp[..codec::OFFSET_BUFFER_SIZE].copy_from_slice(&self.offset_data);
            codec::prepare_offset_8x8(&mut self.temp).map_err(|_| Error::InvalidParam)?;
            self.write_temp(0x2E18, codec::OFFSET_BUFFER_SIZE).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            // Send 8x8 default Xtalk data per ST ULD lines 954 and 209-212
            self.temp[..codec::XTALK_BUFFER_SIZE].copy_from_slice(DEFAULT_XTALK);
            self.write_temp(0x2CF8, codec::XTALK_BUFFER_SIZE).await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            Ok(())
        }

        pub async fn set_ranging_mode_continuous(&mut self) -> Result<(), Error<BUS::Error>> {
            let mut buf = [0u8; 8];
            self.dci_read(DCI_RANGING_MODE, &mut buf).await?;
            buf[0x01] = 0x01;
            buf[0x03] = 0x03;
            self.dci_write(DCI_RANGING_MODE, &buf).await?;
            self.dci_write(DCI_SINGLE_RANGE, &[0x00, 0x00, 0x00, 0x00])
                .await
        }

        pub async fn set_ranging_frequency_hz(&mut self, hz: u8) -> Result<(), Error<BUS::Error>> {
            if hz == 0 || hz > MAX_FREQ_HZ_4X4 {
                return Err(Error::InvalidParam);
            }
            self.dci_replace(DCI_FREQ_HZ, 4, &[hz], 1).await
        }

        pub async fn get_ranging_frequency_hz(&mut self) -> Result<u8, Error<BUS::Error>> {
            let mut b = [0u8; 4];
            self.dci_read(DCI_FREQ_HZ, &mut b).await?;
            Ok(b[1])
        }

        /// Configures the target order reported by the sensor (Closest vs Strongest).
        pub async fn set_target_order(
            &mut self,
            order: TargetOrder,
        ) -> Result<(), Error<BUS::Error>> {
            self.dci_replace(DCI_TARGET_ORDER, 4, &[order as u8], 0)
                .await
        }

        /// Reads back the current target order configuration.
        pub async fn get_target_order(&mut self) -> Result<TargetOrder, Error<BUS::Error>> {
            let mut b = [0u8; 4];
            self.dci_read(DCI_TARGET_ORDER, &mut b).await?;
            match b[0] {
                1 => Ok(TargetOrder::Closest),
                2 => Ok(TargetOrder::Strongest),
                _ => Err(Error::InvalidParam),
            }
        }

        /// Sets the sharpener percentage (0% to 99%) to filter optical crosstalk and veiling glare.
        ///
        /// Corresponds to ST ULD `vl53l5cx_set_sharpener_percent`.
        pub async fn set_sharpener_percent(
            &mut self,
            percent: u8,
        ) -> Result<(), Error<BUS::Error>> {
            if percent >= 100 {
                return Err(Error::InvalidParam);
            }
            let sharpener = ((percent as u16 * 255) / 100) as u8;
            self.dci_replace(DCI_SHARPENER, 16, &[sharpener], 0x0D)
                .await
        }

        /// Reads back the active sharpener percentage (0% to 99%).
        ///
        /// Corresponds to ST ULD `vl53l5cx_get_sharpener_percent`.
        pub async fn get_sharpener_percent(&mut self) -> Result<u8, Error<BUS::Error>> {
            let mut b = [0u8; 16];
            self.dci_read(DCI_SHARPENER, &mut b).await?;
            let percent = (b[0x0D] as u16 * 100 / 255) as u8;
            Ok(percent)
        }

        pub async fn start_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
            let mut zone = [0u8; 8];
            self.dci_read(DCI_ZONE_CONFIG, &mut zone).await?;
            let resolution = u16::from(zone[0]) * u16::from(zone[1]);
            if resolution as usize != codec::ZONES_4X4 && resolution as usize != codec::ZONES_8X8 {
                return Err(Error::UnsupportedResolution(resolution));
            }

            self.data_read_size = 0;
            self.streamcount = 255;

            let cfg = codec::build_output_config(resolution);
            let frame_size = cfg.data_read_size as usize;
            if frame_size > TEMP_SIZE {
                return Err(Error::InvalidParam);
            }

            let list: [u8; 48] = codec::words_to_le(&cfg.list);
            self.dci_write(DCI_OUTPUT_LIST, &list).await?;
            let header: [u8; 8] = codec::words_to_le(&[cfg.data_read_size, cfg.entries]);
            self.dci_write(DCI_OUTPUT_CONFIG, &header).await?;
            let enables: [u8; 16] = codec::words_to_le(&codec::OUTPUT_ENABLES);
            self.dci_write(DCI_OUTPUT_ENABLES, &enables).await?;

            self.wr(0x7FFF, 0x00).await?;
            self.wr(0x09, 0x05).await?;
            self.wr(0x7FFF, 0x02).await?;

            self.write_slice(UI_CMD_END - 3, &[0x00, 0x03, 0x00, 0x00])
                .await?;
            self.poll_for_answer(4, 1, UI_CMD_STATUS, 0xFF, 0x03)
                .await?;

            let mut check = [0u8; 12];
            self.dci_read(DCI_UI_RANGE_DATA, &mut check).await?;
            let reported = u16::from_le_bytes([check[8], check[9]]);
            if u32::from(reported) != cfg.data_read_size {
                return Err(Error::OutputSizeMismatch {
                    expected: cfg.data_read_size,
                    reported,
                });
            }
            self.data_read_size = frame_size;
            self.streamcount = 255;
            Ok(())
        }

        pub async fn stop_ranging(&mut self) -> Result<(), Error<BUS::Error>> {
            self.data_read_size = 0;
            let mut first_error = None;

            let mut flag = [0u8; 4];
            read_multi_async(&mut self.bus, self.addr, 0x2FFC, &mut flag)
                .await
                .map_err(Error::Bus)?;
            if u32::from_le_bytes(flag) != 0x4FF {
                self.wr(0x7FFF, 0x00).await?;
                self.wr(0x15, 0x16).await?;
                self.wr(0x14, 0x01).await?;
                let mut stopped = false;
                let mut last = 0;
                for _ in 0..500u16 {
                    last = self.rd(0x06).await?;
                    if last & 0x80 != 0 {
                        stopped = true;
                        break;
                    }
                    Timer::after_millis(POLL_INTERVAL_MS as u64).await;
                }
                if !stopped {
                    first_error = Some(Error::Timeout {
                        register: 0x06,
                        last,
                    });
                }
            }

            let status0 = self.rd(0x06).await?;
            if status0 & 0x80 != 0 {
                let status1 = match self.rd(0x07).await {
                    Ok(v) => v,
                    Err(_) => status0,
                };
                if status1 != 0x84 && status1 != 0x85 && first_error.is_none() {
                    first_error = Some(Error::Go2Status(status1));
                }
            }

            self.wr(0x7FFF, 0x00).await?;
            self.wr(0x14, 0x00).await?;
            self.wr(0x15, 0x00).await?;
            self.wr(0x09, 0x04).await?;
            self.wr(0x7FFF, 0x02).await?;

            match first_error {
                Some(e) => Err(e),
                None => Ok(()),
            }
        }

        pub async fn check_data_ready(&mut self) -> Result<bool, Error<BUS::Error>> {
            let mut regs = [0u8; 4];
            read_multi_async(&mut self.bus, self.addr, 0x0000, &mut regs)
                .await
                .map_err(Error::Bus)?;

            match codec::decode_data_ready(regs, self.streamcount) {
                codec::ReadyState::Ready(new_sc) => {
                    self.streamcount = new_sc;
                    Ok(true)
                }
                codec::ReadyState::NotReady => Ok(false),
                codec::ReadyState::Abnormal(code) => Err(Error::Go2Status(code)),
            }
        }

        pub async fn get_ranging_data(&mut self) -> Result<Frame4x4, Error<BUS::Error>> {
            let n = self.data_read_size;
            if n == 0 {
                return Err(Error::NotRanging);
            }
            self.read_temp(0x0000, n).await?;
            let new_streamcount = self.temp[0];
            let frame = codec::parse_frame(&mut self.temp[..n]).map_err(Error::Frame)?;
            self.streamcount = new_streamcount;
            Ok(frame)
        }

        /// Reads and parses the current 8x8 ranging frame (64 zones).
        pub async fn get_ranging_data_8x8(&mut self) -> Result<Frame8x8, Error<BUS::Error>> {
            let n = self.data_read_size;
            if n == 0 {
                return Err(Error::NotRanging);
            }
            self.read_temp(0x0000, n).await?;
            let new_streamcount = self.temp[0];
            let frame = codec::parse_frame_8x8(&mut self.temp[..n]).map_err(Error::Frame)?;
            self.streamcount = new_streamcount;
            Ok(frame)
        }
    }
}

#[cfg(feature = "async")]
pub use asynch::{probe as probe_async, Vl53l5cx as AsyncVl53l5cx};

// ============================================================================
// Unit Tests
// ============================================================================

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn test_device_identification() {
        assert!(is_vl53l5cx_id(0xF0, 0x02));
        assert!(!is_vl53l5cx_id(0xEB, 0xAA)); // VL53L4CD
        assert!(!is_vl53l5cx_id(0xF0, 0x01));
    }

    #[derive(Default, Clone, Copy)]
    struct MockDelay;
    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, _ns: u32) {}
    }

    struct MockBus {
        page: u8,
        dev_id: u8,
        rev_id: u8,
        last_dci_cmd_index: u16,
        dci_data: HashMap<u16, Vec<u8>>,
        data_0000: Vec<u8>,
        status_0006: [u8; 4],
        status_0021: u8,
        read_2ffc: [u8; 4],
    }

    impl MockBus {
        fn new() -> Self {
            let mut dci = HashMap::new();
            dci.insert(DCI_DSS_CONFIG, vec![0u8; 16]);
            let mut zone = vec![0u8; 8];
            zone[0] = 4;
            zone[1] = 4;
            dci.insert(DCI_ZONE_CONFIG, zone);
            dci.insert(DCI_RANGING_MODE, vec![0u8; 8]);
            dci.insert(DCI_FREQ_HZ, vec![0, 15, 0, 0]);
            dci.insert(DCI_TARGET_ORDER, vec![1, 0, 0, 0]);
            let mut sharp = vec![0u8; 16];
            sharp[0x0D] = 89;
            dci.insert(DCI_SHARPENER, sharp);
            let mut ui_range = vec![0u8; 12];
            ui_range[8..10].copy_from_slice(&128u16.to_le_bytes());
            dci.insert(DCI_UI_RANGE_DATA, ui_range);

            Self {
                page: 2,
                dev_id: 0xF0,
                rev_id: 0x02,
                last_dci_cmd_index: 0,
                dci_data: dci,
                data_0000: vec![0u8; 4],
                status_0006: [0x00, 0x03, 0x00, 0x00],
                status_0021: 0x10,
                read_2ffc: [0xFF, 0x04, 0x00, 0x00],
            }
        }
    }

    impl embedded_hal::i2c::ErrorType for MockBus {
        type Error = core::convert::Infallible;
    }

    impl I2c for MockBus {
        fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            if write.len() >= 3 && write[0] == 0x7F && write[1] == 0xFF {
                self.page = write[2];
                return Ok(());
            }
            if write.len() >= 2 {
                let reg = u16::from_be_bytes([write[0], write[1]]);
                if reg == DCI_CMD_ADDR && write.len() >= 4 {
                    self.last_dci_cmd_index = u16::from_be_bytes([write[2], write[3]]);
                } else if (0x2F00..=0x3000).contains(&reg) && write.len() >= 14 {
                    let index = u16::from_be_bytes([write[2], write[3]]);
                    let n = write.len() - 14;
                    let mut payload = write[6..6 + n].to_vec();
                    codec::swap_words(&mut payload);
                    if index == DCI_OUTPUT_CONFIG && payload.len() >= 4 {
                        let data_read_size =
                            u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                        if let Some(ui_range) = self.dci_data.get_mut(&DCI_UI_RANGE_DATA) {
                            ui_range[8..10].copy_from_slice(&(data_read_size as u16).to_le_bytes());
                        }
                    }
                    self.dci_data.insert(index, payload);
                }
            }
            Ok(())
        }

        fn write_read(
            &mut self,
            _address: u8,
            write: &[u8],
            read: &mut [u8],
        ) -> Result<(), Self::Error> {
            if write.len() >= 2 {
                let reg = u16::from_be_bytes([write[0], write[1]]);
                match reg {
                    0x0000 => {
                        if self.page == 0 && read.len() >= 2 {
                            read[0] = self.dev_id;
                            read[1] = self.rev_id;
                        } else {
                            let n = read.len().min(self.data_0000.len());
                            read[..n].copy_from_slice(&self.data_0000[..n]);
                        }
                    }
                    0x0006 | 0x2C00 => {
                        let n = read.len().min(4);
                        read[..n].copy_from_slice(&self.status_0006[..n]);
                    }
                    0x0021 => {
                        if !read.is_empty() {
                            read[0] = self.status_0021;
                        }
                    }
                    0x2C04 => {
                        if let Some(data) = self.dci_data.get(&self.last_dci_cmd_index) {
                            let n = data.len();
                            let mut buf = vec![0u8; n + 12];
                            buf[4..4 + n].copy_from_slice(data);
                            codec::swap_words(&mut buf);
                            let to_copy = read.len().min(buf.len());
                            read[..to_copy].copy_from_slice(&buf[..to_copy]);
                        }
                    }
                    0x2FFC => {
                        let n = read.len().min(4);
                        read[..n].copy_from_slice(&self.read_2ffc[..n]);
                    }
                    _ => {}
                }
            }
            Ok(())
        }

        fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            if operations.len() >= 2 {
                if let (Operation::Write(reg_bytes), Operation::Write(data)) =
                    (&operations[0], &operations[1])
                {
                    if reg_bytes.len() >= 2 {
                        let reg = u16::from_be_bytes([reg_bytes[0], reg_bytes[1]]);
                        if reg == DCI_CMD_ADDR && data.len() >= 4 {
                            self.last_dci_cmd_index = u16::from_be_bytes([data[0], data[1]]);
                        } else if (0x2F00..=0x3000).contains(&reg) && data.len() >= 12 {
                            let index = u16::from_be_bytes([data[0], data[1]]);
                            let n = data.len() - 12;
                            let mut payload = data[4..4 + n].to_vec();
                            codec::swap_words(&mut payload);
                            if index == DCI_OUTPUT_CONFIG && payload.len() >= 4 {
                                let data_read_size = u32::from_le_bytes([
                                    payload[0], payload[1], payload[2], payload[3],
                                ]);
                                if let Some(ui_range) = self.dci_data.get_mut(&DCI_UI_RANGE_DATA) {
                                    ui_range[8..10]
                                        .copy_from_slice(&(data_read_size as u16).to_le_bytes());
                                }
                            }
                            self.dci_data.insert(index, payload);
                        }
                    }
                }
            }
            Ok(())
        }
    }

    fn synthetic_4x4_frame() -> Vec<u8> {
        let cfg = codec::build_output_config(16);
        let mut f = vec![0u8; 16];
        f[8..10].copy_from_slice(&1u16.to_be_bytes());
        f.extend_from_slice(&cfg.list[1].to_le_bytes());
        let mut meta = [0u8; 12];
        meta[8] = 28;
        f.extend_from_slice(&meta);
        f.extend_from_slice(&cfg.list[2].to_le_bytes());
        f.extend_from_slice(&[0u8; 4]);
        f.extend_from_slice(&cfg.list[5].to_le_bytes());
        f.extend_from_slice(&[1u8; 16]);
        f.extend_from_slice(&cfg.list[8].to_le_bytes());
        let mut dist = Vec::new();
        for z in 0..16i16 {
            dist.extend_from_slice(&((120 + z) * 4).to_le_bytes());
        }
        f.extend_from_slice(&dist);
        f.extend_from_slice(&cfg.list[10].to_le_bytes());
        f.extend_from_slice(&[5u8; 16]);
        while f.len() < cfg.data_read_size as usize {
            f.push(0);
        }
        let n = f.len();
        f[n - 4..n - 2].copy_from_slice(&1u16.to_be_bytes());
        f[n - 2..n].copy_from_slice(&[0xFF, 0xFF]);
        codec::swap_words(&mut f);
        f
    }

    #[cfg(feature = "firmware-blob")]
    fn synthetic_8x8_frame() -> Vec<u8> {
        let cfg = codec::build_output_config(64);
        let mut f = vec![0u8; 16];
        f[8..10].copy_from_slice(&2u16.to_be_bytes());
        f.extend_from_slice(&cfg.list[1].to_le_bytes());
        let mut meta = [0u8; 12];
        meta[8] = 32;
        f.extend_from_slice(&meta);
        f.extend_from_slice(&cfg.list[2].to_le_bytes());
        f.extend_from_slice(&[0u8; 4]);
        f.extend_from_slice(&cfg.list[5].to_le_bytes());
        f.extend_from_slice(&[1u8; 64]);
        f.extend_from_slice(&cfg.list[8].to_le_bytes());
        let mut dist = Vec::new();
        for z in 0..64i16 {
            dist.extend_from_slice(&((220 + z) * 4).to_le_bytes());
        }
        f.extend_from_slice(&dist);
        f.extend_from_slice(&cfg.list[10].to_le_bytes());
        f.extend_from_slice(&[5u8; 64]);
        while f.len() < cfg.data_read_size as usize {
            f.push(0);
        }
        codec::swap_words(&mut f);
        f
    }

    #[test]
    fn test_mock_blocking_probe_success() {
        let mut bus = MockBus::new();
        let mut delay = MockDelay;
        let res = probe(&mut bus, &mut delay, DEFAULT_ADDR);
        assert_eq!(res, Ok((0xF0, 0x02)));
        assert_eq!(bus.page, 2); // Must restore page to 2
    }

    #[test]
    fn test_driver_is_alive_and_address_change() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);
        assert_eq!(sensor.address(), DEFAULT_ADDR);
        assert!(sensor.is_alive().is_ok());

        assert!(sensor.set_i2c_address(0x35).is_ok());
        assert_eq!(sensor.address(), 0x35);

        assert_eq!(sensor.set_i2c_address(0x00), Err(Error::InvalidParam));
        assert_eq!(sensor.set_i2c_address(0x80), Err(Error::InvalidParam));

        let mut dead_bus = MockBus::new();
        dead_bus.dev_id = 0xAA;
        let mut dead_sensor = Vl53l5cx::new(dead_bus, MockDelay, DEFAULT_ADDR);
        assert_eq!(
            dead_sensor.is_alive(),
            Err(Error::NotAlive {
                device_id: 0xAA,
                revision_id: 0x02
            })
        );
    }

    #[test]
    fn test_target_order_configuration() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert_eq!(sensor.get_target_order(), Ok(TargetOrder::Closest));
        assert!(sensor.set_target_order(TargetOrder::Strongest).is_ok());
        assert_eq!(sensor.get_target_order(), Ok(TargetOrder::Strongest));
        assert!(sensor.set_target_order(TargetOrder::Closest).is_ok());
        assert_eq!(sensor.get_target_order(), Ok(TargetOrder::Closest));
    }

    #[test]
    fn test_sharpener_configuration() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert_eq!(sensor.get_sharpener_percent(), Ok(34)); // 89 * 100 / 255 = 34
        assert!(sensor.set_sharpener_percent(0).is_ok());
        assert_eq!(sensor.get_sharpener_percent(), Ok(0));

        assert!(sensor.set_sharpener_percent(99).is_ok());
        assert_eq!(sensor.get_sharpener_percent(), Ok(98));

        assert_eq!(sensor.set_sharpener_percent(100), Err(Error::InvalidParam));
    }

    #[test]
    fn test_ranging_frequency_configuration() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert_eq!(sensor.get_ranging_frequency_hz(), Ok(15));
        assert!(sensor.set_ranging_frequency_hz(30).is_ok());
        assert_eq!(sensor.get_ranging_frequency_hz(), Ok(30));

        assert_eq!(sensor.set_ranging_frequency_hz(0), Err(Error::InvalidParam));
        assert_eq!(
            sensor.set_ranging_frequency_hz(61),
            Err(Error::InvalidParam)
        );
    }

    #[test]
    fn test_resolution_and_mode_configuration() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert!(sensor.set_ranging_mode_continuous().is_ok());
        assert!(sensor.set_resolution_4x4().is_ok());
        #[cfg(feature = "firmware-blob")]
        assert!(sensor.set_resolution_8x8().is_ok());
    }

    #[test]
    fn test_start_and_stop_ranging_lifecycle() {
        let bus = MockBus::new();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert_eq!(sensor.get_ranging_data(), Err(Error::NotRanging));
        assert!(sensor.start_ranging().is_ok());
        assert!(sensor.stop_ranging().is_ok());
    }

    #[test]
    fn test_data_ready_and_frame_retrieval() {
        let mut bus = MockBus::new();
        bus.data_0000 = synthetic_4x4_frame();
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);

        assert!(sensor.start_ranging().is_ok());

        // Streamcount 1, Ready
        sensor.bus.data_0000[0] = 1;
        sensor.bus.data_0000[1] = 0x05;
        sensor.bus.data_0000[2] = 0x05;
        sensor.bus.data_0000[3] = 0x10;
        assert_eq!(sensor.check_data_ready(), Ok(true));

        // Same streamcount -> Not ready
        assert_eq!(sensor.check_data_ready(), Ok(false));

        // Get 4x4 ranging data
        sensor.bus.data_0000 = synthetic_4x4_frame();
        let frame = sensor.get_ranging_data().unwrap_or_default();
        assert_eq!(frame.silicon_temp_degc, 28);
        assert_eq!(frame.distance_mm[0], 120);
        assert_eq!(frame.target_status[0], 5);

        // 8x8 mode test
        #[cfg(feature = "firmware-blob")]
        {
            assert!(sensor.set_resolution_8x8().is_ok());
            assert!(sensor.start_ranging().is_ok());
            sensor.bus.data_0000 = synthetic_8x8_frame();
            let frame8 = sensor.get_ranging_data_8x8().unwrap_or_default();
            assert_eq!(frame8.silicon_temp_degc, 32);
            assert_eq!(frame8.distance_mm[0], 220);
            assert_eq!(frame8.target_status[0], 5);
        }

        // Abnormal status code
        sensor.bus.data_0000 = vec![0xFF, 0xFF, 0xFF, 0xFF];
        assert_eq!(sensor.check_data_ready(), Err(Error::Go2Status(0xFF)));
    }

    #[test]
    fn test_into_inner() {
        let bus = MockBus::new();
        let sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);
        let (_b, _d) = sensor.into_inner();
    }

    #[test]
    fn test_poll_for_answer_mcu_error() {
        let mut bus = MockBus::new();
        // Byte 2 >= 0x7F triggers McuError
        bus.status_0006 = [0x00, 0x00, 0x85, 0x00];
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);
        assert_eq!(
            sensor.set_target_order(TargetOrder::Strongest),
            Err(Error::McuError(0x85))
        );
    }

    #[test]
    fn test_error_variants() {
        let err: Error<u8> = Error::NotAlive {
            device_id: 0x12,
            revision_id: 0x34,
        };
        assert_eq!(
            err,
            Error::NotAlive {
                device_id: 0x12,
                revision_id: 0x34
            }
        );
        let timeout: Error<u8> = Error::Timeout {
            register: 0x06,
            last: 0xFF,
        };
        assert_eq!(
            timeout,
            Error::Timeout {
                register: 0x06,
                last: 0xFF
            }
        );
    }

    #[test]
    fn test_target_order_values() {
        assert_eq!(TargetOrder::Closest as u8, 1);
        assert_eq!(TargetOrder::Strongest as u8, 2);
        assert_eq!(DCI_TARGET_ORDER, 0xAE64);
    }

    #[test]
    fn test_sharpener_dci_and_scaling() {
        assert_eq!(DCI_SHARPENER, 0xAED8);
        // 0% -> 0
        #[allow(clippy::erasing_op)]
        let s0 = ((0u16 * 255) / 100) as u8;
        assert_eq!(s0, 0);
        // 35% -> 89 (89 * 100 / 255 = 34%)
        let s35 = ((35u16 * 255) / 100) as u8;
        assert_eq!(s35, 89);
        let roundtrip = (s35 as u16 * 100 / 255) as u8;
        assert_eq!(roundtrip, 34); // integer division truncation
                                   // 99% -> 252
        let s99 = ((99u16 * 255) / 100) as u8;
        assert_eq!(s99, 252);
    }
}
