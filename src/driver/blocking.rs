use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, Operation};

use crate::codec::{self, Frame4x4, Frame8x8};
use crate::{
    is_vl53l5cx_id, Error, TargetOrder, DCI_CMD_ADDR, DCI_DSS_CONFIG, DCI_FREQ_HZ,
    DCI_OUTPUT_CONFIG, DCI_OUTPUT_ENABLES, DCI_OUTPUT_LIST, DCI_RANGING_MODE, DCI_SHARPENER,
    DCI_SINGLE_RANGE, DCI_TARGET_ORDER, DCI_UI_RANGE_DATA, DCI_ZONE_CONFIG, MAX_FREQ_HZ_4X4,
    POLL_ATTEMPTS, POLL_INTERVAL_MS, TEMP_SIZE, UI_CMD_END, UI_CMD_START, UI_CMD_STATUS,
};

#[cfg(feature = "firmware-blob")]
use crate::{DCI_GLARE_FILTER, DCI_PIPE_CONTROL, MCU_BOOT_ATTEMPTS};

#[cfg(feature = "firmware-blob")]
use super::{
    DEFAULT_CONFIGURATION, DEFAULT_XTALK, FW_PAGE_09, FW_PAGE_0A, FW_PAGE_0B, GET_NVM_CMD,
};

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
    pub(crate) bus: BUS,
    delay: DELAY,
    addr: u8,
    temp: [u8; TEMP_SIZE],
    #[allow(dead_code)]
    offset_data: [u8; codec::OFFSET_BUFFER_SIZE],
    pub(crate) streamcount: u8,
    pub(crate) data_read_size: usize,
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

        // HARDWARE QUIRK: ST VL53L5CX Soft-Reset NACK Tolerance
        // During soft-reset, internal LDOs and the oscillator reboot.
        // Per ST reference driver (vl53l5cx_init in ULD v1.3), I2C write errors/NACKs
        // on these initial boot writes are deliberately ignored until register 0x06
        // is polled. Propagating errors here causes spurious boot failures on physical MCUs.
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
        self.streamcount = 255;
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
