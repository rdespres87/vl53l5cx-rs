use embassy_time::Timer;
use embedded_hal_async::i2c::I2c as AsyncI2c;

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

/// Writes multi-byte payloads using contiguous `[reg_hi, reg_lo, chunk...]` frames
/// chunked to 256 bytes, adhering to the ST ULD `VL53L5CX_WrMulti` specification.
/// Contiguous frames guarantee standard I2C bus compatibility across all MCU architectures.
pub(crate) async fn write_multi_async<B: AsyncI2c>(
    bus: &mut B,
    addr: u8,
    mut reg: u16,
    mut data: &[u8],
) -> Result<(), B::Error> {
    const CHUNK_SIZE: usize = 256;
    let mut chunk_buf = [0u8; 2 + CHUNK_SIZE];
    while !data.is_empty() {
        let chunk_len = data.len().min(CHUNK_SIZE);
        chunk_buf[0..2].copy_from_slice(&reg.to_be_bytes());
        chunk_buf[2..2 + chunk_len].copy_from_slice(&data[..chunk_len]);

        let mut last_err = None;
        for attempt in 0..5 {
            match bus.write(addr, &chunk_buf[..2 + chunk_len]).await {
                Ok(()) => {
                    last_err = None;
                    break;
                }
                Err(e) => {
                    last_err = Some(e);
                    Timer::after_millis(10 * (attempt + 1) as u64).await;
                }
            }
        }
        if let Some(err) = last_err {
            return Err(err);
        }
        reg = reg.wrapping_add(chunk_len as u16);
        data = &data[chunk_len..];
    }
    Ok(())
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

    write_multi_async(bus, addr, 0x0000, data)
        .await
        .map_err(Error::Bus)?;

    Timer::after_millis(50).await;
    Ok(())
}

/// Asynchronous VL53L5CX driver instance.
pub struct Vl53l5cx<BUS> {
    pub(crate) bus: BUS,
    addr: u8,
    temp: [u8; TEMP_SIZE],
    #[allow(dead_code)]
    offset_data: [u8; codec::OFFSET_BUFFER_SIZE],
    pub(crate) streamcount: u8,
    pub(crate) data_read_size: usize,
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
        let (device_id, revision_id) = probe(&mut self.bus, self.addr).await.map_err(Error::Bus)?;
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

        // HARDWARE QUIRK: ST VL53L5CX Soft-Reset NACK Tolerance
        // During soft-reset, internal LDOs and the oscillator reboot.
        // Per ST reference driver (vl53l5cx_init in ULD v1.3), I2C write errors/NACKs
        // on these initial boot writes are deliberately ignored until register 0x06
        // is polled. Propagating errors here causes spurious boot failures on physical MCUs.
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
    pub async fn set_target_order(&mut self, order: TargetOrder) -> Result<(), Error<BUS::Error>> {
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
    pub async fn set_sharpener_percent(&mut self, percent: u8) -> Result<(), Error<BUS::Error>> {
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
        self.streamcount = 255;
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
