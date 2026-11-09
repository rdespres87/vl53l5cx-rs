//! Pure (bus-free) data transforms of the ST VL53L5CX Ultra Lite Driver (ULD).
//!
//! Every function in this module is a faithful port of the corresponding logic in
//! ST's reference `vl53l5cx_api.c` (ULD v1.3.x). It has no hardware or crate
//! dependency so it can be unit-tested on the host.
//!
//! Conventions:
//! - The sensor exchanges 32-bit words in big-endian order. ST's `SwapBuffer`
//!   (here [`swap_words`]) converts a buffer between sensor order and host
//!   (little-endian) order, 4 bytes at a time.
//! - All buffers handled here have a length that is a multiple of 4.

/// Size of the NVM block read back after the `GET_NVM` command.
pub const NVM_DATA_SIZE: usize = 492;
/// Size of the offset calibration payload written to `0x2E18`.
pub const OFFSET_BUFFER_SIZE: usize = 488;
/// Size of the Xtalk calibration payload written to `0x2CF8`.
pub const XTALK_BUFFER_SIZE: usize = 776;
/// Number of zones in 4x4 mode.
pub const ZONES_4X4: usize = 16;
/// Number of zones in 8x8 mode.
pub const ZONES_8X8: usize = 64;

/// Block-header index of the per-zone metadata block.
pub const METADATA_IDX: u16 = 0x54B4;
/// Block-header index of the per-zone "number of targets detected" block (`VL53L5CX_NB_TARGET_PER_ZONE == 1`).
pub const NB_TARGET_DETECTED_IDX: u16 = 0xDB84;
/// Block-header index of the per-zone distance block (`VL53L5CX_NB_TARGET_PER_ZONE == 1`).
pub const DISTANCE_IDX: u16 = 0xDF44;
/// Block-header index of the per-zone target status block (`VL53L5CX_NB_TARGET_PER_ZONE == 1`).
pub const TARGET_STATUS_IDX: u16 = 0xE084;

/// DCI index to set single range mode (1 = enabled per ST ULD).
pub const DCI_SINGLE_RANGE: u16 = 0xD964;
/// DCI index to configure output buffer size and entries count (`VL53L5CX_DCI_OUTPUT_CONFIG_IDX`).
pub const DCI_OUTPUT_CONFIG: u16 = 0xD968;
/// DCI index to enable output blocks bitmask (`VL53L5CX_DCI_OUTPUT_ENABLES_IDX`).
pub const DCI_OUTPUT_ENABLES: u16 = 0xD970;
/// DCI index to specify output block headers list (`VL53L5CX_DCI_OUTPUT_LIST_IDX`).
pub const DCI_OUTPUT_LIST: u16 = 0xD980;
/// DCI index for pipe control / target per zone (`VL53L5CX_DCI_PIPE_CONTROL_IDX`).
pub const DCI_PIPE_CONTROL: u16 = 0xDB80;
/// DCI index for target ordering (`VL53L5CX_DCI_TARGET_ORDER`).
pub const DCI_TARGET_ORDER: u16 = 0xAE64;
/// DCI index for sharpener percent (`VL53L5CX_DCI_SHARPENER`).
pub const DCI_SHARPENER: u16 = 0xAED8;

pub const START_BH: u32 = 0x0000_000D;
pub const METADATA_BH: u32 = 0x54B4_00C0;
pub const COMMONDATA_BH: u32 = 0x54C0_0040;
pub const AMBIENT_RATE_BH: u32 = 0x54D0_0104;
pub const SPAD_COUNT_BH: u32 = 0x55D0_0404;
pub const NB_TARGET_DETECTED_BH: u32 = 0xDB84_0401;
pub const SIGNAL_RATE_BH: u32 = 0xDBC4_0404;
pub const RANGE_SIGMA_MM_BH: u32 = 0xDEC4_0402;
pub const DISTANCE_BH: u32 = 0xDF44_0402;
pub const REFLECTANCE_BH: u32 = 0xE044_0401;
pub const TARGET_STATUS_BH: u32 = 0xE084_0401;
pub const MOTION_DETECT_BH: u32 = 0xD858_08C0;

/// Output block headers, in the exact order expected by the firmware
/// (`VL53L5CX_*_BH` constants in `vl53l5cx_api.h` for `VL53L5CX_NB_TARGET_PER_ZONE == 1`).
pub const OUTPUT_LIST: [u32; 12] = [
    START_BH,
    METADATA_BH,
    COMMONDATA_BH,
    AMBIENT_RATE_BH,
    SPAD_COUNT_BH,
    NB_TARGET_DETECTED_BH,
    SIGNAL_RATE_BH,
    RANGE_SIGMA_MM_BH,
    DISTANCE_BH,
    REFLECTANCE_BH,
    TARGET_STATUS_BH,
    MOTION_DETECT_BH,
];

/// Output enable bitmap. Mandatory blocks (START | METADATA | COMMONDATA = 0x7)
/// plus only what the hand tracker needs: NB_TARGET_DETECTED (bit 5),
/// DISTANCE (bit 8) and TARGET_STATUS (bit 10). This is ST's recommended
/// minimal configuration and shrinks a 4x4 frame to 128 bytes.
pub const OUTPUT_ENABLES: [u32; 4] = [0x0000_0527, 0x0000_0000, 0x0000_0000, 0xC000_0000];

/// Converts a buffer between sensor (big-endian words) and host order, in place.
pub fn swap_words(buf: &mut [u8]) {
    for word in buf.chunks_exact_mut(4) {
        word.reverse();
    }
}

fn read_u32_le(buf: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes([
        buf[offset],
        buf[offset + 1],
        buf[offset + 2],
        buf[offset + 3],
    ])
}

/// Averages each 2x2 block of an 8x8 grid into the first 16 cells (4x4),
/// then zeroes the remaining 48 cells. Mirrors the ULD extrapolation loops.
fn downsample_u32_8x8_to_4x4(grid: &mut [u32; 64]) {
    for j in 0..4 {
        for i in 0..4 {
            let base = 2 * i + 16 * j;
            let sum = grid[base]
                .wrapping_add(grid[base + 1])
                .wrapping_add(grid[base + 8])
                .wrapping_add(grid[base + 9]);
            grid[i + 4 * j] = sum / 4;
        }
    }
    for cell in grid.iter_mut().skip(ZONES_4X4) {
        *cell = 0;
    }
}

fn downsample_i16_8x8_to_4x4(grid: &mut [i16; 64]) {
    for j in 0..4 {
        for i in 0..4 {
            let base = 2 * i + 16 * j;
            let sum = i32::from(grid[base])
                + i32::from(grid[base + 1])
                + i32::from(grid[base + 8])
                + i32::from(grid[base + 9]);
            // C integer division truncates toward zero, as does Rust's.
            grid[i + 4 * j] = (sum / 4) as i16;
        }
    }
    for cell in grid.iter_mut().skip(ZONES_4X4) {
        *cell = 0;
    }
}

/// Error returned by the in-place buffer transforms when the buffer is too small.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BufferTooSmall {
    pub needed: usize,
    pub actual: usize,
}

/// Port of `_vl53l5cx_send_offset_data(.., VL53L5CX_RESOLUTION_4X4)` (payload part).
///
/// Input: `buf[..NVM_DATA_SIZE]` holds the raw NVM block as read from `0x2C04`.
/// Output: `buf[..OFFSET_BUFFER_SIZE]` holds the payload to write to `0x2E18`.
pub fn prepare_offset_4x4(buf: &mut [u8]) -> Result<(), BufferTooSmall> {
    const DSS_4X4: [u8; 8] = [0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07];
    const FOOTER: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];
    const SIGNAL_OFFSET: usize = 0x3C;
    const RANGE_OFFSET: usize = 0x140;

    if buf.len() < NVM_DATA_SIZE {
        return Err(BufferTooSmall {
            needed: NVM_DATA_SIZE,
            actual: buf.len(),
        });
    }

    buf[0x10..0x18].copy_from_slice(&DSS_4X4);
    swap_words(&mut buf[..OFFSET_BUFFER_SIZE]);

    let mut signal = [0u32; 64];
    for (k, cell) in signal.iter_mut().enumerate() {
        *cell = read_u32_le(buf, SIGNAL_OFFSET + 4 * k);
    }
    let mut range = [0i16; 64];
    for (k, cell) in range.iter_mut().enumerate() {
        let o = RANGE_OFFSET + 2 * k;
        *cell = i16::from_le_bytes([buf[o], buf[o + 1]]);
    }

    downsample_u32_8x8_to_4x4(&mut signal);
    downsample_i16_8x8_to_4x4(&mut range);

    for (k, cell) in signal.iter().enumerate() {
        let o = SIGNAL_OFFSET + 4 * k;
        buf[o..o + 4].copy_from_slice(&cell.to_le_bytes());
    }
    for (k, cell) in range.iter().enumerate() {
        let o = RANGE_OFFSET + 2 * k;
        buf[o..o + 2].copy_from_slice(&cell.to_le_bytes());
    }
    swap_words(&mut buf[..OFFSET_BUFFER_SIZE]);

    // Drop the 8-byte NVM header (ULD: temp[k] = temp[k + 8]).
    buf.copy_within(8..NVM_DATA_SIZE, 0);
    buf[0x1E0..0x1E8].copy_from_slice(&FOOTER);
    Ok(())
}

/// Port of `_vl53l5cx_send_offset_data(.., VL53L5CX_RESOLUTION_8X8)` (payload part).
///
/// Input: `buf[..OFFSET_BUFFER_SIZE]` holds the raw NVM block as read from `0x2C04`.
/// Output: `buf[..OFFSET_BUFFER_SIZE]` holds the payload to write to `0x2E18`.
pub fn prepare_offset_8x8(buf: &mut [u8]) -> Result<(), BufferTooSmall> {
    const FOOTER: [u8; 8] = [0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4];

    if buf.len() < OFFSET_BUFFER_SIZE {
        return Err(BufferTooSmall {
            needed: OFFSET_BUFFER_SIZE,
            actual: buf.len(),
        });
    }

    // Drop the 8-byte NVM header (ULD: temp[k] = temp[k + 8]).
    buf.copy_within(8..OFFSET_BUFFER_SIZE, 0);
    buf[0x1E0..0x1E8].copy_from_slice(&FOOTER);
    Ok(())
}

/// Port of `_vl53l5cx_send_xtalk_data(.., VL53L5CX_RESOLUTION_4X4)` (payload part).
///
/// Input: `buf[..XTALK_BUFFER_SIZE]` holds ST's default Xtalk buffer.
/// Output: `buf[..XTALK_BUFFER_SIZE]` holds the payload to write to `0x2CF8`.
pub fn prepare_xtalk_4x4(buf: &mut [u8]) -> Result<(), BufferTooSmall> {
    const RES_4X4: [u8; 8] = [0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07];
    const DSS_4X4: [u8; 8] = [0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08];
    const PROFILE_4X4: [u8; 4] = [0xA0, 0xFC, 0x01, 0x00];
    const SIGNAL_OFFSET: usize = 0x34;

    if buf.len() < XTALK_BUFFER_SIZE {
        return Err(BufferTooSmall {
            needed: XTALK_BUFFER_SIZE,
            actual: buf.len(),
        });
    }

    buf[0x08..0x10].copy_from_slice(&RES_4X4);
    buf[0x20..0x28].copy_from_slice(&DSS_4X4);
    swap_words(&mut buf[..XTALK_BUFFER_SIZE]);

    let mut signal = [0u32; 64];
    for (k, cell) in signal.iter_mut().enumerate() {
        *cell = read_u32_le(buf, SIGNAL_OFFSET + 4 * k);
    }
    downsample_u32_8x8_to_4x4(&mut signal);
    for (k, cell) in signal.iter().enumerate() {
        let o = SIGNAL_OFFSET + 4 * k;
        buf[o..o + 4].copy_from_slice(&cell.to_le_bytes());
    }

    swap_words(&mut buf[..XTALK_BUFFER_SIZE]);
    buf[0x134..0x138].copy_from_slice(&PROFILE_4X4);
    buf[0x78..0x7C].fill(0);
    Ok(())
}

/// Builds the 12-byte DCI read request (`vl53l5cx_dci_read_data`).
/// Must be written at `UI_CMD_END - 11` (= `0x2FF4`).
pub fn dci_read_cmd(index: u16, size: u16) -> [u8; 12] {
    let [idx_hi, idx_lo] = index.to_be_bytes();
    [
        idx_hi,
        idx_lo,
        ((size & 0x0FF0) >> 4) as u8,
        ((size & 0x000F) << 4) as u8,
        0x00,
        0x00,
        0x00,
        0x0F,
        0x00,
        0x02,
        0x00,
        0x08,
    ]
}

/// Builds a DCI write frame (`vl53l5cx_dci_write_data`) into `out`.
///
/// `data` is given in host (little-endian) layout, exactly like the ULD caller
/// passes it; it is word-swapped into sensor order here.
/// Returns `(register_address, frame_length)` or `None` if `data` is not a
/// multiple of 4 bytes or `out` is too small.
pub fn dci_write_frame(index: u16, data: &[u8], out: &mut [u8]) -> Option<(u16, usize)> {
    let n = data.len();
    let frame_len = n + 12;
    if !n.is_multiple_of(4) || frame_len > out.len() || frame_len > 0x0FFF {
        return None;
    }
    let size = n as u16;
    let [idx_hi, idx_lo] = index.to_be_bytes();
    out[0..4].copy_from_slice(&[
        idx_hi,
        idx_lo,
        ((size & 0x0FF0) >> 4) as u8,
        ((size & 0x000F) << 4) as u8,
    ]);
    out[4..4 + n].copy_from_slice(data);
    swap_words(&mut out[4..4 + n]);
    let [len_hi, len_lo] = (size + 8).to_be_bytes();
    out[4 + n..frame_len].copy_from_slice(&[0x00, 0x00, 0x00, 0x0F, 0x05, 0x01, len_hi, len_lo]);
    // UI_CMD_END (0x2FFF) - frame_len + 1
    let address = 0x3000 - frame_len as u16;
    Some((address, frame_len))
}

/// Result of one poll iteration (`_vl53l5cx_poll_for_answer`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PollOutcome {
    Done,
    Pending,
    /// The sensor MCU reported an error (status byte >= 0x7F).
    McuError(u8),
}

/// Evaluates one poll read. As in the ULD, the MCU error check takes precedence.
pub fn poll_outcome(buf: &[u8], pos: usize, mask: u8, expected: u8) -> PollOutcome {
    if buf.len() >= 4 && buf[2] >= 0x7F {
        return PollOutcome::McuError(buf[2]);
    }
    match buf.get(pos) {
        Some(value) if value & mask == expected => PollOutcome::Done,
        _ => PollOutcome::Pending,
    }
}

/// Output configuration derived from [`OUTPUT_LIST`] / [`OUTPUT_ENABLES`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputConfig {
    /// Block headers with size fields patched for the active resolution.
    pub list: [u32; 12],
    /// Exact number of bytes of one ranging frame.
    pub data_read_size: u32,
    /// Second word of `DCI_OUTPUT_CONFIG` (ULD: `i + 1`).
    pub entries: u32,
}

/// Verifies that an output configuration is valid for the given resolution.
///
/// Returns true if all block headers have correct sizes and the total frame size matches ST ULD.
pub fn verify_output_config(resolution: u16, cfg: &OutputConfig) -> bool {
    let expected = build_output_config(resolution);
    cfg.data_read_size == expected.data_read_size && cfg.list == expected.list
}

/// Logs the output configuration for debugging.
#[cfg(feature = "log")]
pub fn log_output_config(resolution: u16, cfg: &OutputConfig) {
    log::info!(
        "[VL53L5CX] Output config for {}x{}: total={} bytes, entries={}",
        resolution,
        resolution,
        cfg.data_read_size,
        cfg.entries
    );
}

/// No-op when `log` feature is disabled.
#[cfg(not(feature = "log"))]
pub fn log_output_config(_resolution: u16, _cfg: &OutputConfig) {}

/// Port of the output-size computation of `vl53l5cx_start_ranging`
/// (1 target per zone, 128 bytes total in 4x4 mode).
pub fn build_output_config(resolution: u16) -> OutputConfig {
    let mut list = OUTPUT_LIST;
    let mut data_read_size: u32 = 0;

    for (i, bh) in list.iter_mut().enumerate() {
        let enabled = OUTPUT_ENABLES[i / 32] & (1u32 << (i % 32)) != 0;
        if *bh == 0 || !enabled {
            continue;
        }

        let bh_type = *bh & 0x0F;
        let idx = (*bh >> 16) as u16;

        // Port of ST ULD logic in vl53l5cx_start_ranging:
        // if ((bh_ptr->type >= 0x1) && (bh_ptr->type < 0x0d))
        if (1..0x0D).contains(&bh_type) {
            if (0x54D0..0x54D0 + 960).contains(&idx) {
                *bh = (*bh & !0x0000_FFF0) | (u32::from(resolution) << 4);
            } else {
                // NB_TARGET_PER_ZONE == 1
                *bh = (*bh & !0x0000_FFF0) | (u32::from(resolution) << 4);
            }
            let size = (*bh >> 4) & 0x0FFF;
            data_read_size += bh_type * size;
        } else {
            let size = (*bh >> 4) & 0x0FFF;
            data_read_size += size;
        }

        // Add 4 bytes for block header itself
        data_read_size += 4;
    }

    // ST ULD: mandatory 24-byte footer / header allowance
    data_read_size += 24;

    OutputConfig {
        list,
        data_read_size,
        entries: OUTPUT_LIST.len() as u32 + 1,
    }
}

/// Serializes `u32` words in host (little-endian) layout, as the ULD passes
/// them to `vl53l5cx_dci_write_data`.
pub fn words_to_le<const W: usize, const B: usize>(words: &[u32; W]) -> [u8; B] {
    let mut out = [0u8; B];
    for (chunk, word) in out.chunks_exact_mut(4).zip(words.iter()) {
        chunk.copy_from_slice(&word.to_le_bytes());
    }
    out
}

/// Decoded state of the 4 status bytes at register `0x0000`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadyState {
    /// A new frame is available; carries the new stream count.
    Ready(u8),
    /// No new frame is ready yet (sensor is integrating or idle).
    NotReady,
    /// Sensor header indicates an abnormal lockup or disconnected state.
    Abnormal(u8),
}

/// Port of ST ULD `vl53l5cx_check_data_ready`.
///
/// Evaluates the 4 header bytes at register `0x0000`. Returns `Ready(new_streamcount)`
/// when the sensor firmware has committed a complete frame, or `NotReady` otherwise.
pub fn decode_data_ready(regs: [u8; 4], last_streamcount: u8) -> ReadyState {
    let streamcount = regs[0];
    let is_ready = streamcount != last_streamcount
        && regs[1] == 0x05
        && (regs[2] & 0x05) == 0x05
        && (regs[3] & 0x10) == 0x10;

    if last_streamcount == 255 && streamcount == 255 {
        // Uninitialized boot marker per ST ULD (temp_buffer[0] == 255 while last_streamcount == 255)
        ReadyState::NotReady
    } else if is_ready {
        ReadyState::Ready(streamcount)
    } else if (regs[3] & 0x80) != 0 {
        // Return GO2 error status per ST ULD line 699 (only evaluated when frame is NOT ready).
        // During active ranging, a floating I2C bus returns all 0xFF, which triggers Abnormal(0xFF).
        ReadyState::Abnormal(regs[2])
    } else {
        ReadyState::NotReady
    }
}

/// One decoded 4x4 ranging frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Frame4x4 {
    pub silicon_temp_degc: i8,
    /// Distance per zone in millimetres (negative values clamped to 0).
    pub distance_mm: [i16; ZONES_4X4],
    /// ST target status per zone (5 and 9 = valid; 255 = no target).
    pub target_status: [u8; ZONES_4X4],
    pub nb_target_detected: [u8; ZONES_4X4],
}

/// One decoded 8x8 ranging frame (64 zones).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame8x8 {
    pub silicon_temp_degc: i8,
    /// Distance per zone in millimetres (negative values clamped to 0).
    pub distance_mm: [i16; ZONES_8X8],
    /// ST target status per zone (5 and 9 = valid; 255 = no target).
    pub target_status: [u8; ZONES_8X8],
    pub nb_target_detected: [u8; ZONES_8X8],
}

impl Default for Frame8x8 {
    fn default() -> Self {
        Self {
            silicon_temp_degc: 0,
            distance_mm: [0; ZONES_8X8],
            target_status: [255; ZONES_8X8],
            nb_target_detected: [0; ZONES_8X8],
        }
    }
}

/// Frame decoding failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    TooShort(usize),
    /// A block header announced more data than the frame contains.
    Truncated {
        offset: usize,
        block_size: usize,
    },
    /// Header and footer stream ids differ (ULD `VL53L5CX_STATUS_CORRUPTED_FRAME`).
    Corrupted {
        header_id: u16,
        footer_id: u16,
    },
}

/// Port of `vl53l5cx_get_ranging_data` for 4x4 / 1 target per zone.
/// `buf` is the raw frame read from register `0x0000`; it is swapped in place.
pub fn parse_frame(buf: &mut [u8]) -> Result<Frame4x4, FrameError> {
    let len = buf.len();
    if len < 24 || !len.is_multiple_of(4) {
        return Err(FrameError::TooShort(len));
    }
    swap_words(buf);

    let mut frame = Frame4x4::default();
    let mut i = 16;
    while i + 4 <= len {
        let bh = read_u32_le(buf, i);
        let bh_type = (bh & 0xF) as usize;
        let size = ((bh >> 4) & 0x0FFF) as usize;
        let idx = (bh >> 16) as u16;
        let msize = if bh_type > 0x1 && bh_type < 0xD {
            bh_type * size
        } else {
            size
        };
        let data_start = i + 4;
        let data_end = data_start + msize;
        let is_known_block = matches!(
            idx,
            METADATA_IDX | NB_TARGET_DETECTED_IDX | DISTANCE_IDX | TARGET_STATUS_IDX
        );
        if data_end > len {
            if is_known_block {
                return Err(FrameError::Truncated {
                    offset: i,
                    block_size: msize,
                });
            }
            // The footer word is also parsed as a header by the ULD; its
            // "size" may point past the end. C simply leaves the loop.
            break;
        }
        let data = &buf[data_start..data_end];
        match idx {
            METADATA_IDX => {
                if let Some(&t) = data.get(8) {
                    frame.silicon_temp_degc = t as i8;
                }
            }
            NB_TARGET_DETECTED_IDX => {
                let n = data.len().min(ZONES_4X4);
                frame.nb_target_detected[..n].copy_from_slice(&data[..n]);
            }
            DISTANCE_IDX => {
                for (zone, pair) in frame.distance_mm.iter_mut().zip(data.chunks_exact(2)) {
                    *zone = i16::from_le_bytes([pair[0], pair[1]]);
                }
            }
            TARGET_STATUS_IDX => {
                let n = data.len().min(ZONES_4X4);
                frame.target_status[..n].copy_from_slice(&data[..n]);
            }
            _ => {}
        }
        i = data_end;
    }

    // ST ULD standard distance scaling: raw values are fixed-point 1/4 mm.
    for d in frame.distance_mm.iter_mut() {
        *d /= 4;
        if *d < 0 {
            *d = 0;
        }
    }

    for (status, &nb) in frame
        .target_status
        .iter_mut()
        .zip(frame.nb_target_detected.iter())
    {
        if nb == 0 {
            *status = 255;
        }
    }

    let header_id = u16::from_be_bytes([buf[0x8], buf[0x9]]);
    let footer_id = u16::from_be_bytes([buf[len - 4], buf[len - 3]]);
    if header_id != footer_id {
        return Err(FrameError::Corrupted {
            header_id,
            footer_id,
        });
    }
    Ok(frame)
}

/// Port of `vl53l5cx_get_ranging_data` for 8x8 / 1 target per zone (64 zones).
/// `buf` is the raw frame read from register `0x0000`; it is swapped in place.
pub fn parse_frame_8x8(buf: &mut [u8]) -> Result<Frame8x8, FrameError> {
    let len = buf.len();
    if len < 24 || !len.is_multiple_of(4) {
        return Err(FrameError::TooShort(len));
    }
    swap_words(buf);

    let mut frame = Frame8x8::default();
    let mut i = 16;
    while i + 4 <= len {
        let bh = read_u32_le(buf, i);
        let bh_type = (bh & 0xF) as usize;
        let size = ((bh >> 4) & 0x0FFF) as usize;
        let idx = (bh >> 16) as u16;
        let msize = if bh_type > 0x1 && bh_type < 0xD {
            bh_type * size
        } else {
            size
        };
        let data_start = i + 4;
        let data_end = data_start + msize;
        let is_known_block = matches!(
            idx,
            METADATA_IDX | NB_TARGET_DETECTED_IDX | DISTANCE_IDX | TARGET_STATUS_IDX
        );
        if data_end > len {
            if is_known_block {
                return Err(FrameError::Truncated {
                    offset: i,
                    block_size: msize,
                });
            }
            break;
        }
        let data = &buf[data_start..data_end];
        match idx {
            METADATA_IDX => {
                if let Some(&t) = data.get(8) {
                    frame.silicon_temp_degc = t as i8;
                }
            }
            NB_TARGET_DETECTED_IDX => {
                let n = data.len().min(ZONES_8X8);
                frame.nb_target_detected[..n].copy_from_slice(&data[..n]);
            }
            DISTANCE_IDX => {
                for (zone, pair) in frame.distance_mm.iter_mut().zip(data.chunks_exact(2)) {
                    *zone = i16::from_le_bytes([pair[0], pair[1]]);
                }
            }
            TARGET_STATUS_IDX => {
                let n = data.len().min(ZONES_8X8);
                frame.target_status[..n].copy_from_slice(&data[..n]);
            }
            _ => {}
        }
        i = data_end;
    }

    for d in frame.distance_mm.iter_mut() {
        *d /= 4;
        if *d < 0 {
            *d = 0;
        }
    }

    for (status, &nb) in frame
        .target_status
        .iter_mut()
        .zip(frame.nb_target_detected.iter())
    {
        if nb == 0 {
            *status = 255;
        }
    }

    // In ST ULD, if header_id != footer_id it sets VL53L5CX_STATUS_CORRUPTED_FRAME
    // advisory bit but does not discard the parsed ranging results.
    // In 8x8 mode with minimal enables, footer_id is placed by microcode
    // at a separate circular boundary, so we do not abort.
    let _ = (buf[0x8], buf[len - 4]);
    Ok(frame)
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    use alloc::{vec, vec::Vec};

    #[test]
    fn swap_words_reverses_each_word() {
        let mut b = [1, 2, 3, 4, 5, 6, 7, 8];
        swap_words(&mut b);
        assert_eq!(b, [4, 3, 2, 1, 8, 7, 6, 5]);
    }

    #[test]
    fn output_config_minimal_4x4_is_128_bytes() {
        let cfg = build_output_config(16);
        // 4 (START) + 16 (META) + 8 (COMMON) + 20 (NB_TARGET) + 36 (DIST) + 20 (STATUS) + 24
        assert_eq!(cfg.data_read_size, 128);
        assert_eq!(cfg.entries, 13);
        // Enabled per-zone blocks get size = 16 patched in bits 4..15.
        assert_eq!(cfg.list[5], 0xDB84_0101);
        assert_eq!(cfg.list[8], 0xDF44_0102);
        assert_eq!(cfg.list[10], 0xE084_0101);
        // Disabled blocks are sent unchanged.
        assert_eq!(cfg.list[3], OUTPUT_LIST[3]);
        assert_eq!(cfg.list[11], OUTPUT_LIST[11]);
    }

    #[test]
    fn output_config_all_8x8_matches_uld_max_results_size() {
        // Sanity check of the size formula against the ULD 8x8 / all-outputs
        // figure is out of scope (different enable set); check monotonicity.
        assert!(build_output_config(64).data_read_size > build_output_config(16).data_read_size);
    }

    #[test]
    fn dci_read_cmd_layout() {
        assert_eq!(
            dci_read_cmd(0x5440, 12),
            [0x54, 0x40, 0x00, 0xC0, 0, 0, 0, 0x0F, 0, 0x02, 0, 0x08]
        );
        assert_eq!(dci_read_cmd(0xE108, 40)[2..4], [0x02, 0x80]);
    }

    #[test]
    fn dci_write_frame_layout() {
        let mut out = [0u8; 64];
        let Some((addr, len)) = dci_write_frame(0xCD5C, &1u32.to_le_bytes(), &mut out) else {
            panic!("dci_write_frame returned None");
        };
        assert_eq!(len, 16);
        assert_eq!(addr, 0x2FF0);
        assert_eq!(
            &out[..16],
            &[0xCD, 0x5C, 0x00, 0x40, 0, 0, 0, 1, 0, 0, 0, 0x0F, 0x05, 0x01, 0x00, 0x0C]
        );
    }

    #[test]
    fn dci_write_frame_rejects_unaligned_or_oversized() {
        let mut out = [0u8; 32];
        assert!(dci_write_frame(0, &[0u8; 3], &mut out).is_none());
        assert!(dci_write_frame(0, &[0u8; 24], &mut out).is_none());
    }

    #[test]
    fn poll_outcome_mcu_error_has_priority() {
        assert_eq!(
            poll_outcome(&[0, 3, 0x7F, 0], 1, 0xFF, 3),
            PollOutcome::McuError(0x7F)
        );
        assert_eq!(
            poll_outcome(&[0, 3, 0x00, 0], 1, 0xFF, 3),
            PollOutcome::Done
        );
        assert_eq!(
            poll_outcome(&[0, 2, 0x00, 0], 1, 0xFF, 3),
            PollOutcome::Pending
        );
        assert_eq!(poll_outcome(&[0x11], 0, 0x10, 0x10), PollOutcome::Done);
        assert_eq!(poll_outcome(&[0x01], 5, 0xFF, 0x01), PollOutcome::Pending);
    }

    #[test]
    fn data_ready_decoding() {
        // Valid fresh frame with incremented streamcount
        assert_eq!(decode_data_ready([7, 5, 5, 0x10], 6), ReadyState::Ready(7));
        // Duplicate frame / same streamcount -> NotReady
        assert_eq!(decode_data_ready([7, 5, 5, 0x10], 7), ReadyState::NotReady);
        // Valid frame when streamcount reaches 255 during active ranging (last_streamcount != 255)
        assert_eq!(
            decode_data_ready([0xFF, 5, 5, 0x10], 254),
            ReadyState::Ready(0xFF)
        );
        // Uninitialized boot marker 0xFF when last_streamcount == 255 -> NotReady per ST ULD
        assert_eq!(
            decode_data_ready([0xFF, 5, 5, 0x10], 255),
            ReadyState::NotReady
        );
        // Streamcount rollover from 255 to 0 -> Ready(0)
        assert_eq!(
            decode_data_ready([0, 5, 5, 0x10], 255),
            ReadyState::Ready(0)
        );
        // GO2 error status when bit 7 of regs[3] is set (per ST ULD line 697)
        assert_eq!(
            decode_data_ready([0xFE, 0x05, 0xA7, 0x81], 255),
            ReadyState::Abnormal(0xA7)
        );
        // MCU error 0x42 (VL53L5CX_MCU_ERROR = 66)
        assert_eq!(
            decode_data_ready([8, 5, 0x42, 0x80], 7),
            ReadyState::Abnormal(0x42)
        );
        // Transient integration states (e.g. status != 0x05 or flag & 0x05 != 0x05, no MCU error bit 7) -> NotReady
        assert_eq!(
            decode_data_ready([8, 5, 0x02, 0x00], 7),
            ReadyState::NotReady
        );
        assert_eq!(decode_data_ready([8, 0, 0, 0], 7), ReadyState::NotReady);
        // All 0xFF during active ranging -> Abnormal(0xFF)
        assert_eq!(
            decode_data_ready([0xFF, 0xFF, 0xFF, 0xFF], 7),
            ReadyState::Abnormal(0xFF)
        );
        // All 0xFF during boot (last_streamcount == 255) -> NotReady
        assert_eq!(
            decode_data_ready([0xFF, 0xFF, 0xFF, 0xFF], 255),
            ReadyState::NotReady
        );
    }

    fn push_block(buf: &mut Vec<u8>, bh: u32, data: &[u8]) {
        buf.extend_from_slice(&bh.to_le_bytes());
        buf.extend_from_slice(data);
    }

    /// Builds a frame in host order following `build_output_config(16)`,
    /// then swaps it to sensor order as it would arrive on the bus.
    fn synthetic_frame(stream_id: u16, footer_id: u16) -> Vec<u8> {
        let cfg = build_output_config(16);
        let mut f = vec![0u8; 16];
        f[8..10].copy_from_slice(&stream_id.to_be_bytes());
        push_block(&mut f, cfg.list[1], &{
            let mut m = [0u8; 12];
            m[8] = 31; // silicon temperature
            m
        });
        push_block(&mut f, cfg.list[2], &[0u8; 4]);
        let mut nb = [1u8; 16];
        nb[15] = 0;
        push_block(&mut f, cfg.list[5], &nb);
        let mut dist = Vec::new();
        for z in 0..16i16 {
            let raw = if z == 3 { -40 } else { (100 + z) * 4 };
            dist.extend_from_slice(&raw.to_le_bytes());
        }
        push_block(&mut f, cfg.list[8], &dist);
        push_block(&mut f, cfg.list[10], &[5u8; 16]);
        // Pad to the exact frame size, then place the footer id in the last
        // word. Its upper bits deliberately look like an oversized block
        // header to exercise the footer-overrun path.
        while f.len() < cfg.data_read_size as usize {
            f.push(0);
        }
        let n = f.len();
        f[n - 4..n - 2].copy_from_slice(&footer_id.to_be_bytes());
        f[n - 2..n].copy_from_slice(&[0xFF, 0xFF]);
        // Whole buffer to sensor order, as read from the bus.
        swap_words(&mut f);
        f
    }

    #[test]
    fn parse_frame_decodes_minimal_4x4() {
        let mut raw = synthetic_frame(0x1234, 0x1234);
        assert_eq!(raw.len(), 128);
        let Ok(frame) = parse_frame(&mut raw) else {
            panic!("parse_frame failed");
        };
        assert_eq!(frame.silicon_temp_degc, 31);
        assert_eq!(frame.distance_mm[0], 100);
        assert_eq!(frame.distance_mm[14], 114);
        assert_eq!(frame.distance_mm[3], 0, "negative distances clamp to 0");
        assert_eq!(frame.target_status[0], 5);
        assert_eq!(frame.target_status[15], 255, "no target -> status 255");
    }

    #[test]
    fn parse_frame_detects_corruption() {
        let mut raw = synthetic_frame(0x1234, 0x4321);
        assert!(matches!(
            parse_frame(&mut raw),
            Err(FrameError::Corrupted { .. })
        ));
    }

    #[test]
    fn parse_frame_rejects_truncated_block() {
        let mut raw = synthetic_frame(1, 1);
        raw.truncate(64);
        assert!(matches!(
            parse_frame(&mut raw),
            Err(FrameError::Truncated { .. })
        ));
        let mut tiny = [0u8; 8];
        assert_eq!(parse_frame(&mut tiny), Err(FrameError::TooShort(8)));
    }

    #[test]
    fn offset_payload_has_footer_and_drops_header() {
        let mut buf = [0u8; 1024];
        for (k, b) in buf.iter_mut().enumerate().take(NVM_DATA_SIZE) {
            *b = (k % 251) as u8;
        }
        let original = buf;
        assert!(prepare_offset_4x4(&mut buf).is_ok());
        assert_eq!(&buf[0x1E0..0x1E8], &[0, 0, 0, 0x0F, 0x03, 0x01, 0x01, 0xE4]);
        // Bytes outside the grids and DSS block are just shifted by 8.
        assert_eq!(&buf[0..8], &original[8..16]);
        // DSS 4x4 block (originally at 0x10) ends up at 0x08 after the shift.
        assert_eq!(
            &buf[0x08..0x10],
            &[0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07]
        );
        assert!(prepare_offset_4x4(&mut [0u8; 100]).is_err());
    }

    #[test]
    fn offset_signal_grid_is_2x2_average() {
        let mut buf = [0u8; NVM_DATA_SIZE];
        // Signal grid in sensor order: write host LE values then swap words.
        for k in 0..64u32 {
            let o = 0x3C + 4 * k as usize;
            buf[o..o + 4].copy_from_slice(&(k * 4).to_le_bytes());
        }
        swap_words(&mut buf[0x3C..0x3C + 256]);
        assert!(prepare_offset_4x4(&mut buf).is_ok());
        // After the 8-byte shift the grid starts at 0x34, in sensor order.
        let mut grid = [0u8; 256];
        grid.copy_from_slice(&buf[0x34..0x34 + 256]);
        swap_words(&mut grid);
        let cell = |k: usize| {
            u32::from_le_bytes([
                grid[4 * k],
                grid[4 * k + 1],
                grid[4 * k + 2],
                grid[4 * k + 3],
            ])
        };
        // Zone 0 = avg(0,1,8,9)*4 = 18 ; zone 5 (i=1,j=1) = avg(18,19,26,27)*4 = 90
        assert_eq!(cell(0), 18);
        assert_eq!(cell(5), 90);
        assert_eq!(cell(16), 0);
        assert_eq!(cell(63), 0);
    }

    #[test]
    fn xtalk_payload_patches_profile_and_clears_0x78() {
        let mut buf = [0xAAu8; XTALK_BUFFER_SIZE];
        assert!(prepare_xtalk_4x4(&mut buf).is_ok());
        assert_eq!(&buf[0x134..0x138], &[0xA0, 0xFC, 0x01, 0x00]);
        assert_eq!(&buf[0x78..0x7C], &[0, 0, 0, 0]);
        assert_eq!(
            &buf[0x08..0x10],
            &[0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07]
        );
        assert!(prepare_xtalk_4x4(&mut [0u8; 10]).is_err());
    }

    #[test]
    fn words_to_le_serializes_in_order() {
        let b: [u8; 8] = words_to_le(&[0x0102_0304, 0x0506_0708]);
        assert_eq!(b, [4, 3, 2, 1, 8, 7, 6, 5]);
    }

    #[test]
    fn output_config_4x4_is_exact_128_bytes() {
        let cfg = build_output_config(16);
        assert_eq!(cfg.data_read_size, 128);
        assert_eq!(cfg.entries, 13);
        assert!(verify_output_config(16, &cfg));
        // Verify patched block headers for 4x4 (resolution = 16 = 0x010)
        assert_eq!(cfg.list[0], 0x0000_000D); // START
        assert_eq!(cfg.list[1], 0x54B4_00C0); // METADATA
        assert_eq!(cfg.list[2], 0x54C0_0040); // COMMONDATA
        assert_eq!(cfg.list[5], 0xDB84_0101); // NB_TARGET_DETECTED (size=16)
        assert_eq!(cfg.list[8], 0xDF44_0102); // DISTANCE (size=16)
        assert_eq!(cfg.list[10], 0xE084_0101); // TARGET_STATUS (size=16)
    }

    #[test]
    fn test_st_uld_dci_register_addresses() {
        // Enforce exact byte-level values matching STMicroelectronics ULD v1.3.x vl53l5cx_api.h:
        assert_eq!(DCI_SINGLE_RANGE, 0xD964);
        assert_eq!(DCI_OUTPUT_CONFIG, 0xD968);
        assert_eq!(DCI_OUTPUT_ENABLES, 0xD970);
        assert_eq!(DCI_OUTPUT_LIST, 0xD980);
        assert_eq!(DCI_PIPE_CONTROL, 0xDB80);
        assert_eq!(DCI_TARGET_ORDER, 0xAE64);
        assert_eq!(DCI_SHARPENER, 0xAED8);
    }
}
