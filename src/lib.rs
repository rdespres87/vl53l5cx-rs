#![no_std]

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
mod driver;

pub use codec::{
    Frame4x4, Frame8x8, FrameError, GenericFrame, DCI_DSS_CONFIG, DCI_FREQ_HZ, DCI_GLARE_FILTER,
    DCI_OUTPUT_CONFIG, DCI_OUTPUT_ENABLES, DCI_OUTPUT_LIST, DCI_PIPE_CONTROL, DCI_RANGING_MODE,
    DCI_SHARPENER, DCI_SINGLE_RANGE, DCI_TARGET_ORDER, DCI_UI_RANGE_DATA, DCI_ZONE_CONFIG,
};

pub use driver::blocking::{probe, Vl53l5cx};

#[cfg(feature = "async")]
pub use driver::asynch;
#[cfg(feature = "async")]
pub use driver::asynch::{probe as probe_async, Vl53l5cx as AsyncVl53l5cx};

/// Factory I2C address (7-bit).
pub const DEFAULT_ADDR: u8 = 0x29;

pub const DEVICE_ID: u8 = 0xF0;
pub const REVISION_ID: u8 = 0x02;

pub const UI_CMD_STATUS: u16 = 0x2C00;
pub const UI_CMD_START: u16 = 0x2C04;
pub const UI_CMD_END: u16 = 0x2FFF;

/// Address for writing DCI commands to VL53L5CX.
///
/// Per ST ULD vl53l5cx_api.c, this is defined as `UI_CMD_END - 11` = 0x2FF4.
pub const DCI_CMD_ADDR: u16 = 0x2FF4;

/// Target ordering within each multi-target detection zone (ST ULD `vl53l5cx_set_target_order`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[must_use = "sensor driver operations return Errors that should be handled"]
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
// Unit Tests
// ============================================================================

#[cfg(test)]
extern crate std;

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{I2c, Operation};
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
        #[allow(clippy::erasing_op)]
        let s0 = ((0u16 * 255) / 100) as u8;
        assert_eq!(s0, 0);
        let s35 = ((35u16 * 255) / 100) as u8;
        assert_eq!(s35, 89);
        let roundtrip = (s35 as u16 * 100 / 255) as u8;
        assert_eq!(roundtrip, 34);
        let s99 = ((99u16 * 255) / 100) as u8;
        assert_eq!(s99, 252);
    }

    #[test]
    fn test_dci_constants_reexported_and_correct() {
        assert_eq!(DCI_ZONE_CONFIG, 0x5450);
        assert_eq!(DCI_RANGING_MODE, 0xAD30);
        assert_eq!(DCI_DSS_CONFIG, 0xAD38);
        assert_eq!(DCI_FREQ_HZ, 0x5458);
        assert_eq!(DCI_SINGLE_RANGE, 0xD964);
        assert_eq!(DCI_OUTPUT_CONFIG, 0xD968);
        assert_eq!(DCI_OUTPUT_ENABLES, 0xD970);
        assert_eq!(DCI_OUTPUT_LIST, 0xD980);
        assert_eq!(DCI_PIPE_CONTROL, 0xDB80);
        assert_eq!(DCI_GLARE_FILTER, 0xE108);
        assert_eq!(DCI_UI_RANGE_DATA, 0x5440);
        assert_eq!(DCI_TARGET_ORDER, 0xAE64);
        assert_eq!(DCI_SHARPENER, 0xAED8);
    }

    #[derive(Debug, PartialEq, Eq)]
    struct CustomBusError;
    impl embedded_hal::i2c::Error for CustomBusError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }
    struct FailingBus;
    impl embedded_hal::i2c::ErrorType for FailingBus {
        type Error = CustomBusError;
    }
    impl I2c for FailingBus {
        fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
            Err(CustomBusError)
        }
        fn write(&mut self, _address: u8, _write: &[u8]) -> Result<(), Self::Error> {
            Err(CustomBusError)
        }
        fn write_read(
            &mut self,
            _address: u8,
            _write: &[u8],
            _read: &mut [u8],
        ) -> Result<(), Self::Error> {
            Err(CustomBusError)
        }
        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            Err(CustomBusError)
        }
    }

    #[test]
    fn test_stop_ranging_resets_state_even_on_bus_error() {
        let bus = FailingBus;
        let mut sensor = Vl53l5cx::new(bus, MockDelay, DEFAULT_ADDR);
        sensor.streamcount = 42;
        sensor.data_read_size = 128;

        let res = sensor.stop_ranging();
        assert!(res.is_err());
        assert_eq!(sensor.streamcount, 255);
        assert_eq!(sensor.data_read_size, 0);
    }

    struct RecordingBus {
        writes: Vec<Vec<u8>>,
    }
    impl embedded_hal::i2c::ErrorType for RecordingBus {
        type Error = CustomBusError;
    }
    impl I2c for RecordingBus {
        fn read(&mut self, _address: u8, _read: &mut [u8]) -> Result<(), Self::Error> {
            Ok(())
        }
        fn write(&mut self, _address: u8, write: &[u8]) -> Result<(), Self::Error> {
            self.writes.push(write.to_vec());
            Ok(())
        }
        fn write_read(
            &mut self,
            _address: u8,
            _write: &[u8],
            _read: &mut [u8],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
        fn transaction(
            &mut self,
            _address: u8,
            _operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            panic!("transaction() is not used: multi-byte writes use contiguous write() frames per ST ULD specification");
        }
    }

    #[test]
    fn test_write_multi_chunking_and_address_increment() {
        let mut bus = RecordingBus { writes: Vec::new() };
        let payload = [0xAAu8; 600];
        let res = driver::blocking::write_multi_blocking(
            &mut bus,
            &mut MockDelay,
            DEFAULT_ADDR,
            0x1000,
            &payload,
        );
        assert!(res.is_ok());
        assert_eq!(bus.writes.len(), 3);
        // Chunk 0: 256 bytes payload + 2 bytes addr (0x1000)
        assert_eq!(bus.writes[0].len(), 258);
        assert_eq!(bus.writes[0][0..2], [0x10, 0x00]);
        // Chunk 1: 256 bytes payload + 2 bytes addr (0x1000 + 256 = 0x1100)
        assert_eq!(bus.writes[1].len(), 258);
        assert_eq!(bus.writes[1][0..2], [0x11, 0x00]);
        // Chunk 2: remaining 88 bytes payload + 2 bytes addr (0x1100 + 256 = 0x1200)
        assert_eq!(bus.writes[2].len(), 90);
        assert_eq!(bus.writes[2][0..2], [0x12, 0x00]);
    }

    #[cfg(feature = "defmt")]
    #[test]
    fn test_defmt_format_implemented() {
        use defmt::Format;
        fn assert_format<T: Format>() {}
        assert_format::<Error<core::convert::Infallible>>();
        assert_format::<TargetOrder>();
        assert_format::<codec::ReadyState>();
        assert_format::<codec::PollOutcome>();
        assert_format::<FrameError>();
        assert_format::<codec::BufferTooSmall>();
        assert_format::<codec::OutputConfig>();
        assert_format::<Frame4x4>();
        assert_format::<Frame8x8>();
    }
}
