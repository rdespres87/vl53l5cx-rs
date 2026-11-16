//! VL53L5CX hardware driver implementations (blocking and async).

#[cfg(feature = "async")]
pub mod asynch;
pub mod blocking;

#[cfg(feature = "firmware-blob")]
pub(crate) const FIRMWARE: &[u8] = include_bytes!("../vl53l5cx_fw.bin");
#[cfg(feature = "firmware-blob")]
pub(crate) const DEFAULT_CONFIGURATION: &[u8] = include_bytes!("../vl53l5cx_cfg.bin");
#[cfg(feature = "firmware-blob")]
pub(crate) const DEFAULT_XTALK: &[u8] = include_bytes!("../vl53l5cx_xtalk.bin");
#[cfg(feature = "firmware-blob")]
pub(crate) const GET_NVM_CMD: &[u8] = include_bytes!("../vl53l5cx_nvm_cmd.bin");

// Compile-time guarantees on the ST blobs (fails the build, never at runtime).
#[cfg(feature = "firmware-blob")]
const _: () = assert!(FIRMWARE.len() == 0x15000);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(DEFAULT_CONFIGURATION.len() == 972);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(DEFAULT_XTALK.len() == crate::codec::XTALK_BUFFER_SIZE);
#[cfg(feature = "firmware-blob")]
const _: () = assert!(GET_NVM_CMD.len() == 40);

#[cfg(feature = "firmware-blob")]
pub(crate) const FW_PAGE_09: &[u8] = FIRMWARE.split_at(0x8000).0;
#[cfg(feature = "firmware-blob")]
pub(crate) const FW_REST: &[u8] = FIRMWARE.split_at(0x8000).1;
#[cfg(feature = "firmware-blob")]
pub(crate) const FW_PAGE_0A: &[u8] = FW_REST.split_at(0x8000).0;
#[cfg(feature = "firmware-blob")]
pub(crate) const FW_PAGE_0B: &[u8] = FW_REST.split_at(0x8000).1;
#[cfg(feature = "firmware-blob")]
const _: () = assert!(FW_PAGE_0B.len() == 0x5000);
