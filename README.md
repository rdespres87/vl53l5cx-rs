# vl53l5cx-rs

[![Crates.io](https://img.shields.io/crates/v/vl53l5cx.svg)](https://crates.io/crates/vl53l5cx)
[![Docs.rs](https://docs.rs/vl53l5cx/badge.svg)](https://docs.rs/vl53l5cx)
[![CI](https://github.com/rdespres87/vl53l5cx-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/rdespres87/vl53l5cx-rs/actions)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](LICENSE-MIT)
[![no_std](https://img.shields.io/badge/no__std-compatible-success.svg)](https://github.com/rust-embedded)

A pure `#![no_std]` Rust driver for the **STMicroelectronics VL53L5CX** 8×8 multizone Time-of-Flight (ToF) ranging sensor.

---

## Highlights

- **Pure Rust, Zero C Toolchain:** Native, byte-exact implementation of ST's Ultra Lite Driver (ULD). No `bindgen`, no C compiler, no vendor HAL dependencies.
- **Strict `#![no_std]` & Zero Heap Allocations:** Deterministic static memory usage. Safe for bare-metal microcontrollers (ARM Cortex-M, RISC-V, ESP32).
- **`embedded-hal` 1.0 Ready:** Full compliance with standard `embedded-hal` 1.0 (`I2c`, `DelayNs`).
- **First-Class Async Support:** Optional non-blocking driver built on `embedded-hal-async` 1.0 and `embassy-time`.
- **Zero Panic Guarantee:** Exactly 0 instances of `.unwrap()`, `.expect()`, or `panic!()` in production code. All fallible bus transactions and framing errors are modeled via explicit `Result<T, Error>`.
- **Integrated Firmware Microcode:** Inlines the official ST firmware binary blob via feature flag (`firmware-blob`).

---

## Features

- **Dual-Resolution Multizone Ranging:**
  - **4×4 Mode (16 discrete zones):** High-speed ranging up to 60 Hz, ideal for gesture recognition, proximity sensing, and low-latency tracking.
  - **8×8 Mode (64 discrete zones):** High-resolution spatial depth mapping up to 15 Hz, ideal for 2D scene analysis and robotics obstacle detection.
- **Rich Per-Zone Telemetry (`Frame4x4` & `Frame8x8`):**
  - Millimeter-accurate distance per zone (`distance_mm`).
  - Target qualification status (`target_status`): `5` (Valid target), `6` (Phase wrap valid), `9` (Low signal valid), `255` (No target).
  - Target count per zone (`nb_target_detected`).
  - Internal silicon temperature in °C (`silicon_temp_degc`).
- **Flexible Sensor Configuration:**
  - **Resolution:** Seamlessly switch between `set_resolution_4x4()` and `set_resolution_8x8()`.
  - **Ranging Frequency:** Dynamic adjustment via `set_ranging_frequency_hz()` (1–60 Hz in 4×4, 1–15 Hz in 8×8).
  - **Target Selection:** Configure `set_target_order()` to prioritize either `TargetOrder::Closest` or `TargetOrder::Strongest`.
  - **Motion Blur / Sharpener Filter:** Adjust edge contrast with `set_sharpener_percent(0..=99)`.
  - **Dynamic I2C Readdressing:** Reconfigure sensor I2C address via `set_i2c_address()` to chain multiple VL53L5CX sensors on a single bus using LPn reset pins.
- **Factory Calibration & Crosstalk:** Embedded default configuration, optical crosstalk compensation, and NVM compensation tables.

---

## Installation

Add `vl53l5cx` to your `Cargo.toml`:

```toml
[dependencies]
vl53l5cx = "0.1.0"
```

For async Embassy projects:

```toml
[dependencies]
vl53l5cx = { version = "0.1.0", features = ["async"] }
```

### Feature Flags

| Feature | Default | Description |
| :--- | :---: | :--- |
| `firmware-blob` | **Yes** | Inlines ST's ~84 KB firmware microcode binary in Flash. Disable if storing microcode on external SPI flash. |
| `async` | No | Enables `asynch::Vl53l5cx` using `embedded-hal-async` 1.0 and `embassy-time`. |
| `defmt` | No | Implements `defmt::Format` on frame structures and error types for zero-cost embedded logging. |
| `log` | No | Enables debug logging using the `log` crate facade. |

---

## Quickstart

### 1. Blocking API (`embedded-hal` 1.0)

```rust,no_run
#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use vl53l5cx::{Vl53l5cx, DEFAULT_ADDR};

fn run<I2C, DELAY>(i2c: I2C, delay: DELAY) -> Result<(), vl53l5cx::Error<I2C::Error>>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    // Initialize sensor on factory address (0x29) with dedicated delay handle
    let mut sensor = Vl53l5cx::new(i2c, delay, DEFAULT_ADDR);

    // Upload firmware microcode and initialize sensor
    sensor.init()?;

    // Start 4x4 continuous ranging at 15 Hz
    sensor.start_ranging()?;

    loop {
        // Poll for new measurement frame
        if sensor.check_data_ready()? {
            let frame = sensor.get_ranging_data()?;

            // Read center zone (Zone 10)
            let center_distance = frame.distance_mm[10];
            let status = frame.target_status[10];
            if status == 5 || status == 6 || status == 9 {
                // Valid measurement detected!
            }
        }
    }
}
```

### 2. High-Resolution 8×8 Ranging (64 Zones)

```rust,no_run
// Configure 8x8 resolution (64 zones) and 15 Hz ranging
sensor.set_resolution_8x8()?;
sensor.set_ranging_frequency_hz(15)?;
sensor.start_ranging()?;

loop {
    if sensor.check_data_ready()? {
        let frame = sensor.get_ranging_data_8x8()?;
        // Access 64 depth zones (0..64)
        for (zone_idx, &dist_mm) in frame.distance_mm.iter().enumerate() {
            let status = frame.target_status[zone_idx];
            if status == 5 || status == 6 || status == 9 {
                // Zone zone_idx has a valid target at dist_mm
            }
        }
    }
}
```

### 3. Async API (`embedded-hal-async` 1.0 + Embassy)

```rust,no_run
#![no_std]
#![no_main]

use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;
use vl53l5cx::asynch::Vl53l5cx;
use vl53l5cx::DEFAULT_ADDR;

#[embassy_executor::task]
async fn tof_task(i2c: impl I2c) {
    let mut sensor = Vl53l5cx::new(i2c, DEFAULT_ADDR);

    // Async non-blocking initialization
    if sensor.init().await.is_err() {
        return;
    }

    if sensor.start_ranging().await.is_err() {
        return;
    }

    loop {
        match sensor.check_data_ready().await {
            Ok(true) => {
                if let Ok(frame) = sensor.get_ranging_data().await {
                    let center_distance = frame.distance_mm[10];
                    let status = frame.target_status[10];
                    if status == 5 || status == 6 || status == 9 {
                        // Valid measurement detected!
                    }
                }
            }
            _ => Timer::after_millis(5).await,
        }
    }
}
```

---

## Hardware Connection

The VL53L5CX operates at 3.3V I/O logic levels (requires 2.8V / 3.3V analog supply and 1.8V / 3.3V I/O supply).

| Sensor Pin | MCU Connection | Description |
| :--- | :--- | :--- |
| **VIN** | 3.3V | Power supply (2.8V–3.3V) |
| **GND** | GND | Common ground |
| **SCL** | I2C SCL | Pull-up to 3.3V (2.2 kΩ recommended for 1 MHz Fast-Mode Plus) |
| **SDA** | I2C SDA | Pull-up to 3.3V |
| **LPn** | GPIO Out | Active-low reset (pull high for operational mode) |
| **PWR_EN** | GPIO Out | Power enable for internal 1.8V LDO |
| **INT** | GPIO In / IRQ | Active-low data ready interrupt |

---

## License

This project is dual-licensed under:
- **MIT License** ([LICENSE-MIT](LICENSE-MIT))
- **Apache License, Version 2.0** ([LICENSE-APACHE](LICENSE-APACHE))

### STMicroelectronics Microcode License
The binary firmware blobs located in `src/vl53l5cx_fw.bin`, `src/vl53l5cx_cfg.bin`, `src/vl53l5cx_xtalk.bin`, and `src/vl53l5cx_nvm_cmd.bin` are property of STMicroelectronics and redistributed under the terms of the STMicroelectronics SLA0044 / BSD-3-Clause license ([LICENSE-ST](LICENSE-ST)).
