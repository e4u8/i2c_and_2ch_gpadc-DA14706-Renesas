# adc_2ch_AND_I2C_aht20

A combined FreeRTOS application for the **Renesas DA14706** that simultaneously reads two analog voltage channels via the GPADC and measures temperature and humidity from an **AHT20** sensor over I2C. Both peripherals run in independent tasks and their data is merged into a single CSV-style output line.

---

## Overview

This project merges two standalone examples:

| Original project | Contribution |
|---|---|
| GPADC-2channels | Dual-channel ADC task (`gpadc_app_task`) |
| AHT20-i2c_sensor | I2C temperature/humidity task (`aht20_task`) |

Two FreeRTOS tasks run concurrently:

- **`gpadc_app_task`** — opens Ch0 and Ch1 sequentially at full speed, converts raw ADC counts to millivolts (with optional two-point calibration), then prints a combined output line.
- **`aht20_task`** — reads temperature and humidity from the AHT20 sensor every 2 seconds and stores the latest values in shared `volatile` variables (`g_last_temp_c`, `g_last_hum_percent`).

The GPADC task snapshots these shared variables on each iteration so every output line includes the most recent environmental reading without requiring synchronisation primitives (a single aligned `float` read on Cortex-M33 is atomic on the bus).

---

## Hardware

| Item | Detail |
|---|---|
| Target MCU | Renesas DA14706 (Cortex-M33) |
| Development board | DA14706 Pro Kit (or compatible) |
| Sensor | ASAIR AHT20 temperature & humidity |
| I2C bus | MikroBUS slot #2 (default) |
| ADC inputs | Two single-ended channels |

### Pin assignments

| Signal | Port | Pin |
|---|---|---|
| I2C SCL (MikroBUS #2) | P1 | P1.12 |
| I2C SDA (MikroBUS #2) | P1 | P1.11 |
| ADC Ch0 | P0 | P0.5 |
| ADC Ch1 | P0 | P0.6 |

To use **MikroBUS slot #1** instead, update `I2C_MASTER_SCL_PORT/PIN` and `I2C_MASTER_SDA_PORT/PIN` in `config/peripheral_setup.h` and set `#define MIKRO_BUS 1` in your `custom_config_*.h`.

The AHT20 I2C address is **0x38** (7-bit, fixed by the device).

---

## Project structure

```
├── main.c                          Entry point; hardware init; creates both FreeRTOS tasks
├── gpadc_app.c / include/gpadc_app.h   GPADC dual-channel task
├── aht20_task.c / include/aht20_task.h I2C AHT20 task; exposes shared volatile variables
├── drivers/
│   └── aht20/
│       ├── driver_aht20.c/h            AHT20 libdriver (open-source)
│       └── driver_aht20_interface.c/h  DA14706-specific I2C/delay/print bindings
└── config/
    ├── peripheral_setup.h          I2C pin definitions
    ├── platform_devices.c/h        GPADC and I2C adapter device descriptors
    ├── custom_config_ram.h         Build config – execute from RAM
    ├── custom_config_qspi.h        Build config – execute from QSPI flash
    └── custom_config_oqspi.h       Build config – execute from OQSPI flash
```

---

## Serial output format

The application prints one CSV line per GPADC conversion cycle (as fast as the ADC can run):

```
<mv_ch0>,<mv_ch1>,<temp_x10>,<humidity_percent>
```

| Field | Type | Example | Notes |
|---|---|---|---|
| `mv_ch0` | integer (mV) | `1823` | Ch0 corrected millivolts |
| `mv_ch1` | integer (mV) | `945` | Ch1 corrected millivolts |
| `temp_x10` | integer | `234` | Temperature × 10, i.e. 23.4 °C |
| `humidity_percent` | integer | `58` | Relative humidity % |

This format is designed for direct import into a **UART analyser / serial plotter**. To switch to human-readable output, uncomment the alternative `printf` block inside `gpadc_app_task()` in `gpadc_app.c`.

Temperature is only updated every 2 seconds; the field will read `0` for the first ~2 s after boot until the AHT20 task completes its first measurement.

---

## ADC configuration

Both channels use the DA14706 GPADC adapter (`ad_gpadc`) with the following settings:

| Setting | Value |
|---|---|
| Input mode | Single-ended |
| Input attenuator | Up to 3.6 V |
| Oversampling | 4 samples |
| Sample time | 4 |
| Chopping | Enabled |
| Result mode | Normal (16-bit) |
| GPIO voltage level | VDD1V8P |

### Calibration

A two-point linear calibration (offset + gain) is applied to each channel via `correct_mv()`. The default coefficients are `offset = 0.0`, `gain = 1.0` (pass-through). Replace the `OFFSET_MV_CH0/1` and `GAIN_CH0/1` macros in `gpadc_app.c` with empirically measured values to improve accuracy.

---

## Build configurations

Three configurations are provided, matching the standard DA14706 SDK templates:

| Configuration | Description |
|---|---|
| `Debug_RAM` | Loads and runs entirely from RAM; no flash required. Fastest iteration cycle. |
| `Debug_QSPI` | Stored in and executed from QSPI flash. |
| `Debug_OQSPI` | Stored in and executed from Octal QSPI flash. |

All configurations disable the watchdog (`dg_configUSE_WDOG = 0`) and enable `CONFIG_RETARGET` for `printf` output over UART.

---

## Getting started

### Prerequisites

- Renesas **SmartSnippets Studio** (Eclipse-based IDE) with the DA14706 SDK installed in the workspace root (the `.project` file expects `SDKROOT` to resolve to `${WORKSPACE_LOC}`).
- A JTAG/SWD probe supported by the DA14706 Pro Kit.
- AHT20 sensor module connected to MikroBUS slot #2.

### Steps

1. Clone or copy this project into your SmartSnippets Studio workspace.
2. Import the project: **File → Import → Existing Projects into Workspace**.
3. Select the desired build configuration (e.g. `Debug_RAM`) from the build toolbar.
4. Build: **Project → Build Project** (or `Ctrl+B`).
5. Flash/run via the debug configuration included in `.cproject`.
6. Open a serial terminal at the board's UART baud rate and observe the CSV output.

---

## Dependencies

- **Renesas DA14706 SDK** — GPADC adapter (`ad_gpadc`), I2C adapter (`ad_i2c`), power/clock managers, FreeRTOS OSAL wrapper.
- **AHT20 libdriver** — bundled in `drivers/aht20/`; platform-specific bindings are in `driver_aht20_interface.c`.

---

## License

The AHT20 libdriver (`drivers/aht20/`) is distributed under its original open-source licence (see file headers). The DA14706 SDK files referenced via linked resources carry the Dialog Semiconductor / Renesas copyright notice. Application code in this repository was written by the project author.
