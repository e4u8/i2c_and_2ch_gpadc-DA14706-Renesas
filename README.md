# adc_2ch_AND_I2C_aht20 — Combined Power Meter + Environmental Sensor

Dual-channel DMA-assisted ADC power measurement firmware for the **Renesas DA14706** (DA1470x
family), combined with real-time temperature and humidity acquisition via an **AHT20** sensor
over I2C. Both peripherals run in independent FreeRTOS tasks and their data is merged into a
single per-second diagnostic output.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Target](#hardware-target)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
  - [RTOS Task Layout](#rtos-task-layout)
  - [Acquisition Flow](#acquisition-flow)
  - [DMA and the Adapter Layer](#dma-and-the-adapter-layer)
  - [Shared Environmental Data](#shared-environmental-data)
- [ADC Configuration](#adc-configuration)
- [I2C / AHT20 Configuration](#i2c--aht20-configuration)
- [GPIO Pin Mapping](#gpio-pin-mapping)
- [Clock & Power Configuration](#clock--power-configuration)
- [RMS and Power Calculations](#rms-and-power-calculations)
  - [Why mean subtraction is required](#why-mean-subtraction-is-required)
  - [Two-pass batch algorithm](#two-pass-batch-algorithm)
  - [Windowed accumulation](#windowed-accumulation)
  - [Power triangle](#power-triangle)
  - [Scaled integer output](#scaled-integer-output)
- [Calibration](#calibration)
  - [Signal conditioning constants](#signal-conditioning-constants)
  - [Calibration procedure](#calibration-procedure)
  - [Critical: calibrate with both peripherals active](#critical-calibrate-with-both-peripherals-active)
- [Serial Output Format](#serial-output-format)
- [Inter-Channel Skew](#inter-channel-skew)
- [Build Configurations](#build-configurations)
  - [Required DMA flags](#required-dma-flags)
- [Task Priorities](#task-priorities)
- [Key Parameters](#key-parameters)
- [Known Limitations](#known-limitations)

---

## Overview

This project is the combination of two standalone firmwares:

| Original project | Contribution |
|---|---|
| `adc_2channels_DMA` | Dual-channel DMA ADC task, batch acquisition, AC power calculations |
| `AHT20-i2c_sensor-DA14706-Renesas` | I2C temperature & humidity task (AHT20 libdriver) |

**Key capabilities:**

- Interleaved single-sample DMA acquisition — CH0 (current) and CH1 (voltage) sampled
  back-to-back; CPU yields to the RTOS scheduler during each conversion
- ARM DWT cycle counter for sub-microsecond timestamps with zero OS overhead
- One-time startup inter-channel skew measurement
- AC RMS, active power P, apparent power S, reactive power Q, and power factor PF computed
  via a numerically stable two-pass mean-subtraction algorithm over a 1-second window
- Temperature and humidity appended to every per-second diagnostic line from the AHT20 sensor
- Scaled integer output — no float `printf`, shared format suitable for a future BLE payload

---

## Hardware Target

| Parameter  | Value                                         |
|------------|-----------------------------------------------|
| SoC        | Renesas DA14706-00                            |
| CPU        | ARM Cortex-M33                                |
| FPU        | FPv5-SP-D16 (single-precision hardware float) |
| BLE core   | Dedicated CMAC (Cortex-M0+), separate from CM33 |
| SDK        | DA1470x SDK                                   |
| RTOS       | FreeRTOS                                      |
| Toolchain  | arm-none-eabi-gcc                             |
| IDE        | Eclipse CDT (SmartSnippets Studio)            |
| Sensor     | ASAIR AHT20 temperature & humidity            |
| I2C bus    | MikroBUS slot #2 (default; see `peripheral_setup.h`) |

---

## Project Structure

```
├── main.c                               Entry point; clock/power init; creates both tasks
├── gpadc_app.c                          GPADC batch acquisition + power calculation task
├── aht20_task.c                         AHT20 I2C task; updates shared volatile variables
├── include/
│   ├── gpadc_app.h
│   └── aht20_task.h
├── drivers/
│   └── aht20/
│       ├── driver_aht20.c/h             AHT20 libdriver (open-source)
│       └── driver_aht20_interface.c/h   DA14706-specific I2C / delay / print bindings
└── config/
    ├── peripheral_setup.h               I2C pin definitions and AHT20 I2C address
    ├── platform_devices.c/h             GPADC and I2C adapter device descriptors
    ├── custom_config_ram.h              BSP feature flags — execute from RAM
    ├── custom_config_qspi.h             BSP feature flags — execute from QSPI flash
    └── custom_config_oqspi.h            BSP feature flags — execute from OQSPI flash
```

Hardware initialisation is centralised in `main.c → prvSetupHardware()`. Both
`gpadc_app_init()` and the AHT20 interface's `iic_init()` are intentionally empty stubs —
the adapter layer manages peripheral lifecycle.

---

## Architecture

### RTOS Task Layout

```
main()
  └─ SysInit task  (OS_TASK_PRIORITY_HIGHEST — runs once, then self-deletes)
       ├─ cm_sys_clk_init(sysclk_XTAL32M)       32 MHz crystal
       ├─ cm_apb/ahb_set_clock_divider(div1)     full-speed buses
       ├─ pm_sleep_mode_set(pm_mode_idle)        CPU halts, DMA stays active
       ├─ pm_set_sys_wakeup_mode(fast)
       ├─ prvSetupHardware()                     GPIO, power domain, adapter IO config
       ├─ creates: gpadc_app_task  (OS_TASK_PRIORITY_NORMAL)
       └─ creates: aht20_task      (OS_TASK_PRIORITY_NORMAL - 1)
```

The GPADC task runs at higher priority so it resumes immediately after each DMA completion
event, regardless of what the AHT20 task is doing. The AHT20 task gets CPU time during the
GPADC task's DMA yield windows (~200 µs each) and during its own 2-second sleep intervals —
this is sufficient because I2C transactions are hardware-driven and complete independently of
CPU scheduling.

### Acquisition Flow

```
[Startup — once]
  Enable DWT CYCCNT (32 MHz free-running cycle counter)
  Open CH0 → read 1 dummy sample → snapshot DWT → Close CH0
  Open CH1 → read 1 dummy sample → snapshot DWT → Close CH1
  skew_cycles = t_ch1_done - t_ch0_done
  Print: *skew=<cycles> (~<µs> us)

[Main loop — repeats forever]

  ① Interleaved batch acquisition
     t_acq_start = DWT->CYCCNT
     For i = 0 .. BATCH_SIZE-1:
       Open CH0 → read raw0[i] (DMA, CPU yields ~200 µs) → Close CH0
       Open CH1 → read raw1[i] (DMA, CPU yields ~200 µs) → Close CH1
     acq_cycles_accum += DWT->CYCCNT - t_acq_start
     batches_in_window++

  ② Per-batch metrics accumulation  (two-pass, both channels together)
     m = compute_batch_metrics(raw1, raw0)
     rms2_accum_ch1 += m.rms_mv_v²    ← voltage channel variance
     rms2_accum_ch0 += m.rms_mv_i²    ← current channel variance
     p_accum        += m.p_mvsq       ← mean cross-product [mV²]

  ③ Per-sample CSV print  [disabled — #if 0, enable for serial plotter]

  ④ Diagnostics — once per second
     fs_acq  = (total_pairs × 2 × CPU_CLOCK_HZ) / acq_cycles_accum
     us_pair = acq_cycles_accum / (total_pairs × 32)

     v_rms = K_V × sqrt(rms2_accum_ch1 / batches) / 1000        [V]
     i_rms = K_I × sqrt(rms2_accum_ch0 / batches) / HALL_SENS   [A]
     p_w   = P_SIGN × P_SCALE × (p_accum / batches)             [W]
     s_va  = v_rms × i_rms                                       [VA]
     q_var = sqrt( |s_va² − p_w²| )                             [VAr]
     pf    = p_w / s_va

     temp_c    = g_last_temp_c       ← volatile read from AHT20 task
     humidity  = g_last_hum_percent

     Print: *fs_acq  *us_pair
     Print: *Vrms  *Irms
     Print: *P  *S  *Q  *PF
     Print: *Temp  *Hum
     Reset all accumulators
```

`ad_gpadc_read_nof_conv(handle, 1, &dest)` is **synchronous and blocking** — it starts one
DMA-backed conversion, suspends the task (yielding the CPU to the scheduler), and returns only
when the DMA completion ISR fires. The CPU is not busy-waiting during conversions.

### DMA and the Adapter Layer

DMA is active on the GPADC. Both `custom_config_*.h` files define:

```c
#define dg_configGPADC_DMA_SUPPORT   (1)
#define dg_configUSE_HW_DMA          (1)
```

**Both flags are required together.** `dg_configGPADC_DMA_SUPPORT` enables the
`#if HW_GPADC_DMA_SUPPORT` blocks in `platform_devices.c` that compile in the DMA struct
and wire `dma_setup` in the driver config. `dg_configUSE_HW_DMA` ensures the DMA hardware
driver itself (`hw_dma.c`) is compiled and the DMA controller is initialised at startup.

If either flag is missing, the DMA config is silently compiled out and `continuous = true` in
the driver config will cause different ADC sampling behaviour compared to the calibrated
standalone project, resulting in a small but consistent voltage reading deviation.

The I2C adapter (`ad_i2c`) does **not** use DMA (`HW_DMA_CHANNEL_INVALID` in
`platform_devices.c`). There is no DMA channel conflict between the two peripherals.

### Shared Environmental Data

The AHT20 task writes to two `volatile` globals defined in `aht20_task.c`:

```c
volatile float   g_last_temp_c     = 0.0f;
volatile uint8_t g_last_hum_percent = 0;
```

The GPADC task reads them with a single snapshot per diagnostic window:

```c
float   temp_c = g_last_temp_c;
uint8_t hum    = g_last_hum_percent;
```

No mutex is needed. On Cortex-M33, a 32-bit aligned `float` read/write is atomic on the bus.
This is a single-producer / single-consumer pattern — only `aht20_task` writes, only
`gpadc_app_task` reads.

The fields will report `0` for the first ~2 seconds after boot until the AHT20 task completes
its first measurement.

---

## ADC Configuration

| Parameter        | Value                                                |
|------------------|------------------------------------------------------|
| CH0 signal       | Current (P0.5)                                       |
| CH1 signal       | Voltage (P0.6)                                       |
| Input mode       | Single-ended                                         |
| Input attenuator | Up to 3.6 V                                          |
| Oversampling     | 4 samples                                            |
| Sample time      | 4 (multiplier × 8 × ADC_CLK)                        |
| Chopping         | Enabled (noise reduction)                            |
| Continuous mode  | `true` (required for DMA-backed `read_nof_conv`)     |
| DMA channel      | `HW_DMA_CHANNEL_0` (must be even — hardware constraint) |
| DMA priority     | `HW_DMA_PRIO_2`                                      |
| DMA mode         | One-shot, no midpoint IRQ (`irq_nr_of_trans = 0`)    |
| Result mode      | Normal (16-bit after oversampling)                   |

Both channels share one `gpadc_dma_cfg` struct. The GPADC block is a single hardware resource;
CH0 and CH1 are never active simultaneously, so sharing the DMA config is correct and
required (only one `dma_setup` slot exists per controller instance).

Full hardware parameters are in `config/platform_devices.c`.

---

## I2C / AHT20 Configuration

| Parameter     | Value                                     |
|---------------|-------------------------------------------|
| Controller    | HW_I2C1                                   |
| Speed         | Standard (100 kHz)                        |
| Addressing    | 7-bit                                     |
| AHT20 address | 0x38 (fixed by device)                    |
| SCL pin       | P1.12 (MikroBUS #2)                       |
| SDA pin       | P1.11 (MikroBUS #2)                       |
| DMA           | Disabled (`HW_DMA_CHANNEL_INVALID`)        |
| GPIO level    | VDD1V8P, open-drain outputs               |
| Sample period | 2000 ms                                   |

To use **MikroBUS slot #1**, update the pin definitions in `config/peripheral_setup.h`.

The AHT20 driver is the open-source libdriver implementation. The DA14706-specific bindings
in `driver_aht20_interface.c` open and close the I2C adapter around every transaction so the
power manager can control PD_COM between reads.

---

## GPIO Pin Mapping

| Signal       | Port | Pin  | Function            |
|--------------|------|------|---------------------|
| ADC CH0 (I)  | P0   | P0.5 | `HW_GPIO_FUNC_ADC`  |
| ADC CH1 (V)  | P0   | P0.6 | `HW_GPIO_FUNC_ADC`  |
| I2C SCL      | P1   | P1.12 | `HW_GPIO_FUNC_I2C_SCL` |
| I2C SDA      | P1   | P1.11 | `HW_GPIO_FUNC_I2C_SDA` |

ADC pins are configured as high-impedance inputs in `periph_init()`. I2C and ADC pins are
managed in their "off" state by the adapter layer between transactions
(`ad_gpadc_io_config` / `ad_i2c_io_config` with `AD_IO_CONF_OFF`).

---

## Clock & Power Configuration

| Parameter     | Setting                                      |
|---------------|----------------------------------------------|
| System clock  | 32 MHz XTAL32M (crystal, ±20–50 ppm)         |
| AHB divider   | div1 → 32 MHz                                |
| APB divider   | div1 → 32 MHz                                |
| LP clock      | 32.768 kHz crystal (drives FreeRTOS tick)    |
| Sleep mode    | `pm_mode_idle` — CPU halts, DMA stays active |
| Wakeup mode   | `pm_sys_wakeup_mode_fast`                    |
| DWT tick      | 1 cycle = 31.25 ns at 32 MHz                 |

**Why `pm_mode_idle` and not `pm_mode_extended_sleep`:** The GPADC DMA runs at ~4400 ISRs/s.
Extended sleep powers down more silicon between wakeups and would incur re-initialisation
overhead on every DMA wakeup event. Idle mode halts only the CPU; DMA, GPADC, and I2C remain
continuously powered, which is correct when one task is acquiring ADC data continuously.

**Why XTAL32M:** The internal RC oscillator (RCHS) has ±2–3% frequency accuracy that drifts
with temperature. XTAL32M at ±20–50 ppm is required because the DWT µs conversion constant
(`cycles / 32`) is only accurate at exactly 32 MHz, and all power and timing diagnostics
derive from DWT timestamps.

---

## RMS and Power Calculations

### Why mean subtraction is required

Both analog signals are bipolar AC (mains voltage and Hall sensor current), but the signal
conditioning circuit adds a DC offset to each signal so the full swing fits within the ADC's
positive input range. Squaring raw ADC values directly would cause the DC offset (~1650 mV
for a 3.3 V mid-rail) to dominate the sum and drown the AC content. The correct approach
removes the DC component first:

```
rms_mv = sqrt( (1/N) × Σ (adc_mv[i] − mean)² )
```

The same mean-subtracted residuals are used for the active power cross-product, which also
prevents the spurious `DC_V × DC_I` term from contaminating P:

```
P_mvsq = (1/N) × Σ ( (v[i] − mean_v) × (i[i] − mean_i) )
```

### Two-pass batch algorithm

`compute_batch_metrics()` in `gpadc_app.c` processes both channels in one function call:

**Pass 1** — convert both channels to mV, compute `mean_v` and `mean_i` (the DC offsets).

**Pass 2** — using the same residuals `rv` and `ri`:
- Accumulate `rv²` and `ri²` → RMS numerators for each channel
- Accumulate `rv × ri` → mean cross-product for P

This eliminates redundant passes and ensures the identical `mean_v`/`mean_i` values are used
for both RMS and P.

### Windowed accumulation

A 64-sample batch covers ~1.4 mains cycles — not an integer. Variance is accumulated across
all batches within a 1-second window; the square root is taken once at the boundary:

```
rms_window = sqrt( Σ batch_variance / batches_in_window )
```

This is mathematically correct because averaging variances from multiple batches of a
stationary signal converges to the true variance.

### Power triangle

```
S (VA)  =  Vrms × Irms                  apparent power
P (W)   =  P_SIGN × P_SCALE × P_mvsq   active (real) power
Q (VAr) =  sqrt( |S² − P²| )           reactive power
PF      =  P / S                        power factor (-1 to +1)
```

`fabsf` inside the Q sqrt guards against floating-point rounding making `S² − P²` go
slightly negative when PF ≈ 1, which would produce `NaN` from `sqrtf`.

`P_SIGN = -1.0f` corrects for a polarity inversion introduced by the signal conditioning
circuit on one channel. Set to `+1.0f` if the hardware inversion is removed.

### Scaled integer output

The DA1470x SDK links **newlib-nano**, which strips float `printf` by default. Results are
converted to scaled integers before printing — the same integers serve directly as a future
BLE payload with no additional conversion.

| Constant      | Scale | Unit          | Example              |
|---------------|-------|---------------|----------------------|
| `V_RMS_SCALE` | 100   | centivolts    | 230.45 V → `23045`   |
| `I_RMS_SCALE` | 1000  | milliamps     | 1.500 A → `1500`     |
| `P_W_SCALE`   | 100   | centiwatts    | 1926.37 W → `192637` |
| `S_VA_SCALE`  | 100   | centi VA      | 1939.57 VA → `193957`|
| `Q_VAR_SCALE` | 100   | centi VAr     | 225.88 VAr → `22588` |
| `PF_SCALE`    | 1000  | milli PF      | 0.993 → `993`        |

---

## Calibration

### Signal conditioning constants

Four constants in `gpadc_app.c` describe the full signal chain from MCU pin to physical unit:

```c
#define K_V                       (289.269f)  // mains V per ADC V  [V/V]
#define K_I                       (1.298f)    // Hall mV per ADC mV [mV/mV]
#define HALL_SENSITIVITY_MV_PER_A (80.0f)     // Hall sensor datasheet [mV/A]
#define P_SIGN                    (-1.0f)     // polarity correction
```

Per-channel ADC-level correction constants (applied before RMS):

```c
#define OFFSET_MV_CH0   0.0f   // subtract from raw mV reading
#define GAIN_CH0        1.0f   // divide result by this factor
#define OFFSET_MV_CH1   0.0f
#define GAIN_CH1        1.0f
```

### Calibration procedure

1. Temporarily add a print of the raw windowed RMS in mV inside the 1-second diagnostics:

```c
printf("*rms_mv_CH1=%"PRId32"  *rms_mv_CH0=%"PRId32"\n",
       (int32_t)(rms_mv1_w * 10),
       (int32_t)(rms_mv0_w * 10));
```

Values are in units of 0.1 mV. Divide by 10 to get mV.

2. Apply a stable resistive load. Record `rms_mv1_w` and `rms_mv0_w` alongside the reference
   Power Analyser's Vrms and Irms:

```
K_V = PA_Vrms [V]  × 1000 / rms_mv1_w [mV]
K_I = PA_Irms [A]  × HALL_SENSITIVITY_MV_PER_A / rms_mv0_w [mV]
```

3. Update the `#define` values in `gpadc_app.c` and rebuild.

### Critical: calibrate with both peripherals active

**Always calibrate with this combined project running, not with the standalone DMA project.**

The GPADC reference voltage is derived from the COM power domain (VDD_COM). The I2C peripheral
(HW_I2C1) is also on VDD_COM. Having I2C powered changes the current drawn from VDD_COM
slightly, which shifts the effective ADC reference and produces a small but consistent
deviation from calibration values derived with I2C unpowered.

If K_V and K_I are measured with the DMA-only project and then used here, the voltage and
current readings will be slightly off even though the code is otherwise identical. Re-calibrate
once with the AHT20 task running and I2C active to get constants that are accurate for the
final operating condition.

---

## Serial Output Format

All output is ASCII text over the retarget UART (`CONFIG_RETARGET` in `custom_config_*.h`).
Lines starting with `*` are diagnostic markers. One group is emitted per second.

```
*skew=6711 cycles (~209 us)
*fs_acq=4421  *us_pair=452
*Vrms=230.45 V  *Irms=8.375 A
*P=1926.37 W
*S=1939.57 VA
*Q=225.88 VAr
*PF=0.993
*Temp=22.18 C  *Hum=39%
```

| Line | When | Meaning |
|------|------|---------|
| `*skew=N cycles (~M us)` | Once at startup | CH0→CH1 inter-sample delay (see [Inter-Channel Skew](#inter-channel-skew)) |
| `*fs_acq=N  *us_pair=N` | Once per second | True ADC throughput and average pair time from DWT. Use `us_pair` for timing calculations — `fs_acq` has ±2% window-boundary jitter |
| `*Vrms=N.NN V  *Irms=N.NNN A` | Once per second | Windowed AC RMS |
| `*P=N.NN W` | Once per second | Active power |
| `*S=N.NN VA` | Once per second | Apparent power |
| `*Q=N.NN VAr` | Once per second | Reactive power |
| `*PF=[−]N.NNN` | Once per second | Power factor, sign preserved |
| `*Temp=N.NN C  *Hum=N%` | Once per second | Last AHT20 reading (updated every ~2 s) |

A per-sample CSV print (`<mv_ch0>,<mv_ch1>`) is available inside a `#if 0` block in
`gpadc_app_task()`. Enable it for use with a serial plotter.

---

## Inter-Channel Skew

CH0 and CH1 cannot be sampled simultaneously — the GPADC has one hardware input mux. They are
sampled back-to-back. The time between CH0's sample and CH1's sample is the **skew**:

```
skew = close_CH0 overhead + open_CH1 overhead + CH1 conversion time  ≈ 209 µs
```

This is measured at startup using DWT and printed as `*skew=`. It is constant across all
samples in a batch.

**Impact on calculations:**

| Quantity | Skew effect |
|----------|-------------|
| Vrms | None — one channel only |
| Irms | None — one channel only |
| P, PF | Yes — V[i] and I[i] are not simultaneous. At 50 Hz, 209 µs = 3.76° phase error |
| Q | Amplified — Q is highly sensitive to PF error when PF ≈ 1 |
| S | Indirect only |

At this accuracy level the skew-induced error is acceptable. Future improvement: linear
interpolation of the voltage sample to the instant the current sample was captured:

```c
v_compensated = mv1[i] + (mv1[i+1] - mv1[i]) * (T_SKEW_US / T_SAMPLE_US);
```

This requires ZCD (cycle-aligned windows) to be fully effective — deferred to a future branch.

---

## Build Configurations

| Configuration  | Execution target | Recommended use          |
|----------------|------------------|--------------------------|
| `Debug_RAM`    | SRAM             | Active development (fastest iteration — no flash cycle) |
| `Debug_QSPI`   | QSPI flash       | Hardware-in-loop debug   |
| `Debug_OQSPI`  | OQSPI flash      | Hardware-in-loop debug   |

All configurations disable the watchdog (`dg_configUSE_WDOG = 0`) and enable `CONFIG_RETARGET`
for `printf` output over UART.

### Required DMA flags

All three `custom_config_*.h` files must contain all four of these defines:

```c
#define dg_configUSE_HW_GPADC         (1)
#define dg_configGPADC_ADAPTER        (1)
#define dg_configGPADC_DMA_SUPPORT    (1)   /* enables #if HW_GPADC_DMA_SUPPORT in platform_devices.c */
#define dg_configUSE_HW_DMA           (1)   /* compiles hw_dma.c and initialises the DMA controller */
```

Missing either DMA flag causes silent fallback to non-DMA ADC reads with `continuous = true`
left in the driver config — the GPADC will produce slightly different readings from the
calibrated DMA project. See [Calibration](#calibration) for the consequence.

---

## Task Priorities

```
OS_TASK_PRIORITY_HIGHEST    →  SysInit (self-deletes after creating tasks)
OS_TASK_PRIORITY_NORMAL     →  gpadc_app_task
OS_TASK_PRIORITY_NORMAL - 1 →  aht20_task
```

The GPADC task runs at higher priority so it resumes immediately after each DMA ISR.
The AHT20 task is at a lower priority because it only needs CPU in short bursts between its
2-second sleep intervals. It gets CPU time during GPADC DMA yield windows (~200 µs each);
I2C transactions complete in hardware between those windows, so the lower priority does not
delay sensor reads.

**Do not set both tasks to the same priority.** Equal priority causes FreeRTOS to time-slice
between them at every tick boundary. The AHT20 task can then preempt the GPADC task
mid-batch during I2C transactions, causing non-uniform sample timing that can introduce a
small bias in the RMS and power calculations.

---

## Key Parameters

All commonly-tuned values are at the top of `gpadc_app.c`:

| Symbol                        | Default | Purpose |
|-------------------------------|---------|---------|
| `BATCH_SIZE`                  | `64`    | Sample pairs per loop iteration. Larger → smoother averages, higher per-batch latency |
| `CPU_CLOCK_HZ`                | `32000000` | Must match `cm_sys_clk_init(sysclk_XTAL32M)`. Used for all DWT µs conversions |
| `OFFSET_MV_CH0/1`             | `0.0f`  | Per-channel DC offset correction [mV], applied before RMS |
| `GAIN_CH0/1`                  | `1.0f`  | Per-channel gain correction, applied before RMS |
| `K_V`                         | `289.269` | Mains Volts per ADC Volt — transformer + signal conditioning ratio |
| `K_I`                         | `1.298` | Hall sensor output mV per ADC mV — signal conditioning gain |
| `HALL_SENSITIVITY_MV_PER_A`   | `80.0`  | Hall sensor sensitivity from datasheet [mV/A] |
| `P_SIGN`                      | `-1.0f` | `-1.0f` if signal conditioning inverts one channel; `+1.0f` otherwise |
| `P_SCALE`                     | `K_V × K_I / (1000 × HALL_SENS)` | Converts ADC cross-product mean [mV²] to Watts |

AHT20-specific parameters in `aht20_task.c`:

| Symbol                | Default | Purpose |
|-----------------------|---------|---------|
| `AHT20_SAMPLE_PERIOD_MS` | `2000` | Sensor polling interval [ms] |
| `AHT20_TASK_STACK_SIZE` | `configMINIMAL_STACK_SIZE + 512` | Task stack in words |
| `AHT20_TASK_PRIORITY` | `OS_TASK_PRIORITY_NORMAL - 1` | Must be below GPADC task priority |

---

## Known Limitations

**Inter-channel skew (~209 µs / ~3.76° at 50 Hz):** V and I samples within each pair are not
simultaneous. This introduces a small but consistent phase error in P and PF. Vrms and Irms
are unaffected. The error in Q is amplified when PF ≈ 1 — a 0.6% PF error translates to a
3× error in Q. Skew compensation via linear interpolation is deferred to a future branch.

**No Zero-Crossing Detection:** The 1-second accumulation window is not aligned to complete
mains cycles. The partial-cycle truncation error at each window boundary is < 1% for RMS and
< 1% for P at 50 Hz over 1 second — acceptable at this stage. ZCD is required for Q accuracy
below ~5% and is deferred to the same future branch as skew compensation.

**AHT20 warm-up period:** The shared `g_last_temp_c` and `g_last_hum_percent` variables are
initialised to 0. The temperature and humidity fields in the UART output will read 0 for the
first ~2 seconds after boot until the AHT20 task completes its initial measurement.

**Calibration drift:** K_V and K_I are static constants calibrated at one specific operating
condition. Mains voltage varies ±5–10% depending on grid load and time of day. Re-calibrate
periodically or treat Vrms as a relative measurement unless the reference voltage is stable.
