# FxLMS Active Noise Cancellation — Zynq-7000 FPGA

Implementing a **Filtered-x Least Mean Squares (FxLMS)** Active Noise Cancellation system on a Xilinx Zynq-7000 SoC, with **CV-FxLMS** (Cosine Variable step-size FxLMS) extensions for improved convergence performance.

## System Overview

```
Mic 0 (ref) ──► x(n) ──► W(z) ──► y(n) ──► Speaker
                │                              │
                └──► S'(z) ──► x'(n)          │ (acoustic path)
                                  │            ▼
Mic 1 (err) ──────────────────── e(n) ◄──── room
                                  │
                              LMS update ──► W(z) coeffs
```

- **PL (FPGA fabric)**: FxLMS algorithm, I2S/I2C audio interface, AXI master
- **PS (ARM Cortex-A9)**: Data readback via DDR (AXI HP port)
- **Audio**: Dual WM8731 codecs, 48 kHz sample rate

## Key Modules

| Module                | File                            | Role                                        |
| --------------------- | ------------------------------- | ------------------------------------------- |
| FxLMS state machine   | `new/fxlms_core.v`            | W(z) / S'(z) filtering + LMS updates        |
| Top-level (hardware)  | `IO/WM8731_dual_mic.v`        | Dual-mic ANC with real audio codecs         |
| FIR filter (W)        | `new/fir_filter.v`            | Control filter, 128 taps                    |
| Secondary path filter | `new/secondary_path_filter.v` | S'(z), 256 taps, writable coefficients      |
| LMS update            | `new/lms_update.v`            | W(z) coefficient adaptation                 |
| CV-FxLMS (time)       | `new/cosine_annealing_mu.v`   | Cosine-annealing µ scheduler (time-driven) |
| CV-FxLMS (error)      | `new/error_cosine_mu.v`       | Error-driven cosine µ scheduler            |
| AXI master            | `imports/aq_axi_master.v`     | 64-bit burst read/write to DDR              |
| Global parameters     | `new/fxlms_params.vh`         | All widths, orders, µ values, init modes   |

## Operating Modes

| `estimate_mode` | `adapt_enable` | Behaviour                                                      |
| :---------------: | :--------------: | -------------------------------------------------------------- |
|         0         |        0        | Frozen filter — W(z) fixed                                    |
|         0         |        1        | Normal FxLMS — adapts W(z) with error signal                  |
|         1         |        —        | Secondary path estimation — injects excitation, updates S'(z) |

**State flow (normal):** `IDLE → FILTER_W → FILTER_S → UPDATE_W → DONE`
**State flow (estimation):** `IDLE → EST_FILTER_S → EST_UPDATE_S → DONE`

## Button Mapping

| Button    | Pin | Function                         |
| --------- | --- | -------------------------------- |
| PS button | B13 | Toggle W(z) adaptation           |
| PL button | N15 | Toggle secondary path estimation |
| —        | —  | Reset W(z) weights to zero       |

All buttons debounced at 20 ms / 50 MHz in `fxlms_top.v`.

## Simulation

```
# Include path must resolve fxlms_params.vh and tb_secondary_path_cfg.vh
sim_1/new/fxlms_tb.v          # Core FxLMS unit test
sim_1/new/tb_dual_mic_fxlms.v # Full dual-mic system test
```

- Simulation clock: 50 MHz (20 ns period)
- `sample_valid` driven at 48 kHz audio rate
- Secondary path modelled as exponential-decay FIR in `true_secondary_path_filter.v`

## Related Repository

The Vitis/PS-side software companion for this project is maintained at:
[erchen-17/AXI_for_AX7020_Vitis](https://github.com/erchen-17/AXI_for_AX7020_Vitis)
