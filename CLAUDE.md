# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

6-channel temperature / force control for shape-memory-alloy (SMA) fabric actuators. Two pieces:

- **`Core/`** — STM32G474CEUx firmware (STM32CubeIDE project, no build artifacts in repo). Runs PID temperature control on 6 thermocouple+PWM channels, with optional force PID on one channel via an I2C loadcell.
- **`SMA_control/`** — Python PC tools that talk to the MCU over UART: PyQt5 GUI (`main.py` + `window.ui`), terminal CLI (`ctrl.py`), and standalone test scripts (`diag.py`, `Force test.py`).

The link between the two is the UART protocol described below — almost any change touches both sides.

## Build / run

**Firmware.** Open `Core/` in STM32CubeIDE; the IDE-generated `.ioc`/`.cproject` are NOT committed, so cloning will not produce a buildable project. Target MCU: `STM32G474CEUx` (see `Core/Startup/startup_stm32g474ceux.s`). Flash via ST-Link.

**PC GUI / scripts.** No `requirements.txt`. Dependencies: `PyQt5`, `pyqtgraph`, `pyserial`.

```
cd SMA_control
python main.py        # GUI (loads window.ui from CWD — must be run from SMA_control/)
python ctrl.py        # terminal control
python diag.py        # force-control / I2C test sequence
python "Force test.py"
```

The serial port is hard-coded near the top of each Python file (currently `COM6`, 115200 8N1). Update all four if you change ports. `main.py` also has `DISPLAY_CH` at the top to pick which of the 6 channels the GUI shows.

## Architecture

### Firmware control loop (Core/Src/main.c)

Two TIM ISRs drive everything; the `while(1)` in `main()` is empty.

- **TIM4 ISR** — sensor + telemetry tick. Calls `TMC_Scan()` (SPI2+DMA scan of all 6 MAX31855 thermocouples), runs `Check_Temperature_Sensor()` per channel, fills `system.buf_fdcan_tx` (despite the name, used for UART now), and calls `UartComm_SendState()`. Also sends `UartComm_SendForceState()` when in force mode.
- **TIM5 ISR** — control tick. Per channel: if force mode + active channel → read I2C loadcell, run `ForceControl_Calculate()`, write PWM. Else → `Check_Safety_Temperature()`, then `Calculate_Ctrl()` (PID) → `Set_PWM_Output()` → `Control_Fan_By_Temperature()`.

NVIC priorities are explicitly set: TIM4 sub-prio 0 (highest), TIM5 sub-prio 1. The `pid.startup_phase` flag is held high for ~10 TIM5 ticks (~1 s) to suppress integral kick at boot.

PWM outputs are split across two timers and accessed by direct CCR pointer — `system.pnt_pwm[i]` is initialized in `main()` to point at `TIM2->CCRx` / `TIM3->CCRx` so the control loop can write duty cycles without going through HAL. Mapping is in `Core/Inc/system_defs.h` (`PWMn_TIM`, `PWMn_TIM_CH`, `PWMn_TIM_CCR`).

`g_sys.ctrl_loop_period_ms` is intentionally computed at runtime from TIM register values rather than hardcoded — see the comment block in `system_defs.h`. If you change TIM4/TIM5 prescaler/period, the period value flows automatically.

### Safety state machine (Core/Src/temp_control.c)

Per-channel `safety_mode` field with hysteresis:

| Mode | Trigger | PWM | Fan | PID |
|------|---------|-----|-----|-----|
| 0 normal | — | normal | temp-controlled | active |
| 1 | T ≥ 80 °C | normal | forced ON | active |
| 2 | T ≥ 120 °C | forced 0 | forced ON | disabled |
| 3 sensor error | T > 200 or T < −50 | forced 0 | forced ON | disabled |

Recovery: mode 1 → 0 at < 75 °C, mode 2 → 0 at ≤ 30 °C. The `fan[i]` byte sent in state frames doubles as a status indicator: `0`=off, `1`=on, `17`=safety-1, `49`=sensor-error — the PC distinguishes safety state from this byte, not from a separate field.

### UART protocol (Core/Src/uart_protocol.c ↔ SMA_control/uart_driver.py)

Frame: `[STX=0xAA] [CMD] [LEN] [DATA × LEN] [CRC8]`. CRC-8 polynomial 0x07, init 0x00, computed over `CMD+LEN+DATA`. State machine in `UartComm_ProcessRxByte`.

| CMD | Dir | LEN | Payload |
|-----|-----|-----|---------|
| 0x01 Control | PC→MCU | 30 | `pwm[6] + fan[6] + enable_pid[6] + target_temp[6×u16]` (target in 0.25 °C/LSB) |
| 0x02 State | MCU→PC | 24 | `pwm[6] + fan[6] + temp[6×u16 raw]` |
| 0x03 Gain | PC→MCU | 13 | `float kp, ki, kd; u8 channel` |
| 0x04 Force | PC→MCU | 6 | `u8 channel, enable; float target_force` |
| 0x05 Force State | MCU→PC | 10 | `u8 mode, channel; float force, disp` |
| 0x06 I2C Test | PC→MCU | 0 | (triggers an MCU printf I2C diagnostic) |

**The same UART line carries MCU `printf` debug text.** PC parsers MUST treat any byte arriving outside a valid frame (i.e. not 0xAA where STX is expected) as plain debug output and surface it separately. `uart_driver.py` does this in `UartWorker.run()` — when modifying the parser, preserve the IDLE-state debug-byte buffering.

`CTRL_CH = 6` must stay synchronized between `Core/Inc/system_defs.h`, `SMA_control/const.py`, and `SMA_control/uart_driver.py`.

### Legacy FDCAN

The PC link migrated from FDCAN3 to UART. `fdcan.c/h` and the CAN IDs in `system_defs.h` (`CAN3_RXID_*`, `CAN3_TXID_*` = 0x400/0x401/0x402) and `MX_FDCAN3_Init()` are still compiled in but no longer carry the PC link — they correspond 1:1 to the new CMD codes. The struct names `Buf_FDCAN_Tx_*` / `buf_fdcan_pid_tuning` in `main.h` and `temp_control.h` are legacy names for what are now UART payload buffers.

### PID parameter naming

`PID_Param_TypeDef` in `temp_control.h` stores gains as `lambda` (kp), `alpha` (ki), `gain` (kd). The CMD 0x03 payload uses the more conventional `kp/ki/kd` names. The mapping happens inside the gain-update handler — do not assume the field names match across files.

### PC GUI flow

`MainWindow` (in `main.py`) loads `window.ui` from the current working directory, owns a `UartWorker(QThread)`, and connects four signals: `data_received`, `force_received`, `debug_message`, `error_occurred`. `tx_timer` (100 ms) sends control heartbeats; `plot_timer` (50 ms) refreshes the pyqtgraph plots. Three modes: `MODE_MANUAL`, `MODE_TEMP`, `MODE_FORCE`. CSV logs land in `SMA_control/data_logs/` (auto-created).

## Conventions

- Korean comments are used throughout firmware and the longer Python files. Match the surrounding language when editing.
- `Core/Src/readme.txt` is the canonical Korean spec for the protocol and safety modes — when those change, update it in lockstep with the code.
- Most state lives in two large globals — `system` (in `main.c`) and `pid` (in `temp_control.c`), with `force_ctrl` for the force path. New control state should slot into one of these rather than introducing more globals.
