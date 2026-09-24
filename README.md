# 2026 NUEDC H — MSPM0G3507 Ball-Balancing Control Firmware

2026 年全国大学生电子设计竞赛 H 题滚球控制系统的 MCU 端固件。仓库整理范围是控制端 Q2–Q6：底盘行程、球位置控制、静态恢复、动态车球控制和任意起点目标保持。第 1 问属于独立视觉侧；本仓库只包含 MCU 侧 UART3 接收与视觉帧有效性处理，不包含视觉模块算法/固件。

This repository contains the MCU-side control firmware for the 2026 NUEDC H ball-balancing project. It covers the implemented Q2–Q6 control paths. The independent Q1 vision algorithm/firmware is out of scope; the MCU-side UART3 interface and frame validation are included.

## Project at a glance

- MCU: TI MSPM0G3507, bare-metal cooperative main loop and timer-driven control tick.
- Actuator: X42S stepper using the X firmware protocol; chassis drive and pendulum actuation are separate software paths.
- Application layers: `user/app/q3`, `user/app/q4`, `user/app/q6`; foreground key, display and telemetry adapters in `user/app`; board services in `user/bsp`; device/protocol code in `user/code`.
- `main.c` remains the composition root for startup, key-to-mode routing, and the original fast-path order. ISR-event debounce, Q4/Q5/Q6 status rendering, and periodic diagnostic cadence are isolated in `user/app/app_key_input.*`, `user/app/app_status_display.*`, and `user/app/app_telemetry.*`.
- Configuration source: `empty.syscfg`; generated `ti_msp_dl_config.c/.h` are checked in for the current project setup and must not be hand-edited.
- Build: CMake + Ninja + Arm GNU toolchain, with Debug and Release presets.

## Capabilities in this firmware

| Area | Code location | Scope |
|---|---|---|
| Q2 | `user/bsp/lap_trace.*`, `user/main.c` | Chassis lap/route exercise selected through the existing key path. |
| Q3 | `user/app/q3/` | Ball-state estimation, position control, pendulum geometry/calibration, and the Q3 sequence. The control behavior and tuned values are preserved; source formatting is normalized. |
| Q4 | `user/app/q4/q45_vehicle_control.*` | Static vehicle route and ball recovery/hold behavior. |
| Q5 | `user/app/q4/q5_*`, `q45_vehicle_control.*` | Dynamic vehicle/ball control and motion-derived feedforward. |
| Q6 | `user/app/q6/`, `q45_vehicle_control.*` | Capture the ball's stable starting position or accept an explicit target, then perform target-relative closed-loop control. |
| Hardware/protocol | `user/code/`, `user/bsp/` | X42S, vision UART framing, keys, display, motor/encoder/gyro, clocks, and debug telemetry. |

The Q4/Q5/Q6 vehicle controller shares a state machine. Q6-specific target capture and tuning are guarded in the Q6 path; this repository intentionally does not advertise a wholesale redesign of the validated Q3–Q5 control laws.

The Q2 route and Q6 start currently share the PB24 input behind `Q6_TASK_ENABLE`: when Q6 is enabled PB24 requests Q6; when disabled the existing standalone Q2 path uses PB24. The codebase contains both paths, but this macro means they are not both selected by that key in the same configuration.

Current source key map: PB21 stop, PB11 Q5, PB24 Q6/Q2 (compile-time selection), PA28 Q4, and PA31 Q3 zero/start flow. See [hardware notes](docs/hardware.md) before operating the board.

## Build prerequisites

Install CMake 3.22 or newer, Ninja, and the Arm GNU toolchain. The current CMake presets are Windows-oriented. Set `ARM_GCC_ROOT` and `NINJA_ROOT` in the shell before configuring; the VS Code SysConfig helper also reads `MSPM0_SDK_ROOT` and `SYSCONFIG_ROOT` from a local `.env` file. Copy `.env.example` as a variable-name template; `.env` is ignored and must never be committed.

The optional serial capture utilities under `tools/` use pySerial. Install their pinned dependency with `python -m pip install -r tools/requirements.txt`; no vendored pySerial copy is part of the public source tree.

From a shell where those variables are available:

```powershell
$env:ARM_GCC_ROOT = "C:\Toolchains\arm-gnu-toolchain"
$env:NINJA_ROOT = "C:\Tools\ninja"
cmake --preset debug-armgcc
cmake --build --preset build-debug
```

The project is Windows-oriented in its current CMake toolchain file. The commands above describe the checked-in build entrypoint. Q6 bench macros are distinct from the normal run configuration. Review `user/app/q6/q6_config.h` and the CMake cache before producing any image intended for hardware.

For the software regression harnesses, install the pinned Python emulator dependencies with `python -m pip install -r tests/requirements.txt --target build/testdeps`, then run the PowerShell runners listed in [testing and evidence](docs/testing.md). This does not exercise physical sensors, motors, wiring, or real-time timing.

## Hardware and safety

The current SysConfig assigns UART0 to `PRINT` (PA0/PA1), UART1 to `DEBUG` (PB6/PB7), UART2 to `GYRO` (PA21/PA24), UART3 to `VISION` (PA25/PA26), and TIMG6 to `Timer_Control`. Consult [hardware notes](docs/hardware.md) and the board wiring before connecting external devices; generated names and actual physical wiring must agree. Do not infer pin responsibilities from old logs.

This firmware can move motors immediately after a start request. Keep the chassis mechanically supported, the ball path clear, and a physical power cutoff accessible. Never flash a bench auto-start image as a competition image.

## Validation status

The user reports Q5 and Q6 acceptance on hardware. In this audit, the four checked-in host runners passed (key input: 9 checks; telemetry: 14; Q5: 5,664; Q6 target and static-image harnesses: 235 and 39), and the Debug ARM image built successfully (Flash 100,968 / 131,072 bytes; SRAM 17,672 / 32,768 bytes). The exact accepted device image has not been linked to this source snapshot, and no flash or physical trial was performed during this audit. See [testing and evidence](docs/testing.md) for scope and caveats.

## Documentation

- [Architecture and module ownership](docs/architecture.md)
- [Source and build baseline](docs/baseline.md)
- [Hardware and peripheral ownership](docs/hardware.md)
- [Control paths](docs/control-algorithm.md)
- [Validation and evidence policy](docs/testing.md)
- [Code style and formatting](docs/code-style.md)
- [Debugging case studies](docs/debugging-case-studies.md)
- [Known limitations](docs/known-limitations.md)
- [Third-party notices](THIRD_PARTY_NOTICES.md)

## Resume-ready project summary

Implemented and tuned a bare-metal MSPM0G3507 ball-balancing control system for the 2026 NUEDC H problem, integrating UART-based vision feedback, X42S stepper control, state estimation, target capture, and Q2–Q6 task flows. Specific performance claims should be added only with the corresponding dated logs and hardware conditions.
