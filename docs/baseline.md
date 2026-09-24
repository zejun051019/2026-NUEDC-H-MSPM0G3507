# Source and build baseline

## Authority decision

The project contains two related working copies. The current `s1` branch is based on the Q2–Q5 project state and has no Q6 task path. The isolated Q6 branch adds `user/app/q6`, Q6 target/configuration logic and Q6 mode integration. Its source and build metadata match the user's accepted Q6 run records. Therefore this cleanup is based on the Q6-integrated source snapshot, not on the `s1` directory name alone.

This repository covers the MCU control paths Q2–Q6. Q1 is the external vision-recognition task and is intentionally out of scope; only the MCU-side UART3 interface is included.

## Snapshot identity

- Source baseline branch: `q6-isolated-baseline`.
- Parent commit: `6640813c8b907b38c7611fd83320b1dec8ef8809`.
- At snapshot time, eight tracked Q6/application files contained uncommitted user changes. The isolated cleanup worktree was initialized from that complete working snapshot, not from HEAD alone.
- Byte-level SHA-256 values for those eight files are recorded in [`baseline-source.sha256`](baseline-source.sha256); the unchanged tracked remainder is identified by the parent commit above.
- This cleanup branch is separate from both original working directories. No commit or push has been made.
- The 2026-09-24 audit rebuilt the Debug ARM image and ran the checked-in host regression runners. This confirms build/test status only; no flash or physical trial was performed during the audit.

## Toolchain and configuration

- Device/SysConfig metadata: MSPM0G3507, LQFP-64 (PM), MSPM0 SDK `2.10.00.04`, SysConfig `1.27.1+4634`.
- Previously observed compiler: GNU Arm Embedded `10.3-2021.10`, GCC `10.3.1 (20210824)`.
- C standard: C11. Build system: CMake 3.22+ and Ninja, using the checked-in presets.
- `empty.syscfg` is the configuration source. Generated `ti_msp_dl_config.c/.h` are kept as checked-in generated output and are not hand-edited.

## Firmware variants observed in local CMake caches

| Variant | Relevant compile definitions | Intended use |
|---|---|---|
| `build_drive` | `Q6_TASK_ENABLE=1U`, `Q6_STATIC_TEST_MODE=0U`, `Q6_BENCH_AUTO_START=0U`, `Q6_BOOT_AUTO_SELECT=1U`, `Q6_BENCH_EXPLICIT_TARGET_MM=-32768` | Dynamic/competition-style Q6 image with capture target and PB24 start flow. This is cache evidence, not proof of a fresh build. |
| `build_static` | `Q6_STATIC_TEST_MODE=1U`, `Q6_BENCH_AUTO_START=0U`, `Q6_BOOT_AUTO_SELECT=1U`, `Q6_BENCH_EXPLICIT_TARGET_MM=-32768` | Static bench variant; never describe it as the competition image. |
| `build_vehicle_ninja` | `Q6_STATIC_TEST_MODE=0U`, `Q6_BENCH_AUTO_START=1U`, `Q6_BOOT_AUTO_SELECT=1U`, `Q6_BENCH_EXPLICIT_TARGET_MM=-32768` | Automatic-start experiment variant; verify before flashing. |

The Q6 header defaults enable the task but disable static bench mode, auto-start and boot auto-selection. CMake cache flags override these defaults; inspect the exact cache/compile command before any release build.

PB24 is compile-time multiplexed: with `Q6_TASK_ENABLE=1` it requests Q6; in the alternate image with Q6 disabled it starts the existing Q2 route. The source tree therefore contains both task paths, while a single PB24 mapping does not expose both simultaneously.

## Historical resource report

A prior build report recorded 90,832 bytes of text, 7,712 bytes of data and 9,936 bytes of BSS for `build_drive`; this corresponds to about 98,544 bytes of text+data in flash and 17,648 bytes of data+BSS in RAM. These are historical measurements from that local build, not measurements of the current formatted worktree. Stack/heap margin and the current map must be rechecked in the user's final validation stage.

## Verification status for this cleanup

- User-reported hardware result: Q5 and Q6 have been accepted. The exact accepted firmware image has not been tied to this source snapshot by a recorded commit/hash.
- Host regression on 2026-09-24: app-key 9 checks, telemetry 14, Q5 5,664, Q6 target 235, and Q6 static-image 39; all reported zero failures.
- ARM Debug build on 2026-09-24: passed; Flash 100,968 / 131,072 bytes, SRAM 17,672 / 32,768 bytes.
- No flash or physical trial was performed during this audit. Historical logs and snapshots remain in the protected local archive; public repository tracking should contain only selected, rights-cleared evidence.
