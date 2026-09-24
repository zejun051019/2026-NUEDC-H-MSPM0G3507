# Changelog

## Unreleased — repository cleanup

- Added project scope, module ownership, hardware, control-path, validation, tuning-history and limitation documentation.
- Extracted foreground key-event debounce consumption and Q4/Q5/Q6 vehicle OLED status rendering into small application modules while leaving key routing, OLED refresh ownership and task order in `main.c`.
- Extracted slow diagnostic cadence selection/publication into `app_telemetry.*`; fast control-frame logging and queue-drain placement remain in the original loop order.
- Added emulator harnesses for the extracted telemetry cadence and key-event debounce helpers; both are pending the final requested test stage.
- Moved Q6 pixel-row admission policy to the Q6 target module and added boundary cases to its host test source.
- Added a focused key-input host test and made the existing Q5/Q6 test runners resolve Arm GCC from an explicit argument, `ARM_GCC_ROOT`, or `PATH` instead of a machine-specific installation path.
- Added a local environment template and ignored machine-specific environment/build/log outputs.
- Replaced the tracked pySerial source copy with a pinned external Python dependency declaration; the local copy is ignored and preserved on disk.
- Pinned the Python packages required by the checked-in ARM-emulator test runners and documented their setup and scope.
- Made the CMake project explicitly C/ASM-only with required C11, matching the checked-in source inventory and avoiding an unused C++ toolchain dependency.
- Excluded bulky per-trial Q4/Q5 tuning transcripts from the publication set while retaining local files and concise evidence-boundary documentation.
- Applied the repository's pinned clang-format 18.1.8 style to all maintained first-party C/H files, including Q3; controller behavior and tuned values were preserved. Vendor files and generated SysConfig output were not reformatted.
- Added a repeatable PowerShell format-check/apply script and documented its scope and tool version.
- Recorded the TI MSPM0 SDK manifest license identifiers and CMSIS-Core SPDX notices in the third-party inventory; no project-level license was selected.
- No tests, ARM build, flash or hardware trial were run during this cleanup.
