# Known limitations

- This repository does not contain the independent Q1 vision algorithm/firmware. It contains only the MCU-side UART3 data consumer and control-side frame validation.
- Q4/Q5/Q6 share `q45_vehicle_control.c`; some Q6 adaptations are mode-gated branches rather than a wholly independent state machine.
- Hardware results are condition-dependent. The user's Q6 acceptance statement is not reproduced by this cleanup; exploratory tuning is not represented as a validated result.
- Current CMake presets are Windows-oriented and use local environment variables for tool paths. A clean checkout still needs the correct Arm GNU, CMake/Ninja and TI SysConfig/SDK setup.
- Third-party SDK/DriverLib content is present in the tree. Preserve its original notices and audit the exact redistributable subset before publishing a public release; see `THIRD_PARTY_NOTICES.md`.
- No top-level project license has been selected. Confirm team ownership and intended reuse terms before adding one; do not imply that the vendor SDK license covers the contest application code.
- The current pass is formatting/documentation/repository hygiene only. No tests, build, flash, or hardware validation were run.
