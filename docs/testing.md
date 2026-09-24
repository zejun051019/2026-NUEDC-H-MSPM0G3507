# Validation and evidence

## Evidence categories

Keep these claims separate:

1. **Source inspection:** a path or guard exists in code.
2. **Host test:** a selected pure/stateful unit test passes with its stubs.
3. **ARM build:** the configured cross compiler links an image.
4. **Flash verification:** the programmer reports successful verification for the exact image.
5. **Hardware trial:** the board runs under recorded start conditions and logs support the result.
6. **Competition acceptance:** user-confirmed or competition evidence establishes the acceptance outcome.

Passing an earlier level does not imply a later one. Historical test files are not proof of complete coverage.

## Existing checks

The checked-in PowerShell runners are `tests/run_app_key_input_test.ps1`, `tests/run_app_telemetry_test.ps1`, `tests/run_q5_checks.ps1`, and `tests/run_q6_checks.ps1`. They compile selected C harnesses with Arm GCC and execute them in the Unicorn ARM emulator through `tests/run_q5_test.py`. The Q5/Q6 harnesses reuse selected Q3 implementation files from `BaselineRoot`; inspect the exact source lists and stubs before interpreting coverage. `tests/ball_control_test.c` and `tests/x42s_parser_test.c` are additional harness sources, but are not currently included by those four runners. The key-input harness covers event clearing, debounce timing and millisecond wraparound. The telemetry harness covers default/Q6 record cadence and return from Q6-running to normal cadence. These host-side cases do not prove full firmware behavior.

To prepare the Python side of the runners in a clean checkout, install the pinned emulator dependencies into the ignored build directory:

```powershell
python -m pip install -r tests/requirements.txt --target build/testdeps
```

With `ARM_GCC_ROOT` set, run the four PowerShell scripts individually. The C harnesses compile to ignored `.elf` files under `tests/`; no hardware connection is required. On 2026-09-24, all four runners passed: key input 9 checks, telemetry 14, Q5 5,664, Q6 target 235, and Q6 static-image 39, with zero failures. An ARM Debug build also passed (Flash 100,968 / 131,072 bytes; SRAM 17,672 / 32,768 bytes). These checks do not replace flash or hardware testing.

## Bench and competition variants

`user/app/q6/q6_config.h` defines defaults for Q6 task selection, static bench mode, automatic start, boot auto-selection and explicit target. These are distinct firmware variants. A bench image can start/move without the normal key sequence; verify the CMake cache and preprocessor definitions before flashing. Never treat a static bench binary as the competition image.

## Hardware test record template

For each future trial, record:

- source branch and commit, dirty-file list, image hash and build macro set;
- board, actuator firmware, wiring, supply and mechanical support;
- starting ball position/state, selected task, key/start sequence and target;
- test duration and conditions;
- logs, stop reason, tracking error, peak actuator command, vision-frame gaps and faults;
- whether the result is user-reported, log-supported, or independently repeated.

## Current status

The user reports Q5 and Q6 acceptance. This audit did not flash or reproduce either result, and the exact accepted binary has not been linked to a source commit/hash. Historical exploratory work is not promoted to a validated baseline without corresponding evidence.
