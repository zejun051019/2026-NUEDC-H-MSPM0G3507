# Hardware and peripheral ownership

## Target

- MCU: MSPM0G3507 on the project board configuration represented by `empty.syscfg`.
- Main actuator: X42S stepper using the X firmware protocol in `user/code/x42s.*`.
- Vision data is consumed by the MCU over UART3; the vision recognition firmware is outside this repository.
- The chassis uses the project motor/PWM and encoder services; verify physical wiring against the actual board before use.

## Current SysConfig assignment

The following values are read from the checked-in `empty.syscfg` and generated `ti_msp_dl_config.h`, not inferred from historic logs:

| Peripheral | SysConfig instance | Pins | Firmware responsibility |
|---|---|---|---|
| UART0 | `PRINT` | PA0 TX / PA1 RX | X42S actuator protocol transport (`user/code/x42s.*`); SysConfig's legacy instance name is PRINT. |
| UART1 | `DEBUG` | PB6 TX / PB7 RX | Debug/log channel. |
| UART2 | `GYRO` | PA21 TX / PA24 RX | Gyro serial interface. |
| UART3 | `VISION` | PA26 TX / PA25 RX | Vision data link. |
| TIMG6 | `Timer_Control` | internal | Control timebase interrupt. |

The current foreground key routing in `user/main.c` is:

| Button | Action |
|---|---|
| PB21 | Stop active tasks and request the existing stop handling. |
| PB11 | Start Q5. |
| PB24 | Start Q6 when `Q6_TASK_ENABLE=1`; otherwise start the Q2 chassis route. |
| PA28 | Start Q4. |
| PA31 | Q3 zero/reference and start flow; the first press arms the session, and the formal start requires the existing second-press/vision-center condition. |

These are source/SysConfig observations, not a substitute for checking the physical button labels and wiring on the board.

The generated file also configures chassis PWM, GPIO keys, encoders, OLED, clocks, and interrupt routes. Do not change these from documentation alone. `empty.syscfg` remains authoritative; generated macros are evidence of the currently generated mapping.

## Runtime and safety boundaries

- Preserve the current TIMG6-driven tick and main-loop task ordering.
- ISRs should only capture bounded events/data or advance timing state. They must not block on serial transmission or refresh the OLED.
- Keep the X42S protocol family, command framing, units, reply handling, stored-zero sequence and terminal stop behavior intact.
- Hardware validation requires matching board, actuator firmware, wiring, power and safe mechanical support. A successful compile is not evidence of physical correctness.
