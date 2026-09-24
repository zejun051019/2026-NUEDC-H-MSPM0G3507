# Architecture and module ownership

## Scope

This is the MCU control firmware for the 2026 H problem. It contains Q2–Q6 paths plus the MCU-side vision serial consumer. The visual recognition firmware for Q1 is external and is not represented by `user/code/vision_link.*`.

## Runtime flow

```text
Startup: SysConfig -> BSP/devices -> application modules
  |
  +-- foreground loop (main.c composition root)
       1. capture current tick -> AppTelemetry_BeginCycle snapshots diagnostic cadence
       2. VisionLink_Poll -> BallActuator_Task
       3. BallPositionControl_Task -> Q3Sequence_Task
       4. Q45Vehicle_Task (Q4/Q5/Q6 shared vehicle state machine)
       5. DebugLog_PublishControlFrame
       6. calibration/identification tasks -> DebugLog_Task
       7. AppTelemetry_PublishDue (slow periodic records)
       8. consume key events / coordinate mutually exclusive modes
       9. process stop/start-pending conditions -> low-rate OLED update

Q45Vehicle_Task
  +-- Q4 route and static recovery policy
  +-- Q5 motion model / dynamic ball controller
  +-- Q6 target capture and mode-gated target-relative behavior
  +-- pendulum geometry -> BallActuator / X42S

TIMG6 ISR -> timebase and bounded chassis tick bookkeeping.
UART/GPIO ISRs -> receive bytes or set event flags; foreground code parses,
controls, formats logs and refreshes the display.
```

The diagram summarizes the source-level sequence in `user/main.c`; it is not a cycle-accurate timing diagram. Preserve this order when changing the scheduler, especially the vision/actuator/control fast path and the telemetry cadence snapshot at the beginning of each loop.

## Ownership

| Layer | Owns | Must not take ownership of |
|---|---|---|
| `user/main.c` | Startup, task selection, key coordination, top-level polling, task ordering, mode routing, and display refresh integration. | Controller-private state or peripheral register setup beyond existing startup. |
| `user/app/app_key_input.*` | Foreground consumption of ISR-recorded GPIO key events and the existing 15 ms event debounce. | Key-to-task mapping, ISR event production, or control-state transitions. |
| `user/app/app_status_display.*` | Foreground rendering of the Q4/Q5/Q6 vehicle-status body from a caller-provided status snapshot. | Reading or changing controller state, refreshing the OLED, or formatting in an ISR. |
| `user/app/app_telemetry.*` | Period selection and foreground cadence for existing diagnostic records across application modules. | Wire-format serialization, direct UART transmission, or control decisions. |
| `user/app/q3/` | Q3 estimator, controller, geometry calibration and sequence. | Q4/Q5/Q6 tuning and UART/device drivers. |
| `user/app/q4/` | Shared Q4/Q5/Q6 vehicle state machine, dynamic Q5 model/controller, Q4 policy/configuration. | Vision module firmware or SysConfig-generated content. |
| `user/app/q6/` | Q6 target selection/capture, Q6 visual-row admission policy, and Q6-only parameters. | Q3 behavior/configuration or Q4/Q5 parameter ownership. |
| `user/app/ball_actuator/` | Ball pendulum actuator abstraction and calibration. | Chassis route selection. |
| `user/bsp/` | Clock, interrupt glue, low-level chassis control, PID primitive and route trace. | Contest-level target policy. |
| `user/code/` | Device drivers, UART links, X42S protocol and debug record serialization. | Competition task state machines. |
| TI generated/vendor files | Device initialization and vendor APIs. | Application-level hand edits. |

## Change boundaries

- Q3 control behavior, configuration values, calibration and sequencing are protected. Formatting and explanatory documentation may be improved; functional edits require a demonstrated compatibility need and separate regression evidence.
- Keep Q4/Q5 tuning and shared control behavior unchanged while documenting or integrating Q6.
- Treat `empty.syscfg` as the editable hardware configuration source. Regenerate generated files only through the matching SysConfig version and an explicit review.
- Keep ISR work bounded and nonblocking. Do not move formatting, floating-point control calculations, OLED updates, or blocking serial output into an ISR.
- Keep `main.c` as the composition root for startup, task selection, and loop ordering. Small app adapters may own isolated input consumption, status-body rendering, or periodic telemetry cadence; preserve the caller's key routing, display clear/draw/refresh sequence, and diagnostic publication order.
- Keep key-to-task routing in `main.c`; `app_key_input.*` only consumes the existing ISR flag and preserves the existing debounce interval and timestamp state.
- `app_telemetry.*` owns only periodic log cadence and due-record publication. The fast control-frame publication and queue-drain placement remain in the main loop, and telemetry cadence is selected before vision/control work as before.
- Keep Q6-only vision admission policy in `user/app/q6/`; the shared Q45 controller decides when that policy applies, while Q4/Q5 frame acceptance remains unchanged.
- Prefer the existing C interfaces and fixed-size data structures. No RTOS, dynamic allocation, or new framework is needed for this project.
