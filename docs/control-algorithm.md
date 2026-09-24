# Control paths

This page describes the current code structure at a high level. It is not a replacement for the task statement or a claim that each path was revalidated by this cleanup.

## Shared data path

```text
Vision UART bytes
  -> vision frame parser / freshness and validity checks
  -> ball state estimate and safety checks
  -> selected task policy (Q3 sequence or Q4/Q5/Q6 vehicle controller)
  -> pendulum geometry mapping / actuator command limits
  -> X42S command and feedback
```

The vision module is external. The MCU-side parser and control-boundary acceptance logic live in `user/code/vision_link.*` and the consuming application modules.

## Question paths in this firmware

- **Q2:** route/lap exercise through the existing task selection and `user/bsp/lap_trace.*` support.
- **Q3:** ball position estimation and control sequence in `user/app/q3/`. Its gains, calibration and sequencing are behavior-protected; formatting-only edits are in scope.
- **Q4:** shared vehicle state machine applies Q4 route, stop and ball-recovery policy.
- **Q5:** `Q45Vehicle_UpdateBallControl` selects the Q5-owned profile and gains, applies the route-shaped feedforward from `user/app/q4/q5_vehicle_model.*`, and adds the bounded first-/second-bend feedback and late-straight trim. `q5_dynamic_ball_control.*` is a standalone controller module; it is not called by the current vehicle-task path.
- **Q6:** `user/app/q6/q6_target.*` captures a stable current ball position or accepts an explicit target; the shared vehicle state machine then runs target-relative closed-loop control with Q6-gated adaptations.

## Q6 target capture

Capture accepts fresh, confident, low-speed frames within configured position bounds. It collects a time-spanning window, rejects gaps and excessive position spread, then locks a candidate target. The normal competition path uses the key-driven start flow; bench-only auto-selection, explicit-target and static-test macros are disabled by default in `q6_config.h` unless a build overrides them.

Q6 still shares the Q4/Q5 state machine and actuator interface. “Q6 module” therefore means a narrow target/configuration adapter plus Q6-gated branches, not a fully separate controller implementation.

The Q6 pixel-row admission predicate is owned by `q6_target.*`. The shared Q45 controller invokes it only in Q6 mode before a visual sample reaches readiness or the ball estimator; Q4/Q5 retain their existing frame checks without this Q6-only restriction. It consumes the existing `pixel_cy` field and does not alter the vision protocol or recognition algorithm.
