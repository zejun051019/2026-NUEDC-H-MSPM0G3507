#ifndef BALL_PULSE_ID_CONFIG_H
#define BALL_PULSE_ID_CONFIG_H

/* Negative-direction breakaway experiment: 0 -> -6.5 -> 0.
 * Negative pendulum angle should move the ball toward visual plus. */
#define BALL_PULSE_TARGET_ANGLE_DEG -7.0f
#define BALL_PULSE_MAX_SPEED_RPM 8.0f
#define BALL_PULSE_ACCEL_RPM_S 100U
#define BALL_PULSE_DECEL_RPM_S 100U
#define BALL_PULSE_MAX_CURRENT_MA 600U

/* The pulse test still uses a conservative time boundary.  Actual motor
 * position is now logged independently and will replace this estimate in
 * the formal Q3 controller after the feedback chain is hardware-verified. */
#define BALL_PULSE_RISE_ESTIMATE_MS 190U
/* Return immediately after the estimated -7 deg arrival: this is a
 * dynamic-inertia test, not a gravity-hold test. */
#define BALL_PULSE_HOLD_MS 0U
#define BALL_PULSE_RETURN_ESTIMATE_MS 500U
#define BALL_PULSE_RESULT_SETTLE_MS 500U

#define BALL_PULSE_ENABLE_SETTLE_MS 300U
#define BALL_PULSE_COMMAND_GAP_MS 100U
#define BALL_PULSE_ZERO_SETTLE_MS 500U
#define BALL_PULSE_VISION_TIMEOUT_MS 250U
#define BALL_PULSE_START_TOLERANCE_CM 1.0f
#define BALL_PULSE_SAFETY_POSITION_CM 6.0f

#endif
