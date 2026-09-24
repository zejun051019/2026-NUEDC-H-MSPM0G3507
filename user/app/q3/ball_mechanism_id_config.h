#ifndef BALL_MECHANISM_ID_CONFIG_H
#define BALL_MECHANISM_ID_CONFIG_H

/* 2026-09-16 bench identification result, positive direction:
 *   - 14 deg: ball first overcame static friction;
 *   - 25 deg: maximum mechanically safe observed angle.
 * Keep this at the measured boundary only for manual identification.  The
 * later closed loop must use a smaller operating limit with safety margin. */
#define BALL_ID_STATIC_FRICTION_BREAKAWAY_DEG 14.0f
#define BALL_ID_MEASURED_SAFE_LIMIT_DEG 25.0f

/* PA31 advances one absolute angle point at a time. */
#define BALL_ID_OPERATOR_CEILING_DEG BALL_ID_MEASURED_SAFE_LIMIT_DEG
#define BALL_ID_FIRST_ANGLE_DEG 1.0f
#define BALL_ID_ANGLE_STEP_DEG 1.0f

#define BALL_ID_START_POSITION_CM 5.0f
#define BALL_ID_START_TOLERANCE_CM 0.5f
#define BALL_ID_MOVED_THRESHOLD_CM 0.4f
#define BALL_ID_HOLD_MS 700U

#define BALL_ID_ENABLE_SETTLE_MS 300U
#define BALL_ID_COMMAND_GAP_MS 100U
#define BALL_ID_RETURN_SETTLE_MS 200U
#define BALL_ID_VISION_TIMEOUT_MS 250U

#define BALL_ID_MAX_SPEED_RPM 2.0f
#define BALL_ID_ACCEL_RPM_S 30U
#define BALL_ID_DECEL_RPM_S 30U
#define BALL_ID_MAX_CURRENT_MA 600U

#endif
