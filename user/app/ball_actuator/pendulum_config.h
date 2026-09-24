#ifndef PENDULUM_CONFIG_H
#define PENDULUM_CONFIG_H

/* Centralized, deliberately conservative values for the first no-ball test.
 * Do not change more than one value between physical trials. */
#define PENDULUM_CAL_TEST_ANGLE_DEG 1.0f
#define PENDULUM_CAL_MAX_SPEED_RPM 2.0f
#define PENDULUM_CAL_ACCEL_RPM_S 30U
#define PENDULUM_CAL_DECEL_RPM_S 30U
#define PENDULUM_CAL_MAX_CURRENT_MA 600U

#define PENDULUM_CAL_ENABLE_SETTLE_MS 300U
#define PENDULUM_CAL_COMMAND_GAP_MS 100U
#define PENDULUM_CAL_POSITION_HOLD_MS 700U
#define PENDULUM_CAL_TOTAL_TIMEOUT_MS 8000U

#endif
