#ifndef PENDULUM_GEOMETRY_CALIBRATION_CONFIG_H
#define PENDULUM_GEOMETRY_CALIBRATION_CONFIG_H

/* Manual geometry calibration: command angle -> measured right-end height.
 * The measured +25 deg boundary is deliberately not used as a routine test
 * target.  Keep 5 deg margin while a ruler is being positioned. */
#define PENDULUM_GEO_FIRST_ANGLE_DEG 0.0f
#define PENDULUM_GEO_ANGLE_STEP_DEG 2.0f
#define PENDULUM_GEO_MAX_TEST_ANGLE_DEG 20.0f

#define PENDULUM_GEO_ENABLE_SETTLE_MS 300U
#define PENDULUM_GEO_COMMAND_GAP_MS 100U
#define PENDULUM_GEO_ZERO_SETTLE_MS 500U

#define PENDULUM_GEO_MAX_SPEED_RPM 2.0f
#define PENDULUM_GEO_ACCEL_RPM_S 30U
#define PENDULUM_GEO_DECEL_RPM_S 30U
#define PENDULUM_GEO_MAX_CURRENT_MA 600U

#endif
