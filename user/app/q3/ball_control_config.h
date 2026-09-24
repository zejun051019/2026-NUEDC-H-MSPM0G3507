#ifndef BALL_CONTROL_CONFIG_H
#define BALL_CONTROL_CONFIG_H

/* Q3 generic outer controller.  Q3Sequence supplies +5 cm and -5 cm. */
#define BALL_CONTROL_START_WINDOW_CM 1.0f
#define BALL_CONTROL_TARGET_TOLERANCE_CM 1.0f
#define BALL_CONTROL_SAFETY_POSITION_CM 11.0f
#define BALL_CONTROL_TARGET_LIMIT_CM 9.0f
#define BALL_CONTROL_CONFIDENCE_MIN 50U
#define BALL_CONTROL_SETTLE_SPEED_CM_S 0.5f
#define BALL_CONTROL_SAMPLE_MAX_MS 250U
#define BALL_CONTROL_SAMPLE_MAX_S 0.25f
#define BALL_CONTROL_KICK_MOVE_CM 0.2f
/* 0: Q3 centre -> +5 -> -5 hold; 1: arbitrary target hold. */
#define BALL_CONTROL_SINGLE_TARGET_MODE 0U
#define BALL_CONTROL_SINGLE_TARGET_CM 5.0f

/* Camera state observer.  Velocity is an estimated damping signal, not a
 * separately measured state and therefore does not own another PID loop. */
#define BALL_ESTIMATOR_ALPHA 0.65f
#define BALL_ESTIMATOR_BETA 0.08f
#define BALL_ESTIMATOR_INNOVATION_LIMIT_CM 2.5f
#define BALL_ESTIMATOR_REACQUIRE_COUNT 3U

/* Ball outer PD, in physical beam angle.  These are conservative first-pass
 * values converted from 0.015 deg/mm and 0.013 deg/(mm/s).
 * Kd/Kp ~= 0.87 s is the approximate velocity look-ahead for braking. */
#define BALL_OUTER_KP_DEG_PER_CM 0.17f
/* Use modest damping while the ball is still far from target.  Full D is
 * restored only in the final braking zone, avoiding premature reversal near
 * the tube's local static-friction pockets. */
#define BALL_OUTER_KD_CRUISE_DEG_PER_CM_S 0.04f
#define BALL_OUTER_KD_DEG_PER_CM_S 0.13f
#define BALL_OUTER_RETURN_CRUISE_KD_DEG_PER_CM_S 0.04f
#define BALL_OUTER_RETURN_BRAKE_KD_DEG_PER_CM_S 0.14f
#define BALL_OUTER_RETURN_HOLD_KD_DEG_PER_CM_S 0.08f
#define BALL_OUTER_BRAKE_ZONE_CM 1.2f
#define BALL_OUTER_FEEDFORWARD_DEG 0.0f
#define BALL_OUTER_POSITION_DEADBAND_CM 0.3f
/* 0.9 beam degree maps through our measured height table to about 7.9 motor
 * degrees. 25 motor degrees remains only the mechanical emergency bound. */
#define BALL_OUTER_BEAM_LIMIT_DEG 0.9f
#define BALL_OUTER_RETURN_CRUISE_BEAM_LIMIT_DEG 0.75f
#define BALL_OUTER_RETURN_BRAKE_BEAM_LIMIT_DEG 0.45f
#define BALL_OUTER_RETURN_HOLD_BEAM_LIMIT_DEG 0.30f
#define BALL_OUTER_BEAM_RATE_LIMIT_DEG_S 6.0f
#define BALL_OUTER_BEAM_ACCEL_LIMIT_DEG_S2 30.0f
#define BALL_CONTROL_COMMAND_DELTA_DEG 0.05f
/* The outer loop is event-driven: every new visual sample is evaluated.
 * A changed angle command is therefore eligible for immediate FC dispatch;
 * unchanged setpoints are deliberately not resent to the X42S. */
#define BALL_CONTROL_COMMAND_MIN_PERIOD_MS 0U

/* A direction-specific one-shot breakaway kick.  It is removed before
 * continuous P control starts; it is not an I term and does not accumulate. */
#define BALL_CONTROL_BREAKAWAY_TO_VISUAL_POS_DEG -7.0f
#define BALL_CONTROL_BREAKAWAY_TO_VISUAL_NEG_DEG 7.0f
#define BALL_CONTROL_BREAKAWAY_POS_REACHED_DEG -6.5f
#define BALL_CONTROL_BREAKAWAY_NEG_REACHED_DEG 6.5f
#define BALL_CONTROL_BREAKAWAY_SPEED_RPM 8.0f
#define BALL_CONTROL_BREAKAWAY_ACCEL_RPM_S 100U
#define BALL_CONTROL_BREAKAWAY_DECEL_RPM_S 100U
#define BALL_CONTROL_BREAKAWAY_MAX_CURRENT_MA 600U
#define BALL_CONTROL_BREAKAWAY_TIMEOUT_MS 800U
#define BALL_CONTROL_BREAKAWAY_HOLD_MS 120U
#define BALL_CONTROL_RETURN_TIMEOUT_MS 500U
#define BALL_CONTROL_RETURN_TOLERANCE_DEG 0.5f
#define BALL_CONTROL_ZERO_MOVE_TIMEOUT_MS 1500U
#define BALL_CONTROL_ZERO_RETRY_PERIOD_MS 100U

/* Static-friction assist for the normal PD phase.  This is deliberately not
 * an integral term: when the ball is outside the 1 cm scoring band but has
 * stopped in the measured tube/groove friction pocket, hold the proven
 * direction-specific breakaway angle.  Release it after real motion appears,
 * then let the PD term supply predictive braking. */
#define BALL_OUTER_STICTION_ERROR_CM 1.0f
#define BALL_OUTER_STICTION_ENTER_SPEED_CM_S 0.5f
#define BALL_OUTER_STICTION_RELEASE_SPEED_CM_S 0.8f
#define BALL_OUTER_STICTION_TO_VISUAL_POS_MOTOR_DEG -7.0f
#define BALL_OUTER_STICTION_TO_VISUAL_NEG_MOTOR_DEG 7.0f

/* Low-speed continuous angle tracking after the ball is moving. */
/* 6 RPM = 36 deg/s: even without ramps, 5 deg takes at least 139 ms.
 * Encoder settling and mechanical settling require measurement. */
#define BALL_CONTROL_MAX_SPEED_RPM 6.0f
#define BALL_CONTROL_ACCEL_RPM_S 100U
#define BALL_CONTROL_DECEL_RPM_S 100U
#define BALL_CONTROL_MAX_CURRENT_MA 600U

#define BALL_CONTROL_ENABLE_SETTLE_MS 300U
#define BALL_CONTROL_COMMAND_GAP_MS 100U
/* Parameter frame F1 must be accepted before the next FC target frame, but
 * a full 100 ms wait after breakaway needlessly leaves the ball open-loop. */
#define BALL_CONTROL_NORMAL_PARAM_GAP_MS 20U
/* One bad detect/status frame must not drop the beam.  The controller holds
 * its last accepted measurement/angle command until this much time has
 * elapsed without a new usable visual sample. At 60 fps this tolerates about
 * 15 consecutive bad frames, but still stops a real camera/link loss. */
#define BALL_CONTROL_VISION_TIMEOUT_MS 250U
/* Actuator position is polled every 50 ms.  A missed reply is retried only
 * after the driver's 100 ms reply timeout, so a valid retry can arrive about
 * 160 ms after the previous feedback.  Keep this above one retry interval;
 * otherwise a normal UART retry is falsely treated as actuator loss. */
#define BALL_CONTROL_ANGLE_TIMEOUT_MS 800U
/* Camera coordinates are quantised.  Do not use a one-frame derivative to
 * decide that the ball is stationary: one pixel at 60 fps looks like a large
 * false velocity.  Instead require all position samples to remain in this
 * small window for the specified duration. */
#define BALL_CONTROL_STILL_POSITION_BAND_CM 0.25f
#define BALL_CONTROL_STILL_WINDOW_MS 500U
#define BALL_CONTROL_SETTLE_HOLD_MS 300U
#define BALL_CONTROL_RUN_TIMEOUT_MS 20000U

/* Bounded pulse to escape a local groove-friction pocket after P is proven
 * active but the ball stays outside the scoring band. */
#define BALL_CONTROL_REKICK_ERROR_CM 1.0f
#define BALL_CONTROL_REKICK_MAX_COUNT 3U

#endif
