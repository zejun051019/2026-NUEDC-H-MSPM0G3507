#ifndef Q45_VEHICLE_CONFIG_H
#define Q45_VEHICLE_CONFIG_H

#include "q4_control_config.h"

/*
 * Vehicle-route configuration for contest questions 4 and 5.
 *
 * The Q4 static-recovery gains remain in q4_control_config.h.  All dynamic
 * vehicle compensation below is Q5-owned so a Q5 tune cannot silently
 * change the validated Q3 or static-Q4 loops.
 */
#define Q45_POSITION_TO_MOTOR_SIGN -1.0f

/* Q5 dynamic model.  The rail is mounted across the vehicle, so the measured
 * lateral acceleration v^2/R is the dominant turn disturbance.  This car has
 * no gyro, so curvature comes from the fixed 1.5 m straight / 0.5 m radius
 * oval geometry and route odometry.  The two semicircles have opposite
 * curvature: the first is negative and the second positive in the rail
 * coordinate.  COM12 confirms that a positive first-turn compensation makes
 * the inherited static PD immediately fight it, so the sign belongs in the
 * route model rather than being globally flipped for both bends. */
#define Q45_ACCELERATION_FEEDFORWARD_ENABLED 1U
#define Q45_Q5_GRAVITY_M_S2 9.80665f
/* Static balance: g*sin(theta) = a_parallel*cos(theta).  The sphere's
 * rolling inertia multiplies both terms and cancels at zero acceleration. */
/* Three completed Q5 traces drifted +1.9--3.5 cm during the accelerating
 * first straight while this term supplied only 0.10--0.20 degree.  Use the
 * full measured a/g projection here; turn compensation remains independent. */
#define Q45_Q5_LONGITUDINAL_FF_GAIN 1.00f
/* At launch, COM12 shows the acceleration model asking for +0.2--0.5 degree
 * while the ball has already moved rearward.  Let the unchanged static-Q4
 * loop establish the departure balance first, then restore the proven full
 * longitudinal model well before the first bend at 150 cm. */
#define Q45_Q5_LONGITUDINAL_FF_RAMP_DISTANCE_CM 60.0f
/* With f100=0 during the first 60 cm, the ball still held +1.8--2.0 cm.
 * This Q5-only multiplier strengthens the inherited static correction only
 * before the first bend; it does not alter Q3, static Q4, or bend tuning. */
#define Q45_Q5_LAUNCH_STATIC_BEAM_GAIN 1.40f
/* COM12: at 300 mm/s, the geometric 1.05 degree full-scale term reverses
 * the ball through centre before the inherited static PD can settle it.
 * Keep its verified sign but apply only 55% (about 0.58 degree at cruise),
 * leaving the validated Q4 PD authority for the residual. */
#define Q45_Q5_LATERAL_FF_GAIN 0.55f
/* First bend: Q5D shows a -0.55--0.65 degree steady feedforward fighting
 * the Q4 brake after the ball crosses centre.  Match the independently
 * proven second-bend level while keeping each bend separately tunable. */
#define Q45_Q5_FIRST_TURN_LATERAL_FF_GAIN 0.30f
/* The 24 cm second-turn ramp removed the entry step, but its 0.6 degree
 * plateau still cancels the inherited Q4 brake and drives the ball negative.
 * Reduce only the second bend; the first-turn compensation stays unchanged. */
#define Q45_Q5_SECOND_TURN_LATERAL_FF_GAIN 0.15f
#define Q45_Q5_FEEDFORWARD_GAIN 1.00f
#define Q45_Q5_FEEDFORWARD_BEAM_SIGN 1.00f
#define Q45_Q5_FEEDFORWARD_LIMIT_DEG 1.60f
#define Q45_Q5_PARALLEL_ACCEL_LIMIT_M_S2 0.40f
#define Q45_Q5_SPEED_FILTER_ALPHA 0.30f
#define Q45_Q5_ACCEL_FILTER_ALPHA 0.20f
#define Q45_Q5_CHASSIS_SAMPLE_MS 20U
/* Q5 logs show isolated 260--280 ms camera frame gaps followed by valid
 * tracking.  Keep the static-Q4 readiness watchdog unchanged; Q5 alone gets
 * enough margin to ride through one camera scheduling gap, while a sustained
 * loss still stops the vehicle safely. */
#define Q45_Q5_VISION_TIMEOUT_MS 600U
#define Q45_Q5_TURN_RADIUS_M 0.50f
#define Q45_Q5_FIRST_TURN_START_CM 150.0f
/* The 0.15 s route preview released the first-bend term before the
 * physical exit.  Retain it 8 cm into the first straight; keep bend two
 * independent. */
#define Q45_Q5_FIRST_TURN_END_CM 315.08f
#define Q45_Q5_SECOND_TURN_START_CM 457.08f
#define Q45_Q5_SECOND_TURN_END_CM 614.16f
#define Q45_Q5_FIRST_TURN_ACCEL_SIGN -1.0f
#define Q45_Q5_SECOND_TURN_ACCEL_SIGN 1.0f
#define Q45_Q5_TURN_BLEND_DISTANCE_CM 6.0f
/* Q5D shows the second-turn feedforward jumping 0 -> 0.46 degree in 0.2 s,
 * before X42S can follow.  Broaden that turn only; first-turn tuning stays
 * unchanged. */
#define Q45_Q5_SECOND_TURN_BLEND_DISTANCE_CM 24.0f
/* Final Q5 route calibration: retain distinct entry, apex and exit profiles
 * for each bend, followed by bounded vision-feedback corrections. */
#define Q45_Q5_FIRST_TURN_ENTRY_LATERAL_FF_GAIN 0.40f
#define Q45_Q5_SECOND_TURN_ENTRY_LATERAL_FF_GAIN 0.20f
#define Q45_Q5_FIRST_TURN_ENTRY_GAIN_HOLD_CM 125.0f
#define Q45_Q5_SECOND_TURN_ENTRY_GAIN_HOLD_CM 125.0f
#define Q45_Q5_TURN_ENTRY_GAIN_FADE_CM 40.0f
#define Q45_Q5_FIRST_TURN_APEX_RAMP_START_CM 10.0f
#define Q45_Q5_FIRST_TURN_APEX_RAMP_FULL_CM 35.0f
#define Q45_Q5_FIRST_TURN_APEX_FADE_START_CM 85.0f
#define Q45_Q5_FIRST_TURN_APEX_FADE_END_CM 125.0f
#define Q45_Q5_SECOND_TURN_APEX_RAMP_START_CM 5.0f
#define Q45_Q5_SECOND_TURN_APEX_RAMP_FULL_CM 35.0f
#define Q45_Q5_SECOND_TURN_APEX_FADE_START_CM 125.0f
#define Q45_Q5_SECOND_TURN_APEX_FADE_END_CM 155.0f
#define Q45_Q5_FIRST_TURN_APEX_EXTRA_GAIN 0.14f
#define Q45_Q5_SECOND_TURN_APEX_EXTRA_GAIN 0.00f
#define Q45_Q5_SECOND_TURN_FF_POLARITY -1.0f
#define Q45_Q5_TURN_PREVIEW_S 0.15f

#define Q45_Q5_FIRST_FB_START_CM 155.0f
#define Q45_Q5_FIRST_FB_FULL_CM 175.0f
#define Q45_Q5_FIRST_FB_FADE_START_CM 290.0f
#define Q45_Q5_FIRST_FB_END_CM 315.0f
#define Q45_Q5_FIRST_FB_DEADBAND_CM 0.60f
#define Q45_Q5_FIRST_FB_KP_DEG_PER_CM 0.090f
#define Q45_Q5_FIRST_FB_KD_DEG_PER_CM_S 0.060f
#define Q45_Q5_FIRST_FB_D_LIMIT_DEG 0.18f
#define Q45_Q5_FIRST_FB_LIMIT_DEG 0.25f
#define Q45_Q5_FIRST_FB_CONF_MIN 80U
#define Q45_Q5_FIRST_TRIAL_ABORT_CM 4.50f
#define Q45_Q5_REVERSED_SECOND_TEST_ABORT_CM 4.50f
#define Q45_Q5_SECOND_FB_START_CM 440.0f
#define Q45_Q5_SECOND_FB_FULL_CM 460.0f
#define Q45_Q5_SECOND_FB_END_CM 622.16f
#define Q45_Q5_SECOND_FB_DEADBAND_CM 0.60f
#define Q45_Q5_SECOND_FB_KP_DEG_PER_CM 0.13f
#define Q45_Q5_SECOND_FB_P_LIMIT_DEG 0.55f
#define Q45_Q5_SECOND_FB_KI_DEG_PER_CM_S 0.07f
#define Q45_Q5_SECOND_FB_I_LIMIT_DEG 0.30f
#define Q45_Q5_SECOND_FB_I_MAX_SPEED_CM_S 2.00f
#define Q45_Q5_SECOND_FB_I_CONF_MIN 80U
#define Q45_Q5_SECOND_FB_I_LEAK_PER_S 1.50f
#define Q45_Q5_TURN_STICTION_DISABLE_LAT_M_S2 0.08f
#define Q45_Q5_LATE_STRAIGHT_STATIC_GAIN 1.08f
#define Q45_Q5_FIRST_STRAIGHT_LATE_START_CM 75.0f
#define Q45_Q5_SECOND_STRAIGHT_LATE_START_CM 386.08f
#define Q45_Q5_LATE_STRAIGHT_GAIN_BLEND_CM 25.0f

/* Q5-only ball loop, expressed in physical beam degrees.  P/D correct the
 * residual after model feedforward.  I is deliberately slow and bounded; it
 * only learns a small remaining installation bias near centre. */
/* Q5-owned dynamic ball-control values copied from the active final Q5
 * baseline; Q4 and Q6 continue to use their existing mode-specific paths. */
#define Q45_Q5_KP_BEAM_DEG_PER_CM 0.20f
#define Q45_Q5_KP_ERROR_POS 0.23f
#define Q45_Q5_KD_BEAM_DEG_PER_CM_S 0.20f
#define Q45_Q5_KD_ERROR_POS 0.20f
#define Q45_Q5_BRAKE_KP 0.12f
#define Q45_Q5_BRAKE_KD 0.12f
#define Q45_Q5_BRAKE_KD_OUTWARD 0.16f
#define Q45_Q5_CRUISE_KD_OUTWARD 0.20f
#define Q45_Q5_HOLD_KP 0.10f
#define Q45_Q5_HOLD_KD 0.12f
#define Q45_Q5_CRUISE_BEAM_LIMIT_DEG 1.80f
#define Q45_Q5_CRUISE_BEAM_LIMIT_POS_DEG 2.20f
#define Q45_Q5_CRUISE_BEAM_LIMIT_NEG_DEG 2.00f
#define Q45_Q5_BRAKE_BEAM_LIMIT_DEG 1.80f
#define Q45_Q5_BRAKE_BEAM_LIMIT_NEG_DEG 1.80f
#define Q45_Q5_HOLD_BEAM_LIMIT_DEG 0.50f
#define Q45_Q5_MID_DISTANCE_CM 5.00f
#define Q45_Q5_MID_BEAM_LIMIT_DEG 1.60f
#define Q45_Q5_HOLD_POSITION_TOLERANCE_CM 1.00f
#define Q45_Q5_HOLD_SPEED_LIMIT_CM_S 3.00f
#define Q45_Q5_BRAKE_FALLBACK_CM 7.00f
#define Q45_Q5_BRAKE_FALLBACK_POS_CM 5.00f
#define Q45_Q5_BRAKE_MARGIN_PROFILE_CM 2.00f
#define Q45_Q5_FEEDBACK_DELAY_MS 150U
#define Q45_Q5_DECELERATION_CM_S2 70.0f
#define Q45_Q5_LEAD_TIME_S 0.25f
#define Q45_Q5_LEAD_TIME_ERROR_NEG_S 0.30f
/* Q5-only slow trim: three full traces retain +1.6--2.4 cm after chassis
 * acceleration has fallen to zero.  This is not part of static Q4. */
/* Q5D proved the trim sign is correct, but only +0.06--0.12 degree has
 * accumulated before the first turn.  Double convergence speed while
 * retaining the existing 0.25 degree cap and all gating. */
#define Q45_Q5_KI_BEAM_DEG_PER_CM_S 0.070f
#define Q45_Q5_INTEGRAL_LIMIT_DEG 0.25f
#define Q45_Q5_INTEGRAL_LEAK_PER_S 0.10f
#define Q45_Q5_INTEGRAL_ACTIVE_POSITION_CM 1.00f
#define Q45_Q5_INTEGRAL_ACTIVE_SPEED_CM_S 2.00f
#define Q45_Q5_INTEGRAL_ACTIVE_ACCEL_M_S2 0.02f
#define Q45_Q5_D_TERM_LIMIT_DEG 0.80f
#define Q45_Q5_TOTAL_BEAM_LIMIT_DEG 2.20f
#define Q45_Q5_BEAM_RATE_LIMIT_DEG_S 6.00f
#define Q45_Q5_BEAM_ACCEL_LIMIT_DEG_S2 30.0f
#define Q45_Q5_COMMAND_LEAD_LIMIT_DEG 0.70f
#define Q45_Q5_COMMAND_PERIOD_MS Q4_RECOVERY_COMMAND_PERIOD_MS
/* X42S position replies are asynchronous diagnostic feedback, not the command
 * acknowledgement.  COM12 traces show normal replies may be 300 ms apart
 * while the FOC link remains online and follows the target.  A zero value
 * delegates link-loss protection to the actuator's established online
 * watchdog, matching the proven Q4 static-recovery path. */
#define Q45_Q5_ACTUAL_ANGLE_TIMEOUT_MS 0U
#define Q45_Q5_PREDICTION_TIME_S 0.18f

/* Scoring and vehicle motion are separate.  Crossing +/-1 cm latches a failed
 * run, but does not toggle chassis speed.  Only route/time/fault/operator
 * termination stops the chassis, and terminal states never auto-restart. */
#define Q45_Q5_INTERNAL_POSITION_CM 0.35f
#define Q45_Q5_INTERNAL_SPEED_CM_S 1.00f
#define Q45_Q5_HOLD_BLEND_END_CM 0.65f
#define Q45_Q5_HOLD_BLEND_END_SPEED_CM_S 2.00f
#define Q45_Q5_BRAKE_MARGIN_CM 0.15f
#define Q45_Q5_BRAKE_BLEND_CM 0.30f

/* Reuse the measured actuator baseline while fixing control scheduling. */
#define Q45_Q5_ACTUATOR_ACCEL_RPM_S Q4_ACCEL_RPM_S
#define Q45_Q5_ACTUATOR_DECEL_RPM_S Q4_DECEL_RPM_S
#define Q45_Q5_ACTUATOR_MAX_SPEED_RPM Q4_MAX_SPEED_RPM
#define Q45_Q5_ACTUATOR_MAX_CURRENT_MA Q4_MAX_CURRENT_MA
#define Q45_Q5_CONFIDENCE_MIN 60U

/* A->B is 1.50 m.  A 5.0 s ramp reaches B in about 7.5 s of formal wheel
 * motion, leaving margin inside the 8 s stopwatch that begins at second key
 * press.  Motor preparation and manual ball placement occur beforehand. */
#define Q45_Q4_CRUISE_SPEED_MM_S 300.0f
#define Q45_Q4_ACCEL_TIME_MS 5000U
#define Q45_Q4_B_DISTANCE_CM 150.0f
#define Q45_Q4_STOP_DISTANCE_CM 180.0f
#define Q45_Q4_TIME_LIMIT_MS 8000U

/* One lap is 2*1.5 m + 2*pi*0.5 m = 6.1416 m.  Continue 8 cm past A so the
 * vehicle demonstrably passes the mark before the ordinary chassis brake. */
#define Q45_Q5_CRUISE_SPEED_MM_S 300.0f
#define Q45_Q5_ACCEL_TIME_MS 6000U
#define Q45_Q5_A_DISTANCE_CM 614.16f
#define Q45_Q5_STOP_DISTANCE_CM 622.16f
#define Q45_Q5_TIME_LIMIT_MS 30000U

/* The motor session is prepared before vehicle motion.  These delays retain
 * the proven X42S F1->FC ordering from the static Q4 controller. */
#define Q45_ACTUATOR_READY_TIMEOUT_MS 1500U
#define Q45_ENABLE_TO_PROFILE_GAP_MS 100U
#define Q45_PROFILE_TO_ZERO_GAP_MS 20U
#define Q45_ZERO_TIMEOUT_MS 1000U
/* Q6 physically returns to the motor's stored absolute zero before every
 * trial, so its zero-stage timeout must cover the return move: a run left
 * parked at ~14 deg needs about one second of return plus ramp margins. */
#define Q45_Q6_ZERO_RETURN_TIMEOUT_MS 5000U

/* Safety/watchdog limits use the same centre and fresh-frame semantics as
 * static Q4; the contest requirement itself remains |x| <= 1 cm. */
#define Q45_VISION_TIMEOUT_MS Q4_VISUAL_READY_AGE_MS
#define Q45_CENTER_POSITION_TOLERANCE_CM Q4_HOLD_POSITION_TOLERANCE_CM
#define Q45_CENTER_SPEED_LIMIT_CM_S Q4_HOLD_SPEED_LIMIT_CM_S
#define Q45_BALL_SAFETY_POSITION_CM 11.0f

/* A finite 4 degree Q4 pulse is available only after a measured stationary
 * error persists.  It is not an acceleration feedforward or an integrator. */
#define Q45_STICTION_CONFIRM_MS 500U
#define Q45_STICTION_HOLD_MS Q4_BREAKAWAY_HOLD_MS
#define Q45_STICTION_MAX_COUNT 3U

#endif
