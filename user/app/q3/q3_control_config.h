#ifndef Q3_CONTROL_CONFIG_H
#define Q3_CONTROL_CONFIG_H

/* Mechanical facts measured on this car.  Keep them centralized so later
 * tuning never hides hardware knowledge inside state-machine code. */
#define Q3_BEAM_LENGTH_MM 250.0f
#define Q3_MOTOR_SAFE_LIMIT_DEG 25.0f

/* Confirmed sign convention:
 *   +motor angle -> right end rises -> ball moves to visual negative.
 * Therefore the outer position controller needs a negative plant sign. */
#define Q3_POSITION_TO_MOTOR_SIGN -1.0f

/* Direction-specific dynamic breakaway values measured at 8 rpm.
 * These are short feedforward boosts, never continuous PID outputs. */
#define Q3_BREAKAWAY_TO_VISUAL_NEG_DEG 7.0f
#define Q3_BREAKAWAY_TO_VISUAL_POS_DEG -7.0f
#define Q3_BREAKAWAY_SPEED_RPM 8.0f

/* First-pass outer-loop constraints.  The active position PID gains and
 * breakaway values live in ball_control_config.h; this file keeps the
 * measured mechanical limits and timing constants in one place. */
#define Q3_OUTER_LOOP_PERIOD_MS 10U
#define Q3_ACTUAL_ANGLE_FEEDBACK_PERIOD_MS 50U
#define Q3_VISION_TIMEOUT_MS 250U
#define Q3_NORMAL_ANGLE_LIMIT_DEG 8.0f

/* The formal final condition is the measured ball position, not the last
 * 0.1 cm internal reference step: it must be stationary inside -5 +/- 1 cm.
 * BallPositionControl's target_reached flag supplies the velocity and
 * stillness proof; this defines the independent scoring-position band. */
#define Q3_FINAL_ACCEPTANCE_TOLERANCE_CM 1.0f

/* Once that final condition is recorded, freeze the stopwatch and immediately
 * command the X42S session target back to the PA31-established zero.  The
 * stable-window test already happened before this state, so a second timed
 * hold would only make the UI appear not to have stopped. */
#define Q3_FINAL_HOLD_MS 0U
#define Q3_RETURN_ZERO_TIMEOUT_MS 1500U
/* Limit re-enable/zero writes while a first PA31 preparation request waits
 * for fresh X42S option feedback after reset or an intentional disable. */
#define Q3_ZERO_PREPARE_RETRY_MS 200U

/* Fast +5 -> -5 transition.  The contest sequence does not require a hold
 * at +5; one valid visual sample in the +/-1.0 cm band triggers reversal.
 * Keep this separate from the final-position tolerance: it prevents the
 * generic outer controller's HOLD state from deadlocking the Q3 sequence
 * around +4.2 cm when static friction is still present. */
#define Q3_PLUS5_SWITCH_TOLERANCE_CM 1.0f
#define Q3_PLUS5_CONFIRM_FRAMES 1U

/* Return profile: the internal reference moves continuously instead of
 * commanding a 10 cm step.  Cruise is faster in the middle; the reference
 * and output both slow down in the predicted braking zone. */
#define Q3_RETURN_FINAL_TARGET_CM -5.0f
#define Q3_RETURN_REFERENCE_UPDATE_MS 20U
#define Q3_RETURN_CRUISE_REFERENCE_SPEED_CM_S 14.0f
#define Q3_RETURN_BRAKE_REFERENCE_SPEED_CM_S 3.0f
#define Q3_RETURN_BRAKE_REFERENCE_START_CM -2.5f
#define Q3_RETURN_FEEDBACK_DELAY_MS 150U
#define Q3_RETURN_DECELERATION_CM_S2 70.0f
#define Q3_RETURN_BRAKE_MARGIN_CM 0.5f
#define Q3_RETURN_BRAKE_FALLBACK_DISTANCE_CM 1.0f

#endif
