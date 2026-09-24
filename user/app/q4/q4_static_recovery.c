#include "q4_static_recovery.h"

#if Q4_STATIC_RECOVERY_TEST_ENABLED

#include <math.h>
#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/ball_outer_pd.h"
#include "app/q3/ball_state_estimator.h"
#include "app/q3/q3_control_config.h"
#include "app/q3/pendulum_geometry.h"
#include "q4_control_config.h"
#include "code/vision_link.h"

static Q4StaticStatus s_status;
static bool s_result_log_pending;
static uint16_t s_random_state;
static uint8_t s_disturb_stage;
static uint32_t s_disturb_started_ms;
static uint8_t s_zero_stage;
static uint32_t s_zero_started_ms;
static uint32_t s_zero_last_command_ms;
static bool s_zero_nudge_sent;
static BallOuterPd s_recovery_pd;
static BallStateEstimator s_recovery_estimator;
static uint32_t s_last_vision_rx_ms;
static uint32_t s_last_valid_vision_ms;
static bool s_have_recovery_sample;
static uint32_t s_hold_started_ms;
static uint32_t s_hold_candidate_ms;
static float s_last_commanded_angle_deg;
static uint32_t s_recovery_command_ready_ms;
static uint32_t s_last_command_send_ms;
static bool s_recovery_breakaway_active;
static bool s_direct_recovery_active;
static bool s_startup_zero_pending;
static uint8_t s_recovery_breakaway_count;
static uint32_t s_recovery_still_started_ms;
static uint32_t s_recovery_breakaway_started_ms;
static uint32_t s_recovery_started_ms;
static uint32_t s_ready_center_since_ms;
static bool s_fault_zero_pending;
static float s_recovery_breakaway_limit_deg;

static void Q4StaticRecovery_Enter(Q4StaticState state, uint32_t now_ms);
static float Q4StaticRecovery_Abs(float value);
static bool Q4StaticRecovery_ComputeFastMotor(float error_cm,
                                              float velocity_cm_s,
                                              BallOuterPdProfile profile,
                                              float *motor_deg);
static void Q4StaticRecovery_ApplyMeasuredDistanceLimit(float measured_cm,
                                                        BallOuterPdProfile profile,
                                                        float *motor_deg);

/* Select the same three output profiles already tuned for Q3.  Q4 keeps a
 * zero-centimetre reference, so the Q3 return-distance test is expressed as
 * absolute distance and speed toward zero.  No Q3 source or parameter is
 * modified; this only chooses which frozen profile the Q4 wrapper uses. */
static BallOuterPdProfile Q4StaticRecovery_SelectProfile(void)
{
    float distance_cm = Q4StaticRecovery_Abs(s_recovery_estimator.position_cm);
    float speed_toward_cm_s = 0.0f;
    float speed_abs = Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s);
    float stopping_distance_cm;
    float brake_fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM;

    /* The two mechanical directions are not symmetric.  Keep the earlier
     * 7 cm brake entry for the direction that benefited from it, while the
     * opposite error direction retains the verified 5 cm entry until it is
     * tuned independently. */
    if (s_recovery_estimator.position_cm > 0.0f)
    {
        brake_fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM_ERROR_NEG;
    }

    /* The first visual sample after the disturbance can still be the delayed
     * centre sample from before the ball has visibly moved.  Keep the finite
     * Q4 breakaway pulse alive for that first sample; otherwise the caller
     * clears it before a target is ever emitted. */
    if (s_recovery_breakaway_active && !s_have_recovery_sample)
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }

    if ((s_recovery_estimator.position_cm * s_recovery_estimator.velocity_cm_s) < 0.0f)
    {
        speed_toward_cm_s = speed_abs;
    }
    stopping_distance_cm =
        speed_toward_cm_s * ((float)Q3_RETURN_FEEDBACK_DELAY_MS / 1000.0f) +
        (speed_toward_cm_s * speed_toward_cm_s) / (2.0f * Q3_RETURN_DECELERATION_CM_S2) +
        Q4_RECOVERY_BRAKE_MARGIN_CM;

    /* A stationary ball can still be trapped in the measured mechanical
     * dead-zone while it is inside the normal brake distance.  Let the
     * finite Q4 breakaway pulse take priority over BRAKE in that case;
     * otherwise the caller clears the pulse before it is ever emitted. */
    if (s_recovery_breakaway_active && (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S) &&
        (distance_cm > BALL_OUTER_STICTION_ERROR_CM))
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }

    if ((distance_cm <= Q4_HOLD_POSITION_TOLERANCE_CM) && (speed_abs <= Q4_HOLD_SPEED_LIMIT_CM_S))
    {
        return BALL_OUTER_PROFILE_RETURN_HOLD;
    }
    if ((distance_cm <= stopping_distance_cm) || (distance_cm <= brake_fallback_cm))
    {
        return BALL_OUTER_PROFILE_RETURN_BRAKE;
    }
    /* Keep Q3's one-shot stiction assist available only while the ball is
     * still effectively stationary.  Once motion is measured, leave NORMAL
     * immediately so the assist cannot become a continuous drive. */
    if (s_recovery_breakaway_active && (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
    {
        return BALL_OUTER_PROFILE_NORMAL;
    }
    if (speed_abs > BALL_OUTER_STICTION_RELEASE_SPEED_CM_S)
    {
        return BALL_OUTER_PROFILE_RETURN_CRUISE;
    }
    return BALL_OUTER_PROFILE_RETURN_CRUISE;
}

static void Q4StaticRecovery_ApplyMeasuredDistanceLimit(float measured_cm,
                                                        BallOuterPdProfile profile,
                                                        float *motor_deg)
{
    float mid_motor_deg;
    if ((motor_deg == NULL) || (profile == BALL_OUTER_PROFILE_RETURN_HOLD) ||
        (Q4StaticRecovery_Abs(measured_cm) > Q4_RECOVERY_MID_DISTANCE_CM))
    {
        return;
    }
    if (!PendulumGeometry_BeamDegToMotorDeg(Q4_RECOVERY_MID_BEAM_LIMIT_DEG, &mid_motor_deg))
    {
        return;
    }
    if (Q4StaticRecovery_Abs(*motor_deg) > Q4StaticRecovery_Abs(mid_motor_deg))
    {
        *motor_deg = (*motor_deg < 0.0f) ? -mid_motor_deg : mid_motor_deg;
    }
}

static bool Q4StaticRecovery_BeginZero(uint32_t now_ms)
{
    BallActuatorStatus actuator;

    BallActuator_GetStatus(&actuator);
    if (!actuator.online || !actuator.options_valid || !actuator.firmware_x ||
        !actuator.foc_closed_loop || !actuator.actual_angle_valid)
    {
        return false;
    }
    BallActuator_SetCommandSession(true);
    if (!BallActuator_ClearProtection() || !BallActuator_Enable(true))
    {
        BallActuator_SetCommandSession(false);
        return false;
    }
    s_zero_stage = 1U;
    s_zero_started_ms = 0U;
    s_zero_last_command_ms = 0U;
    s_zero_nudge_sent = false;
    Q4StaticRecovery_Enter(Q4_STATIC_RETURN_ZERO, now_ms);
    return true;
}

static void Q4StaticRecovery_Fault(uint32_t now_ms)
{
    /* Stop first, then try to return the Q4 session coordinate to zero before
     * exposing the terminal FAULT state.  This is Q4-only; Q3's fault path is
     * untouched.  A zero-return timeout remains a hard terminal fault. */
    (void)BallActuator_Stop();
    s_status.fault = true;
    s_fault_zero_pending = true;
    if ((s_status.state != Q4_STATIC_RETURN_ZERO) && Q4StaticRecovery_BeginZero(now_ms))
    {
        return;
    }
    if (s_status.fault_reason == 0U)
    {
        s_status.fault_reason = 6U;
    }
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    s_fault_zero_pending = false;
    Q4StaticRecovery_Enter(Q4_STATIC_FAULT, now_ms);
}

static bool Q4StaticRecovery_UpdateControl(uint32_t now_ms)
{
    VisionLinkSample sample;
    BallActuatorStatus actuator;
    BallOuterPdOutput output;
    BallOuterPdProfile profile;
    float control_error_cm;
    float dt_s;

    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        !isfinite(sample.velocity_cm_s) || sample.confidence_pct < Q4_RECOVERY_CONFIDENCE_MIN ||
        (now_ms - sample.rx_time_ms) > Q4_VISUAL_READY_AGE_MS)
    {
        if (s_have_recovery_sample && (now_ms - s_last_valid_vision_ms) <= Q4_VISUAL_READY_AGE_MS)
        {
            return true;
        }
        s_status.fault_reason = 1U;
        return false;
    }
    if (s_have_recovery_sample && sample.rx_time_ms == s_last_vision_rx_ms)
    {
        return true;
    }
    if (now_ms < s_recovery_command_ready_ms)
    {
        return true;
    }
    BallActuator_GetStatus(&actuator);
    if (!actuator.actual_angle_valid)
    {
        s_status.fault_reason = 2U;
        return false;
    }
    if (!s_have_recovery_sample)
    {
        dt_s = 0.02f;
    }
    else
    {
        dt_s = (float)(sample.rx_time_ms - s_last_vision_rx_ms) / 1000.0f;
        if (dt_s < 0.01f)
            dt_s = 0.01f;
        if (dt_s > BALL_CONTROL_SAMPLE_MAX_S)
            dt_s = BALL_CONTROL_SAMPLE_MAX_S;
    }
    if (!BallStateEstimator_Update(&s_recovery_estimator, sample.position_cm, dt_s))
    {
        /* A single visual jump is held by the estimator; it is not a Q4
         * fault and the previous valid command remains active. */
        return true;
    }
    {
        float distance_cm = Q4StaticRecovery_Abs(s_recovery_estimator.position_cm);
        float speed_abs = Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s);
        if (!s_recovery_breakaway_active && (distance_cm > BALL_OUTER_STICTION_ERROR_CM) &&
            (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
        {
            if (s_recovery_still_started_ms == 0U)
            {
                s_recovery_still_started_ms = now_ms;
            }
            else if ((now_ms - s_recovery_still_started_ms) >= BALL_CONTROL_STILL_WINDOW_MS &&
                     s_recovery_breakaway_count < BALL_CONTROL_REKICK_MAX_COUNT)
            {
                ++s_recovery_breakaway_count;
                s_recovery_breakaway_active = true;
                s_recovery_breakaway_limit_deg =
                    Q4_RECOVERY_BREAKAWAY_START_DEG +
                    ((float)s_recovery_breakaway_count * Q4_RECOVERY_BREAKAWAY_STEP_DEG);
                if (s_recovery_breakaway_limit_deg > Q4_RECOVERY_MAX_ANGLE_DEG)
                {
                    s_recovery_breakaway_limit_deg = Q4_RECOVERY_MAX_ANGLE_DEG;
                }
                s_recovery_breakaway_started_ms = now_ms;
                s_recovery_still_started_ms = 0U;
            }
        }
        else if ((distance_cm <= BALL_OUTER_STICTION_ERROR_CM) ||
                 (speed_abs > BALL_OUTER_STICTION_RELEASE_SPEED_CM_S))
        {
            s_recovery_still_started_ms = 0U;
        }
        s_status.breakaway_count = s_recovery_breakaway_count;
    }
    profile = Q4StaticRecovery_SelectProfile();
    BallOuterPd_SetProfile(&s_recovery_pd, profile);
    control_error_cm = -s_recovery_estimator.position_cm;
    if (profile == BALL_OUTER_PROFILE_RETURN_CRUISE)
    {
        /* Q3 already characterizes about 150 ms of visual feedback delay.
         * Use that delay as a lead on P only in cruise; braking/hold retain
         * the measured error so D is not double-counted near zero. */
        float lead_time_s =
            (control_error_cm < 0.0f) ? Q4_RECOVERY_LEAD_TIME_S_ERROR_NEG : Q4_RECOVERY_LEAD_TIME_S;
        control_error_cm -= s_recovery_estimator.velocity_cm_s * lead_time_s;
    }
    s_status.control_error_cm = control_error_cm;
    {
        s_status.control_profile = (uint8_t)profile;
        if (s_recovery_breakaway_active && (profile != BALL_OUTER_PROFILE_NORMAL))
        {
            /* Q3 uses 8 rpm only for breakaway.  Restore its normal 6-rpm
             * profile as soon as motion is detected, before the next target. */
            if (!BallActuator_PrepareAbsolutePosition(
                    Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
            {
                return true;
            }
            s_recovery_breakaway_active = false;
            s_recovery_command_ready_ms = now_ms + BALL_CONTROL_NORMAL_PARAM_GAP_MS;
            return true;
        }
    }
    if (s_recovery_breakaway_active && (s_recovery_breakaway_started_ms != 0U) &&
        ((now_ms - s_recovery_breakaway_started_ms) >= Q4_BREAKAWAY_HOLD_MS))
    {
        /* A Q3-sized pulse is finite.  Do not hold the breakaway angle as a
         * continuous drive when a mechanical pocket does not release. */
        s_recovery_breakaway_active = false;
        s_recovery_still_started_ms = now_ms;
        (void)BallActuator_PrepareAbsolutePosition(
            Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA);
        s_recovery_command_ready_ms = now_ms + BALL_CONTROL_NORMAL_PARAM_GAP_MS;
        return true;
    }
    if (!BallOuterPd_Update(&s_recovery_pd,
                            control_error_cm,
                            s_recovery_estimator.velocity_cm_s,
                            dt_s,
                            actuator.actual_angle_deg,
                            &output))
    {
        s_status.fault_reason = 3U;
        return false;
    }
    if (!(s_recovery_breakaway_active &&
          (s_status.control_profile == (uint8_t)BALL_OUTER_PROFILE_NORMAL)))
    {
        float fast_motor_deg;
        if (Q4StaticRecovery_ComputeFastMotor(
                control_error_cm, s_recovery_estimator.velocity_cm_s, profile, &fast_motor_deg))
        {
            output.motor_deg = fast_motor_deg;
            output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(fast_motor_deg);
        }
    }
    Q4StaticRecovery_ApplyMeasuredDistanceLimit(
        s_recovery_estimator.position_cm, profile, &output.motor_deg);
    /* Q4 recovery has its own staged limit. It may rise above the frozen Q3
     * normal limit only after a stationary re-kick, and is hard-clipped at
     * the measured 25 degree safety ceiling. */
    if (output.motor_deg > Q4_RECOVERY_MAX_ANGLE_DEG)
    {
        output.motor_deg = Q4_RECOVERY_MAX_ANGLE_DEG;
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    else if (output.motor_deg < -Q4_RECOVERY_MAX_ANGLE_DEG)
    {
        output.motor_deg = -Q4_RECOVERY_MAX_ANGLE_DEG;
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    if (s_recovery_breakaway_active &&
        (s_status.control_profile == (uint8_t)BALL_OUTER_PROFILE_NORMAL))
    {
        /* The first visual sample after a disturbance can still be stale.
         * Force the initial kick opposite the commanded disturbance instead
         * of trusting that old position error; after a new sample arrives,
         * retain the normal error-directed finite re-kick behavior. */
        if (!s_have_recovery_sample)
        {
            output.motor_deg = (s_status.disturbance_angle_deg >= 0.0f)
                                   ? -s_recovery_breakaway_limit_deg
                                   : s_recovery_breakaway_limit_deg;
        }
        else if (output.motor_deg < 0.0f)
        {
            output.motor_deg = -s_recovery_breakaway_limit_deg;
        }
        else if (output.motor_deg > 0.0f)
        {
            output.motor_deg = s_recovery_breakaway_limit_deg;
        }
        else
        {
            if (control_error_cm > 0.0f)
            {
                output.motor_deg = -s_recovery_breakaway_limit_deg;
            }
            else if (control_error_cm < 0.0f)
            {
                output.motor_deg = s_recovery_breakaway_limit_deg;
            }
            else
            {
                output.motor_deg = (s_status.disturbance_angle_deg >= 0.0f)
                                       ? -s_recovery_breakaway_limit_deg
                                       : s_recovery_breakaway_limit_deg;
            }
        }
        output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
    }
    if (s_direct_recovery_active)
    {
        float direct_limit_motor_deg;
        if (PendulumGeometry_BeamDegToMotorDeg(Q4_RECOVERY_DIRECT_BEAM_LIMIT_DEG,
                                               &direct_limit_motor_deg))
        {
            if (output.motor_deg > direct_limit_motor_deg)
            {
                output.motor_deg = direct_limit_motor_deg;
            }
            else if (output.motor_deg < -direct_limit_motor_deg)
            {
                output.motor_deg = -direct_limit_motor_deg;
            }
            output.end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output.motor_deg);
        }
    }
    if ((!s_have_recovery_sample ||
         Q4StaticRecovery_Abs(output.motor_deg - s_last_commanded_angle_deg) >=
             BALL_CONTROL_COMMAND_DELTA_DEG) &&
        ((!s_have_recovery_sample) ||
         ((now_ms - s_last_command_send_ms) >= Q4_RECOVERY_COMMAND_PERIOD_MS)))
    {
        s_last_command_send_ms = now_ms;
        if (!BallActuator_SetAbsoluteAngleDeg(output.motor_deg))
        {
            /* A transient X42S EE/timeout is retried on the next visual
             * sample.  Vision and actuator-feedback watchdogs, plus the
             * overall recovery timeout, remain the fault boundaries. */
            return true;
        }
        s_last_commanded_angle_deg = output.motor_deg;
    }
    s_last_vision_rx_ms = sample.rx_time_ms;
    s_last_valid_vision_ms = now_ms;
    s_have_recovery_sample = true;
    s_status.target_cm = 0.0f;
    s_status.measured_cm = s_recovery_estimator.position_cm;
    s_status.measured_velocity_cm_s = s_recovery_estimator.velocity_cm_s;
    s_status.error_cm = -s_recovery_estimator.position_cm;
    s_status.control_output_deg = output.motor_deg;
    s_status.recovery_limit_deg = s_recovery_breakaway_limit_deg;
    s_status.commanded_angle_deg = s_last_commanded_angle_deg;
    s_status.actual_angle_deg = actuator.actual_angle_deg;
    s_status.vision_sequence = sample.sequence;
    s_status.vision_age_ms = now_ms - sample.rx_time_ms;
    s_status.target_reached =
        (Q4StaticRecovery_Abs(s_recovery_estimator.position_cm) <= Q4_HOLD_POSITION_TOLERANCE_CM) &&
        (Q4StaticRecovery_Abs(s_recovery_estimator.velocity_cm_s) <= Q4_HOLD_SPEED_LIMIT_CM_S);
    s_status.recovery_elapsed_ms =
        (s_recovery_started_ms == 0U) ? 0U : (now_ms - s_recovery_started_ms);
    return true;
}

static void Q4StaticRecovery_Enter(Q4StaticState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    s_result_log_pending = true;
}

static float Q4StaticRecovery_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* Q4-only fast path. The Q3 controller remains the reference implementation,
 * but its deliberately slow shaped beam trajectory is unsuitable immediately
 * after an 12--18 degree disturbance. This path preserves the measured sign
 * and geometry table while giving Q4 a bounded position/velocity command. */
static bool Q4StaticRecovery_ComputeFastMotor(float error_cm,
                                              float velocity_cm_s,
                                              BallOuterPdProfile profile,
                                              float *motor_deg)
{
    float kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM;
    float kd = Q4_RECOVERY_KD_BEAM_DEG_PER_CM_S;
    float beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG;
    float beam_deg;

    /* The measured mechanism is directionally asymmetric: the positive
     * position error (ball on the negative side) was the slower recovery
     * direction in Q4 logs.  Keep the nominal gain for the opposite
     * direction and apply this Q4-only trim here. */
    if (error_cm > 0.0f)
    {
        kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM_ERROR_POS;
    }

    if (profile == BALL_OUTER_PROFILE_RETURN_BRAKE)
    {
        kp = Q4_RECOVERY_BRAKE_KP_BEAM_DEG_PER_CM;
        kd = Q4_RECOVERY_BRAKE_KD_BEAM_DEG_PER_CM_S;
        /* When position and velocity have the same physical sign, the ball
         * is still moving away from zero.  Keep some damping, but reduce the
         * delayed velocity term so it cannot reinforce the outward phase.
         * Once velocity reverses, retain the full brake damping. */
        if ((error_cm * velocity_cm_s) < 0.0f)
        {
            kd = Q4_RECOVERY_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S;
        }
        beam_limit = Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG;
        if (error_cm < 0.0f)
        {
            beam_limit = Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG_ERROR_NEG;
        }
    }
    else if (profile == BALL_OUTER_PROFILE_RETURN_HOLD)
    {
        kp = Q4_RECOVERY_HOLD_KP_BEAM_DEG_PER_CM;
        kd = Q4_RECOVERY_HOLD_KD_BEAM_DEG_PER_CM_S;
        beam_limit = Q4_RECOVERY_HOLD_BEAM_LIMIT_DEG;
    }
    else
    {
        if (error_cm > 0.0f)
        {
            beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG_ERROR_POS;
            kd = Q4_RECOVERY_KD_BEAM_DEG_PER_CM_S_ERROR_POS;
        }
        else if (error_cm < 0.0f)
        {
            beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG_ERROR_NEG;
        }
        if ((profile == BALL_OUTER_PROFILE_RETURN_CRUISE) && ((error_cm * velocity_cm_s) < 0.0f))
        {
            kd = Q4_RECOVERY_CRUISE_KD_OUTWARD_BEAM_DEG_PER_CM_S;
        }
    }
    beam_deg = Q3_POSITION_TO_MOTOR_SIGN * (kp * error_cm - kd * velocity_cm_s);
    if (beam_deg > beam_limit)
        beam_deg = beam_limit;
    if (beam_deg < -beam_limit)
        beam_deg = -beam_limit;
    return PendulumGeometry_BeamDegToMotorDeg(beam_deg, motor_deg) &&
           (Q4StaticRecovery_Abs(*motor_deg) <= Q4_RECOVERY_MAX_ANGLE_DEG);
}

static bool Q4StaticRecovery_Ready(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;

    BallActuator_GetStatus(&actuator);
    if (!actuator.online || !actuator.options_valid || !actuator.firmware_x ||
        !actuator.foc_closed_loop)
    {
        return false;
    }
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        !isfinite(sample.velocity_cm_s) || sample.confidence_pct < BALL_CONTROL_CONFIDENCE_MIN ||
        (now_ms - sample.rx_time_ms) > Q4_VISUAL_READY_AGE_MS)
    {
        return false;
    }
    /* Every autonomous trial starts from a real centre settle.  Require the
     * position and speed limits continuously, not on one lucky frame after a
     * residual recovery, before declaring the platform ready. */
    if ((Q4StaticRecovery_Abs(sample.position_cm) <= Q4_START_POSITION_TOLERANCE_CM) &&
        (Q4StaticRecovery_Abs(sample.velocity_cm_s) <= Q4_HOLD_SPEED_LIMIT_CM_S))
    {
        if (s_ready_center_since_ms == 0U)
        {
            s_ready_center_since_ms = now_ms;
        }
        return (now_ms - s_ready_center_since_ms) >= Q4_READY_CONFIRM_MS;
    }
    s_ready_center_since_ms = 0U;
    return false;
}

static bool Q4StaticRecovery_StartDirectRecovery(uint32_t now_ms)
{
    s_direct_recovery_active = true;
    s_recovery_breakaway_active = true;
    s_recovery_breakaway_limit_deg = Q4_RECOVERY_DIRECT_BREAKAWAY_DEG;
    s_recovery_breakaway_count = 0U;
    s_recovery_still_started_ms = 0U;
    s_recovery_breakaway_started_ms = now_ms;
    s_status.breakaway_count = 0U;
    s_status.acceptance_passed = false;
    s_status.target_reached = false;
    s_recovery_started_ms = now_ms;
    if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
        !BallActuator_PrepareAbsolutePosition(
            Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, BALL_CONTROL_BREAKAWAY_SPEED_RPM, Q4_MAX_CURRENT_MA))
    {
        return false;
    }
    BallOuterPd_Init(&s_recovery_pd);
    BallStateEstimator_Init(&s_recovery_estimator);
    s_have_recovery_sample = false;
    s_hold_started_ms = 0U;
    s_hold_candidate_ms = 0U;
    s_hold_candidate_ms = 0U;
    s_last_commanded_angle_deg = 0.0f;
    s_recovery_command_ready_ms = now_ms + Q4_RECOVERY_START_DELAY_MS;
    s_last_command_send_ms = 0U;
    Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
    return true;
}

static bool Q4StaticRecovery_SafeVision(void)
{
    VisionLinkSample sample;
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm))
    {
        return true;
    }
    (void)sample;
    return true;
}

void Q4StaticRecovery_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = Q4_STATIC_WAIT;
    s_random_state = 0x5A3CU;
    s_disturb_stage = 0U;
    s_disturb_started_ms = 0U;
    s_zero_stage = 0U;
    s_zero_started_ms = 0U;
    s_zero_last_command_ms = 0U;
    s_zero_nudge_sent = false;
    s_result_log_pending = false;
    BallOuterPd_Init(&s_recovery_pd);
    s_last_vision_rx_ms = 0U;
    s_last_valid_vision_ms = 0U;
    s_have_recovery_sample = false;
    s_hold_started_ms = 0U;
    s_last_commanded_angle_deg = 0.0f;
    s_recovery_command_ready_ms = 0U;
    s_last_command_send_ms = 0U;
    s_recovery_breakaway_active = false;
    s_direct_recovery_active = false;
    s_startup_zero_pending = true;
    s_recovery_breakaway_count = 0U;
    s_recovery_still_started_ms = 0U;
    s_recovery_breakaway_started_ms = 0U;
    s_recovery_started_ms = 0U;
    s_ready_center_since_ms = 0U;
    s_fault_zero_pending = false;
    s_recovery_breakaway_limit_deg = Q4_RECOVERY_BREAKAWAY_START_DEG;
    s_status.breakaway_count = 0U;
    s_status.acceptance_passed = false;
}

void Q4StaticRecovery_Task(uint32_t now_ms)
{
    if ((s_status.state == Q4_STATIC_FINISH) || (s_status.state == Q4_STATIC_FAULT))
    {
        return;
    }

    if (!Q4StaticRecovery_SafeVision())
    {
        Q4StaticRecovery_Fault(now_ms);
        return;
    }

    switch (s_status.state)
    {
    case Q4_STATIC_WAIT:
    {
        BallActuatorStatus actuator;
        VisionLinkSample sample;
        BallActuator_GetStatus(&actuator);
        /* On boot the actuator may still report the previous session
                 * angle while the vision sample is already off-centre.  Zero
                 * the motor session first; otherwise direct recovery can
                 * inherit a stale beam angle and create an invalid run=0. */
        if (s_startup_zero_pending)
        {
            if (actuator.actual_angle_valid)
            {
                if (!Q4StaticRecovery_BeginZero(now_ms))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
            }
            break;
        }
        if (actuator.actual_angle_valid &&
            Q4StaticRecovery_Abs(actuator.actual_angle_deg) > Q4_ZERO_RETURN_TOLERANCE_DEG)
        {
            if (!Q4StaticRecovery_BeginZero(now_ms))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            break;
        }
        if (VisionLink_GetLatest(&sample) && sample.valid && isfinite(sample.position_cm) &&
            sample.confidence_pct >= Q4_RECOVERY_CONFIDENCE_MIN &&
            (now_ms - sample.rx_time_ms) <= Q4_VISUAL_READY_AGE_MS &&
            Q4StaticRecovery_Abs(sample.position_cm) > 2.0f)
        {
            BallActuator_SetCommandSession(true);
            if (!Q4StaticRecovery_StartDirectRecovery(now_ms))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            break;
        }
    }
        if (((now_ms - s_status.state_started_ms) >= Q4_NEXT_RUN_GAP_MS) &&
            Q4StaticRecovery_Ready(now_ms))
        {
            /* LFSR-like deterministic sequence: only the direction is
                 * pseudo-random direction and magnitude.  The magnitude is
                 * bounded by the measured +/-18 degree safe test limit. */
            s_random_state = (uint16_t)(s_random_state * 25173U + 13849U);
            {
                uint16_t span10 =
                    (uint16_t)((Q4_DISTURBANCE_MAX_ANGLE_DEG - Q4_DISTURBANCE_MIN_ANGLE_DEG) *
                                   10.0f +
                               0.5f);
                uint16_t magnitude10 = (uint16_t)(Q4_DISTURBANCE_MIN_ANGLE_DEG * 10.0f + 0.5f);
                if (span10 > 0U)
                {
                    magnitude10 = (uint16_t)(magnitude10 + (s_random_state % (span10 + 1U)));
                }
                /* Positive motor angle drives the ball toward visual
                     * -5 cm; the opposite sign tests the other side. */
                s_status.disturbance_angle_deg = ((s_random_state & 1U) != 0U)
                                                     ? (float)magnitude10 / 10.0f
                                                     : -(float)magnitude10 / 10.0f;
            }
            BallActuator_SetCommandSession(true);
            /* X42S needs a command gap between enable, F1 parameter and
                 * CD position.  Sending all three in one UART burst can
                 * leave the motor at its previous angle while the state
                 * machine falsely believes the disturbance started. */
            if (!BallActuator_ClearProtection() || !BallActuator_Enable(true))
            {
                Q4StaticRecovery_Fault(now_ms);
            }
            else
            {
                ++s_status.run_sequence;
                s_ready_center_since_ms = 0U;
                s_disturb_stage = 1U;
                s_disturb_started_ms = 0U;
                Q4StaticRecovery_Enter(Q4_STATIC_DISTURB, now_ms);
            }
        }
        break;

    case Q4_STATIC_DISTURB:
        if (s_disturb_stage == 1U)
        {
            if ((now_ms - s_status.state_started_ms) >= 100U)
            {
                if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
                    !BallActuator_PrepareAbsolutePosition(
                        Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_disturb_stage = 2U;
                    s_status.state_started_ms = now_ms;
                }
            }
        }
        else if (s_disturb_stage == 2U)
        {
            if ((now_ms - s_status.state_started_ms) >= 20U)
            {
                if (!BallActuator_SetAbsoluteAngleDeg(s_status.disturbance_angle_deg))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_disturb_stage = 3U;
                    s_disturb_started_ms = now_ms;
                }
            }
        }
        else if ((s_disturb_stage == 3U) &&
                 ((now_ms - s_disturb_started_ms) >= Q4_DISTURBANCE_DURATION_MS))
        {
            BallActuatorStatus actuator;
            BallActuator_GetStatus(&actuator);
            if (actuator.actual_angle_valid &&
                (Q4StaticRecovery_Abs(actuator.actual_angle_deg - s_status.disturbance_angle_deg) <=
                 1.0f))
            {
                if (!BallActuator_PrepareAbsolutePosition(Q4_ACCEL_RPM_S,
                                                          Q4_DECEL_RPM_S,
                                                          BALL_CONTROL_BREAKAWAY_SPEED_RPM,
                                                          Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    BallOuterPd_Init(&s_recovery_pd);
                    BallStateEstimator_Init(&s_recovery_estimator);
                    s_direct_recovery_active = false;
                    s_recovery_breakaway_active = true;
                    s_recovery_breakaway_limit_deg = Q4_RECOVERY_BREAKAWAY_START_DEG;
                    s_recovery_breakaway_count = 0U;
                    s_recovery_still_started_ms = 0U;
                    s_recovery_breakaway_started_ms = now_ms;
                    s_status.breakaway_count = 0U;
                    s_status.acceptance_passed = false;
                    s_status.target_reached = false;
                    s_recovery_started_ms = now_ms;
                    s_have_recovery_sample = false;
                    s_last_valid_vision_ms = now_ms;
                    s_hold_started_ms = 0U;
                    s_hold_candidate_ms = 0U;
                    s_last_commanded_angle_deg = 0.0f;
                    s_recovery_command_ready_ms = now_ms + Q4_RECOVERY_START_DELAY_MS;
                    s_last_command_send_ms = 0U;
                    Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
                }
            }
            else if ((now_ms - s_disturb_started_ms) >= Q4_DISTURBANCE_VERIFY_TIMEOUT_MS)
            {
                s_status.fault_reason = 7U;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    case Q4_STATIC_RECOVER:
        if (!Q4StaticRecovery_UpdateControl(now_ms))
        {
            Q4StaticRecovery_Fault(now_ms);
        }
        else if (s_status.target_reached)
        {
            if (s_hold_candidate_ms == 0U)
            {
                s_hold_candidate_ms = now_ms;
            }
            else if ((now_ms - s_hold_candidate_ms) >= Q4_HOLD_CONFIRM_MS)
            {
                s_status.acceptance_passed =
                    (s_recovery_started_ms != 0U) &&
                    ((now_ms - s_recovery_started_ms) <= Q4_RECOVERY_ACCEPT_MS);
                s_hold_started_ms = now_ms;
                Q4StaticRecovery_Enter(Q4_STATIC_HOLD, now_ms);
            }
        }
        else
        {
            s_hold_candidate_ms = 0U;
            if ((now_ms - s_status.state_started_ms) >= Q4_RECOVERY_TIMEOUT_MS)
            {
                s_status.fault_reason = 4U;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    case Q4_STATIC_HOLD:
        if (!Q4StaticRecovery_UpdateControl(now_ms))
        {
            Q4StaticRecovery_Fault(now_ms);
        }
        else
        {
            if (s_status.target_reached)
            {
                if ((now_ms - s_hold_started_ms) >= Q4_HOLD_BEFORE_ZERO_MS)
                {
                    if (!Q4StaticRecovery_BeginZero(now_ms))
                    {
                        Q4StaticRecovery_Fault(now_ms);
                    }
                    else
                    {
                        /* BeginZero owns the staged return command. */
                    }
                }
            }
            else
            {
                s_hold_candidate_ms = 0U;
                Q4StaticRecovery_Enter(Q4_STATIC_RECOVER, now_ms);
            }
        }
        break;

    case Q4_STATIC_RETURN_ZERO:
        if (s_zero_stage == 1U)
        {
            if ((now_ms - s_status.state_started_ms) >= 100U)
            {
                if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
                    !BallActuator_PrepareAbsolutePosition(
                        Q4_ACCEL_RPM_S, Q4_DECEL_RPM_S, Q4_MAX_SPEED_RPM, Q4_MAX_CURRENT_MA))
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_zero_stage = 2U;
                    s_status.state_started_ms = now_ms;
                }
            }
        }
        else if (s_zero_stage == 2U)
        {
            if ((now_ms - s_status.state_started_ms) >= 20U)
            {
                if (!BallActuator_ReturnAbsoluteZero())
                {
                    Q4StaticRecovery_Fault(now_ms);
                }
                else
                {
                    s_zero_stage = 3U;
                    s_zero_started_ms = now_ms;
                    s_zero_last_command_ms = now_ms;
                    s_status.commanded_angle_deg = 0.0f;
                }
            }
        }
        else if (s_zero_stage == 3U)
        {
            BallActuatorStatus actuator;
            BallActuator_GetStatus(&actuator);
            if (!s_zero_nudge_sent && ((now_ms - s_zero_started_ms) >= 500U) &&
                actuator.actual_angle_valid &&
                (Q4StaticRecovery_Abs(actuator.actual_angle_deg) > Q4_ZERO_RETURN_TOLERANCE_DEG))
            {
                /* A small opposite-side target crosses the X42S static
                     * position dead zone; the following 9A/04 command then
                     * settles at the stored mechanical zero. */
                float nudge =
                    (actuator.actual_angle_deg > 0.0f) ? -Q4_ZERO_NUDGE_DEG : Q4_ZERO_NUDGE_DEG;
                if (BallActuator_SetAbsoluteAngleDeg(nudge))
                {
                    s_zero_nudge_sent = true;
                    s_zero_last_command_ms = now_ms;
                }
            }
            else if ((now_ms - s_zero_last_command_ms) >= 250U)
            {
                /* CD is idempotent.  Retry the session-zero target while
                     * waiting for feedback so one lost UART frame cannot
                     * turn a normal return into a false Q4 fault. */
                if (BallActuator_ReturnAbsoluteZero())
                {
                    s_zero_last_command_ms = now_ms;
                    s_status.commanded_angle_deg = 0.0f;
                }
            }
            if (actuator.actual_angle_valid &&
                Q4StaticRecovery_Abs(actuator.actual_angle_deg) <= Q4_ZERO_RETURN_TOLERANCE_DEG)
            {
                BallActuator_SetCommandSession(false);
                s_startup_zero_pending = false;
                if (s_fault_zero_pending)
                {
                    s_fault_zero_pending = false;
                    Q4StaticRecovery_Enter(Q4_STATIC_FAULT, now_ms);
                }
                else
                {
                    Q4StaticRecovery_Enter(Q4_STATIC_WAIT, now_ms);
                }
            }
            else if ((now_ms - s_zero_started_ms) >= Q4_ZERO_RETURN_TIMEOUT_MS)
            {
                s_status.fault_reason = 5U;
                (void)BallActuator_Stop();
                (void)BallActuator_Enable(false);
                BallActuator_SetCommandSession(false);
                s_fault_zero_pending = false;
                Q4StaticRecovery_Fault(now_ms);
            }
        }
        break;

    default:
        break;
    }
}

void Q4StaticRecovery_Stop(uint32_t now_ms)
{
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    Q4StaticRecovery_Enter(Q4_STATIC_FINISH, now_ms);
}

bool Q4StaticRecovery_IsActive(void)
{
    return (s_status.state != Q4_STATIC_FINISH) && (s_status.state != Q4_STATIC_FAULT);
}

bool Q4StaticRecovery_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void Q4StaticRecovery_GetStatus(Q4StaticStatus *status)
{
    if (status != NULL)
        *status = s_status;
}

const char *Q4StaticRecovery_StateName(Q4StaticState state)
{
    switch (state)
    {
    case Q4_STATIC_WAIT:
        return "WAIT";
    case Q4_STATIC_DISTURB:
        return "DISTURB";
    case Q4_STATIC_RECOVER:
        return "RECOVER";
    case Q4_STATIC_HOLD:
        return "HOLD0";
    case Q4_STATIC_RETURN_ZERO:
        return "ZERO";
    case Q4_STATIC_FINISH:
        return "FINISH";
    case Q4_STATIC_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

#endif /* Q4_STATIC_RECOVERY_TEST_ENABLED */
