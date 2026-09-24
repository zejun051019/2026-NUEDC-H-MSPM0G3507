#include "q45_vehicle_control.h"

#include <math.h>
#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/ball_outer_pd.h"
#include "app/q3/ball_state_estimator.h"
#include "app/q3/pendulum_geometry.h"
#include "app/q3/q3_control_config.h"
#include "app/q6/q6_config.h"
#include "app/q6/q6_target.h"
#include "bsp/control.h"
#include "code/encoder.h"
#include "code/licheng.h"
#include "code/vision_link.h"
#include "q5_vehicle_model.h"
#include "q45_vehicle_config.h"
#include "q4_control_config.h"

enum
{
    Q45_STOP_NONE = 0U,
    Q45_STOP_ROUTE_COMPLETE,
    Q45_STOP_TIME_LIMIT,
    Q45_STOP_VISION,
    Q45_STOP_ACTUATOR,
    Q45_STOP_BALL_SAFETY,
    Q45_STOP_OPERATOR
};

static Q45VehicleStatus s_status;
static BallStateEstimator s_estimator;
static uint32_t s_stage_started_ms;
static uint32_t s_last_vision_rx_ms;
static uint32_t s_last_processed_vision_rx_ms;
static uint32_t s_last_valid_vision_ms;
static uint32_t s_last_command_ms;
static uint32_t s_still_since_ms;
static uint32_t s_stiction_started_ms;
static float s_last_command_deg;
static bool s_have_sample;
static bool s_stiction_active;
static bool s_zero_command_sent;
static bool s_origin_zero_sent;
static uint32_t s_terminal_zero_retry_ms;
static bool s_terminal_actuator_paused;
static bool s_result_log_pending;
static uint32_t s_run_sequence;
static Q5VehicleModel s_chassis_model;
static float s_q5_bias_integral_deg;
static float s_q5_second_turn_integral_deg;
/* Captured when the formal run starts. The displayed/logged maximum is relative
 * to this actual departure sample, not an assumed mathematical zero. */
static float s_run_start_position_cm;
/* Q6: the hold target is a plain coordinate shift.  Q4/Q5 keep 0 so every
 * frozen absolute-position law stays bit-identical for them. */
static float s_target_cm;
static Q6TargetSource s_q6_source;
static float s_q6_explicit_cm;

static uint32_t Q45Vehicle_VisionTimeoutMs(void);

static float Q45Vehicle_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float Q45Vehicle_Clamp(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static float Q45Vehicle_SmoothStep(float value)
{
    float t = Q45Vehicle_Clamp(value, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

static float Q45Vehicle_Q5LateStraightStaticGain(void)
{
    float distance_cm;
    float start_cm;

    if (s_status.mode != Q45_VEHICLE_MODE_Q5)
    {
        return 1.0f;
    }

    distance_cm = s_status.route_distance_cm;
    if ((distance_cm >= Q45_Q5_FIRST_STRAIGHT_LATE_START_CM) &&
        (distance_cm < Q45_Q5_FIRST_TURN_START_CM))
    {
        start_cm = Q45_Q5_FIRST_STRAIGHT_LATE_START_CM;
    }
    else if ((distance_cm >= Q45_Q5_SECOND_STRAIGHT_LATE_START_CM) &&
             (distance_cm < Q45_Q5_SECOND_TURN_START_CM))
    {
        start_cm = Q45_Q5_SECOND_STRAIGHT_LATE_START_CM;
    }
    else
    {
        return 1.0f;
    }

    return 1.0f +
           (Q45_Q5_LATE_STRAIGHT_STATIC_GAIN - 1.0f) *
               Q45Vehicle_SmoothStep((distance_cm - start_cm) / Q45_Q5_LATE_STRAIGHT_GAIN_BLEND_CM);
}

/* Q6 reuses every dynamic-mode (Q5) path: vision watchdog, actuator profile,
 * launch boost, feedforward, turn speed law and time limit.  Static Q4
 * keeps its exclusive proven settings. */
static bool Q45Vehicle_IsDynamicMode(void)
{
    return (s_status.mode == Q45_VEHICLE_MODE_Q5) || (s_status.mode == Q45_VEHICLE_MODE_Q6);
}

static void Q45Vehicle_ResetDynamicModel(void)
{
    Q5VehicleModel_Init(&s_chassis_model);
    s_q5_bias_integral_deg = 0.0f;
    s_q5_second_turn_integral_deg = 0.0f;
    s_status.actual_speed_mm_s = 0.0f;
    s_status.longitudinal_accel_m_s2 = 0.0f;
    s_status.modeled_turn_rate_deg_s = 0.0f;
    s_status.lateral_accel_m_s2 = 0.0f;
    s_status.parallel_accel_m_s2 = 0.0f;
    s_status.speed_governor_level = 0U;
}

/* Remove the small residual left after acceleration feedforward goes to zero.
 * The trim is inactive while a turn or fast ball transient is present, and
 * leaks away instead of carrying one bend's bias into the next. */
static float Q45Vehicle_UpdateQ5BiasIntegral(float position_cm, float velocity_cm_s, float dt_s)
{
    bool integrate;
    float correction_beam_deg;
    float integral_limit_deg;

    integrate = Q45Vehicle_IsDynamicMode() &&
                (Q45Vehicle_Abs(position_cm) >= ((s_status.mode == Q45_VEHICLE_MODE_Q6)
                                                     ? Q6_STICTION_ENGAGE_CM
                                                     : Q45_Q5_INTEGRAL_ACTIVE_POSITION_CM)) &&
                (Q45Vehicle_Abs(velocity_cm_s) <= Q45_Q5_INTEGRAL_ACTIVE_SPEED_CM_S) &&
                (Q45Vehicle_Abs(s_status.parallel_accel_m_s2) <= Q45_Q5_INTEGRAL_ACTIVE_ACCEL_M_S2);
    if (integrate)
    {
        correction_beam_deg =
            Q3_POSITION_TO_MOTOR_SIGN * Q45_Q5_KI_BEAM_DEG_PER_CM_S * (-position_cm);
        s_q5_bias_integral_deg += correction_beam_deg * dt_s;
    }
    else if ((s_status.mode != Q45_VEHICLE_MODE_Q6) ||
             (Q45Vehicle_Abs(position_cm) >= Q6_STICTION_ENGAGE_CM))
    {
        s_q5_bias_integral_deg *=
            Q45Vehicle_Clamp(1.0f - Q45_Q5_INTEGRAL_LEAK_PER_S * dt_s, 0.0f, 1.0f);
    }
    /* Q6 parked inside the engage window: freeze the learned beam bias.
     * Leakage inside the scoring band can erase compensation for the
     * absolute-zero pipe slope and slowly drive the ball off target. */
    integral_limit_deg =
        (s_status.mode == Q45_VEHICLE_MODE_Q6) ? Q6_INTEGRAL_LIMIT_DEG : Q45_Q5_INTEGRAL_LIMIT_DEG;
    s_q5_bias_integral_deg =
        Q45Vehicle_Clamp(s_q5_bias_integral_deg, -integral_limit_deg, integral_limit_deg);
    return s_q5_bias_integral_deg;
}

/* Foreground-only model work.  The chassis IRQ owns the original speed ramp,
 * line following and terminal-stop check, independent of vision position. */
static void Q45Vehicle_UpdateChassisModel(uint32_t now_ms)
{
    if (!Q45Vehicle_IsDynamicMode())
        return;
    if (!(s_status.mode == Q45_VEHICLE_MODE_Q5
              ? Q5VehicleModel_UpdateFinalQ5(&s_chassis_model,
                                             now_ms,
                                             E_speed[MOTOR_LEFT],
                                             E_speed[MOTOR_RIGHT],
                                             s_status.route_distance_cm)
              : Q5VehicleModel_Update(&s_chassis_model,
                                      now_ms,
                                      E_speed[MOTOR_LEFT],
                                      E_speed[MOTOR_RIGHT],
                                      s_status.route_distance_cm)))
        return;
    s_status.actual_speed_mm_s = s_chassis_model.speed_mm_s;
    s_status.longitudinal_accel_m_s2 = s_chassis_model.longitudinal_accel_m_s2;
    s_status.lateral_accel_m_s2 = s_chassis_model.lateral_accel_m_s2;
    s_status.parallel_accel_m_s2 = s_chassis_model.parallel_accel_m_s2;
    s_status.modeled_turn_rate_deg_s =
        s_chassis_model.turn_blend * (s_chassis_model.speed_mm_s * 0.001f / Q45_Q5_TURN_RADIUS_M) *
        57.2957795f;
}

/* Q5 changes the proven static-Q4 beam request by this one additive term
 * only.  The PD profile selection, brake distance, actuator period and X42S
 * limits below remain the Q4 path. */
static float Q45Vehicle_Q5FeedforwardBeamDeg(void)
{
    float beam_deg;

    if (!Q45Vehicle_IsDynamicMode() || (Q45_ACCELERATION_FEEDFORWARD_ENABLED == 0U))
    {
        return 0.0f;
    }
    beam_deg = Q45_Q5_FEEDFORWARD_BEAM_SIGN * Q45_Q5_FEEDFORWARD_GAIN *
               atanf(s_status.parallel_accel_m_s2 / Q45_Q5_GRAVITY_M_S2) * 57.2957795131f;
    return Q45Vehicle_Clamp(beam_deg, -Q45_Q5_FEEDFORWARD_LIMIT_DEG, Q45_Q5_FEEDFORWARD_LIMIT_DEG);
}

static float Q45Vehicle_UpdateQ5SecondTurnFeedback(float position_cm,
                                                   float velocity_cm_s,
                                                   uint8_t vision_confidence,
                                                   float dt_s)
{
    float distance_cm = s_status.route_distance_cm;
    float error_cm = 0.0f;
    float entry_blend;
    float p_beam_deg;

    if ((s_status.mode != Q45_VEHICLE_MODE_Q5) || (s_status.state != Q45_VEHICLE_RUNNING) ||
        (distance_cm < Q45_Q5_SECOND_FB_START_CM) || (distance_cm > Q45_Q5_SECOND_FB_END_CM))
    {
        s_q5_second_turn_integral_deg = 0.0f;
        return 0.0f;
    }

    entry_blend = Q45Vehicle_SmoothStep((distance_cm - Q45_Q5_SECOND_FB_START_CM) /
                                        (Q45_Q5_SECOND_FB_FULL_CM - Q45_Q5_SECOND_FB_START_CM));
    if (position_cm > Q45_Q5_SECOND_FB_DEADBAND_CM)
    {
        error_cm = position_cm - Q45_Q5_SECOND_FB_DEADBAND_CM;
    }
    else if (position_cm < -Q45_Q5_SECOND_FB_DEADBAND_CM)
    {
        error_cm = position_cm + Q45_Q5_SECOND_FB_DEADBAND_CM;
    }
    p_beam_deg = Q45Vehicle_Clamp(Q45_Q5_SECOND_FB_KP_DEG_PER_CM * error_cm,
                                  -Q45_Q5_SECOND_FB_P_LIMIT_DEG,
                                  Q45_Q5_SECOND_FB_P_LIMIT_DEG) *
                 entry_blend;

    if ((error_cm * s_q5_second_turn_integral_deg) < 0.0f)
    {
        s_q5_second_turn_integral_deg *= Q45Vehicle_Clamp(1.0f - 5.0f * dt_s, 0.0f, 1.0f);
    }
    if ((error_cm != 0.0f) && (vision_confidence >= Q45_Q5_SECOND_FB_I_CONF_MIN) &&
        (Q45Vehicle_Abs(velocity_cm_s) <= Q45_Q5_SECOND_FB_I_MAX_SPEED_CM_S) &&
        !s_status.control_output_limited)
    {
        s_q5_second_turn_integral_deg +=
            Q45_Q5_SECOND_FB_KI_DEG_PER_CM_S * error_cm * dt_s * entry_blend;
    }
    else
    {
        s_q5_second_turn_integral_deg *=
            Q45Vehicle_Clamp(1.0f - Q45_Q5_SECOND_FB_I_LEAK_PER_S * dt_s, 0.0f, 1.0f);
    }
    s_q5_second_turn_integral_deg = Q45Vehicle_Clamp(
        s_q5_second_turn_integral_deg, -Q45_Q5_SECOND_FB_I_LIMIT_DEG, Q45_Q5_SECOND_FB_I_LIMIT_DEG);
    return p_beam_deg + s_q5_second_turn_integral_deg * entry_blend;
}

static float
Q45Vehicle_Q5FirstTurnPositionTrim(float position_cm, float velocity_cm_s, uint8_t confidence)
{
    float distance_cm = s_status.route_distance_cm;
    float error_cm = 0.0f;
    float blend;
    float velocity_trim_deg;

    if ((s_status.mode != Q45_VEHICLE_MODE_Q5) || (s_status.state != Q45_VEHICLE_RUNNING) ||
        (confidence < Q45_Q5_FIRST_FB_CONF_MIN) || (distance_cm < Q45_Q5_FIRST_FB_START_CM) ||
        (distance_cm > Q45_Q5_FIRST_FB_END_CM))
    {
        return 0.0f;
    }
    if (position_cm > Q45_Q5_FIRST_FB_DEADBAND_CM)
    {
        error_cm = position_cm - Q45_Q5_FIRST_FB_DEADBAND_CM;
    }
    else if (position_cm < -Q45_Q5_FIRST_FB_DEADBAND_CM)
    {
        error_cm = position_cm + Q45_Q5_FIRST_FB_DEADBAND_CM;
    }
    blend = Q45Vehicle_SmoothStep((distance_cm - Q45_Q5_FIRST_FB_START_CM) /
                                  (Q45_Q5_FIRST_FB_FULL_CM - Q45_Q5_FIRST_FB_START_CM));
    blend *= 1.0f - Q45Vehicle_SmoothStep((distance_cm - Q45_Q5_FIRST_FB_FADE_START_CM) /
                                          (Q45_Q5_FIRST_FB_END_CM - Q45_Q5_FIRST_FB_FADE_START_CM));
    velocity_trim_deg = Q45Vehicle_Clamp(Q45_Q5_FIRST_FB_KD_DEG_PER_CM_S * velocity_cm_s,
                                         -Q45_Q5_FIRST_FB_D_LIMIT_DEG,
                                         Q45_Q5_FIRST_FB_D_LIMIT_DEG);
    return Q45Vehicle_Clamp(Q45_Q5_FIRST_FB_KP_DEG_PER_CM * error_cm + velocity_trim_deg,
                            -Q45_Q5_FIRST_FB_LIMIT_DEG,
                            Q45_Q5_FIRST_FB_LIMIT_DEG) *
           blend;
}

/* Qualification for the operator's second press.  Position is deliberately
 * the only motion criterion: the UI contract is vision-confirmed |x| <= 1 cm,
 * while the feedback loop manages residual speed after formal timing starts. */
/* Q4/Q5 retain the frozen absolute wall guard. Q6 must be allowed to
 * recover from any captured position without turning this boundary check
 * into a terminal stop; its controller and vision watchdog remain active. */
static bool Q45Vehicle_BallSafetyFault(float position_cm, float velocity_cm_s)
{
    (void)velocity_cm_s;
    if (s_status.mode == Q45_VEHICLE_MODE_Q6)
    {
        return false;
    }
    return Q45Vehicle_Abs(position_cm) > Q45_BALL_SAFETY_POSITION_CM;
}

static bool Q45Vehicle_UpdateCenterReady(uint32_t now_ms)
{
    VisionLinkSample sample;
    uint8_t confidence_min =
        Q45Vehicle_IsDynamicMode() ? Q45_Q5_CONFIDENCE_MIN : Q4_RECOVERY_CONFIDENCE_MIN;

    s_status.center_ready = false;
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        (sample.confidence_pct < confidence_min) ||
        ((now_ms - sample.rx_time_ms) > Q45Vehicle_VisionTimeoutMs()) ||
        ((s_status.mode == Q45_VEHICLE_MODE_Q6) && !Q6Target_IsPixelRowAccepted(sample.pixel_cy)))
    {
        return false;
    }

    s_status.measured_cm = sample.position_cm;
    s_status.raw_position_cm = sample.position_cm;
    s_status.measured_velocity_cm_s = sample.velocity_cm_s;
    s_status.vision_confidence_pct = sample.confidence_pct;
    s_status.vision_sequence = sample.sequence;
    s_status.vision_age_ms = now_ms - sample.rx_time_ms;
    if (s_status.mode == Q45_VEHICLE_MODE_Q6)
    {
        /* Q6 readiness is "hold target captured", not "ball at midpoint":
         * feed every qualifying fresh frame to the capture window and never
         * require O here. */
        Q6TargetFrame frame;
        Q6TargetStatus target;

        frame.position_cm = sample.position_cm;
        frame.velocity_cm_s = sample.velocity_cm_s;
        frame.rx_time_ms = sample.rx_time_ms;
        frame.source_timestamp_ms = sample.source_timestamp_ms;
        frame.confidence_pct = sample.confidence_pct;
        frame.sequence = sample.sequence;
        frame.valid = sample.valid;
        Q6Target_Update(&frame, now_ms);
        Q6Target_GetStatus(&target);
        s_status.center_ready = target.ready && (Q45Vehicle_Abs(sample.position_cm) <=
                                                 Q6_EXPLICIT_CAPTURE_MAX_POSITION_CM);
        return s_status.center_ready;
    }
    s_status.center_ready =
        (Q45Vehicle_Abs(sample.position_cm) <= Q45_CENTER_POSITION_TOLERANCE_CM);
    return s_status.center_ready;
}

static bool Q45Vehicle_IsTerminal(Q45VehicleState state)
{
    return (state == Q45_VEHICLE_FINISHED) || (state == Q45_VEHICLE_TIMEOUT) ||
           (state == Q45_VEHICLE_VISION_LOST) || (state == Q45_VEHICLE_FAULT) ||
           (state == Q45_VEHICLE_STOPPED);
}

static void Q45Vehicle_Enter(Q45VehicleState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
}

static void Q45Vehicle_RequestStop(Q45VehicleState terminal_state, uint8_t reason, uint32_t now_ms)
{
    if (Q45Vehicle_IsTerminal(s_status.state))
    {
        return;
    }

    s_status.stop_requested = true;
    s_status.stop_reason = reason;
    s_status.fault =
        (terminal_state == Q45_VEHICLE_FAULT) || (terminal_state == Q45_VEHICLE_VISION_LOST);
    Q45Vehicle_Enter(terminal_state, now_ms);
    s_result_log_pending = true;
}

static uint32_t Q45Vehicle_TimeLimitMs(void)
{
    return (s_status.mode == Q45_VEHICLE_MODE_Q4) ? Q45_Q4_TIME_LIMIT_MS : Q45_Q5_TIME_LIMIT_MS;
}

/* Static Q4 retains its proven strict visual watchdog.  A moving Q5 run can
 * briefly miss two K230 frames while the next valid frame is already queued;
 * use its separate, still finite watchdog for that case. */
static uint32_t Q45Vehicle_VisionTimeoutMs(void)
{
    return Q45Vehicle_IsDynamicMode() ? Q45_Q5_VISION_TIMEOUT_MS : Q45_VISION_TIMEOUT_MS;
}

static float Q45Vehicle_PassDistanceCm(void)
{
    return (s_status.mode == Q45_VEHICLE_MODE_Q4) ? Q45_Q4_B_DISTANCE_CM : Q45_Q5_A_DISTANCE_CM;
}

static float Q45Vehicle_StopDistanceCm(void)
{
    return (s_status.mode == Q45_VEHICLE_MODE_Q4) ? Q45_Q4_STOP_DISTANCE_CM
                                                  : Q45_Q5_STOP_DISTANCE_CM;
}

static float Q45Vehicle_TraceSpeedMmS(void)
{
    float cruise = (s_status.mode == Q45_VEHICLE_MODE_Q4) ? Q45_Q4_CRUISE_SPEED_MM_S
                                                          : Q45_Q5_CRUISE_SPEED_MM_S;
    uint32_t ramp_ms =
        (s_status.mode == Q45_VEHICLE_MODE_Q4) ? Q45_Q4_ACCEL_TIME_MS : Q45_Q5_ACCEL_TIME_MS;
    float speed = cruise;

    if ((ramp_ms != 0U) && (s_status.drive_elapsed_ms < ramp_ms))
    {
        speed = cruise * (float)s_status.drive_elapsed_ms / (float)ramp_ms;
    }

    /* Only Q4 stops after B.  Q5 must remain at cruise through A. */
    if ((s_status.mode == Q45_VEHICLE_MODE_Q4) &&
        (s_status.route_distance_cm >= Q45_Q4_B_DISTANCE_CM))
    {
        float remaining = Q45_Q4_STOP_DISTANCE_CM - s_status.route_distance_cm;
        float span = Q45_Q4_STOP_DISTANCE_CM - Q45_Q4_B_DISTANCE_CM;
        speed = cruise * Q45Vehicle_Clamp(remaining / span, 0.0f, 1.0f);
    }

    if (Q45Vehicle_IsDynamicMode())
    {
        return Q5VehicleModel_TraceSpeed(s_status.drive_elapsed_ms);
    }
    return speed;
}

/* This is the measured static-Q4 fast PD law, reused unchanged in gain,
 * stopping-distance selection, and X42S safety limit.  It intentionally has
 * no vehicle-acceleration feedforward term. */
static BallOuterPdProfile Q45Vehicle_SelectProfile(void)
{
    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        float position_cm = s_estimator.position_cm;
        float distance_cm = Q45Vehicle_Abs(position_cm);
        float speed_abs = Q45Vehicle_Abs(s_estimator.velocity_cm_s);
        float speed_toward_cm_s = 0.0f;
        float fallback_cm =
            (position_cm > 0.0f) ? Q45_Q5_BRAKE_FALLBACK_POS_CM : Q45_Q5_BRAKE_FALLBACK_CM;
        float stopping_distance_cm;

        if ((position_cm * s_estimator.velocity_cm_s) < 0.0f)
        {
            speed_toward_cm_s = speed_abs;
        }
        stopping_distance_cm =
            speed_toward_cm_s * ((float)Q45_Q5_FEEDBACK_DELAY_MS / 1000.0f) +
            (speed_toward_cm_s * speed_toward_cm_s) / (2.0f * Q45_Q5_DECELERATION_CM_S2) +
            Q45_Q5_BRAKE_MARGIN_PROFILE_CM;
        if ((distance_cm <= Q45_Q5_HOLD_POSITION_TOLERANCE_CM) &&
            (speed_abs <= Q45_Q5_HOLD_SPEED_LIMIT_CM_S))
        {
            return BALL_OUTER_PROFILE_RETURN_HOLD;
        }
        if ((distance_cm <= stopping_distance_cm) || (distance_cm <= fallback_cm))
        {
            return BALL_OUTER_PROFILE_RETURN_BRAKE;
        }
        return BALL_OUTER_PROFILE_RETURN_CRUISE;
    }

    /* The Q4/Q6 branch uses target-relative position; the Q5 branch above
     * retains its original zero-centred route-control coordinate. */
    float position_cm = s_estimator.position_cm - s_target_cm;
    float distance_cm = Q45Vehicle_Abs(position_cm);
    float speed_abs = Q45Vehicle_Abs(s_estimator.velocity_cm_s);
    float speed_toward_cm_s = 0.0f;
    float fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM;
    float stopping_distance_cm;

    if (s_status.mode == Q45_VEHICLE_MODE_Q6)
    {
        fallback_cm = Q6_BRAKE_FALLBACK_CM;
    }
    else if (position_cm > 0.0f)
    {
        fallback_cm = Q4_RECOVERY_BRAKE_FALLBACK_CM_ERROR_NEG;
    }
    if ((position_cm * s_estimator.velocity_cm_s) < 0.0f)
    {
        speed_toward_cm_s = speed_abs;
    }
    stopping_distance_cm =
        speed_toward_cm_s * ((float)Q3_RETURN_FEEDBACK_DELAY_MS / 1000.0f) +
        (speed_toward_cm_s * speed_toward_cm_s) / (2.0f * Q3_RETURN_DECELERATION_CM_S2) +
        Q4_RECOVERY_BRAKE_MARGIN_CM;

    if ((distance_cm <= Q45_CENTER_POSITION_TOLERANCE_CM) &&
        (speed_abs <= Q45_CENTER_SPEED_LIMIT_CM_S))
    {
        return BALL_OUTER_PROFILE_RETURN_HOLD;
    }
    if ((distance_cm <= stopping_distance_cm) || (distance_cm <= fallback_cm))
    {
        return BALL_OUTER_PROFILE_RETURN_BRAKE;
    }
    return BALL_OUTER_PROFILE_RETURN_CRUISE;
}

static bool Q45Vehicle_ComputeFastMotor(float error_cm,
                                        float velocity_cm_s,
                                        BallOuterPdProfile profile,
                                        float *motor_deg)
{
    float kp;
    float kd;
    float beam_limit;
    float beam_deg;

    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        kp = (error_cm > 0.0f) ? Q45_Q5_KP_ERROR_POS : Q45_Q5_KP_BEAM_DEG_PER_CM;
        kd = (error_cm > 0.0f) ? Q45_Q5_KD_ERROR_POS : Q45_Q5_KD_BEAM_DEG_PER_CM_S;
        beam_limit = Q45_Q5_CRUISE_BEAM_LIMIT_DEG;
        if (profile == BALL_OUTER_PROFILE_RETURN_BRAKE)
        {
            kp = Q45_Q5_BRAKE_KP;
            kd = ((error_cm * velocity_cm_s) < 0.0f) ? Q45_Q5_BRAKE_KD_OUTWARD : Q45_Q5_BRAKE_KD;
            beam_limit =
                (error_cm < 0.0f) ? Q45_Q5_BRAKE_BEAM_LIMIT_NEG_DEG : Q45_Q5_BRAKE_BEAM_LIMIT_DEG;
        }
        else if (profile == BALL_OUTER_PROFILE_RETURN_HOLD)
        {
            kp = Q45_Q5_HOLD_KP;
            kd = Q45_Q5_HOLD_KD;
            beam_limit = Q45_Q5_HOLD_BEAM_LIMIT_DEG;
        }
        else
        {
            if (error_cm > 0.0f)
            {
                beam_limit = Q45_Q5_CRUISE_BEAM_LIMIT_POS_DEG;
                kd = Q45_Q5_KD_ERROR_POS;
            }
            else if (error_cm < 0.0f)
            {
                beam_limit = Q45_Q5_CRUISE_BEAM_LIMIT_NEG_DEG;
            }
            if ((profile == BALL_OUTER_PROFILE_RETURN_CRUISE) &&
                ((error_cm * velocity_cm_s) < 0.0f))
            {
                kd = Q45_Q5_CRUISE_KD_OUTWARD;
            }
        }

        beam_deg = Q45_POSITION_TO_MOTOR_SIGN * (kp * error_cm - kd * velocity_cm_s);
        beam_deg = Q45Vehicle_Clamp(beam_deg, -beam_limit, beam_limit);
        return PendulumGeometry_BeamDegToMotorDeg(beam_deg, motor_deg) &&
               (Q45Vehicle_Abs(*motor_deg) <= Q4_RECOVERY_MAX_ANGLE_DEG);
    }

    kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM;
    kd = Q4_RECOVERY_KD_BEAM_DEG_PER_CM_S;
    beam_limit = Q4_RECOVERY_BEAM_LIMIT_DEG;
    if (error_cm > 0.0f)
    {
        kp = Q4_RECOVERY_KP_BEAM_DEG_PER_CM_ERROR_POS;
    }
    if (profile == BALL_OUTER_PROFILE_RETURN_BRAKE)
    {
        kp = Q4_RECOVERY_BRAKE_KP_BEAM_DEG_PER_CM;
        kd = (s_status.mode == Q45_VEHICLE_MODE_Q6) ? Q6_BRAKE_KD_BEAM_DEG_PER_CM_S
                                                    : Q4_RECOVERY_BRAKE_KD_BEAM_DEG_PER_CM_S;
        if ((error_cm * velocity_cm_s) < 0.0f)
        {
            kd = (s_status.mode == Q45_VEHICLE_MODE_Q6)
                     ? Q6_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S
                     : Q4_RECOVERY_BRAKE_KD_OUTWARD_BEAM_DEG_PER_CM_S;
        }
        beam_limit = (error_cm < 0.0f) ? Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG_ERROR_NEG
                                       : Q4_RECOVERY_BRAKE_BEAM_LIMIT_DEG;
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
    beam_deg = Q45Vehicle_Clamp(beam_deg, -beam_limit, beam_limit);
    return PendulumGeometry_BeamDegToMotorDeg(beam_deg, motor_deg) &&
           (Q45Vehicle_Abs(*motor_deg) <= Q4_RECOVERY_MAX_ANGLE_DEG);
}

static void
Q45Vehicle_ApplyNearCenterLimit(float measured_cm, BallOuterPdProfile profile, float *motor_deg)
{
    float limit_motor_deg;

    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        if ((motor_deg == NULL) || (profile == BALL_OUTER_PROFILE_RETURN_HOLD) ||
            (Q45Vehicle_Abs(measured_cm) > Q45_Q5_MID_DISTANCE_CM) ||
            !PendulumGeometry_BeamDegToMotorDeg(Q45_Q5_MID_BEAM_LIMIT_DEG, &limit_motor_deg))
        {
            return;
        }
    }
    else if ((motor_deg == NULL) || (profile == BALL_OUTER_PROFILE_RETURN_HOLD) ||
             (Q45Vehicle_Abs(measured_cm) > Q4_RECOVERY_MID_DISTANCE_CM) ||
             !PendulumGeometry_BeamDegToMotorDeg(Q4_RECOVERY_MID_BEAM_LIMIT_DEG, &limit_motor_deg))
    {
        return;
    }

    if (Q45Vehicle_Abs(*motor_deg) > Q45Vehicle_Abs(limit_motor_deg))
    {
        *motor_deg = (*motor_deg < 0.0f) ? -limit_motor_deg : limit_motor_deg;
    }
}

static bool Q45Vehicle_UpdateBallControl(uint32_t now_ms)
{
    VisionLinkSample sample;
    BallActuatorStatus actuator;
    BallOuterPdProfile profile;
    float dt_s;
    float output_deg;
    float control_error_cm;
    float distance_cm;
    float speed_abs;
    float static_beam_deg;
    float integral_beam_deg;
    float feedforward_beam_deg;
    float second_turn_feedback_beam_deg;
    float first_turn_feedback_beam_deg;
    bool q5_moving_bend;
    uint32_t command_period_ms;
    uint8_t confidence_min =
        Q45Vehicle_IsDynamicMode() ? Q45_Q5_CONFIDENCE_MIN : Q4_RECOVERY_CONFIDENCE_MIN;

    /* A rejected measurement must not keep the accepted-state watchdog alive. */
    if ((now_ms - s_last_valid_vision_ms) > Q45Vehicle_VisionTimeoutMs())
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_VISION_LOST, Q45_STOP_VISION, now_ms);
        return false;
    }
    if (!VisionLink_GetLatest(&sample) || !sample.valid || !isfinite(sample.position_cm) ||
        !isfinite(sample.velocity_cm_s) || (sample.confidence_pct < confidence_min) ||
        ((now_ms - sample.rx_time_ms) > Q45Vehicle_VisionTimeoutMs()) ||
        ((s_status.mode == Q45_VEHICLE_MODE_Q6) && !Q6Target_IsPixelRowAccepted(sample.pixel_cy)))
    {
        if ((now_ms - s_last_valid_vision_ms) <= Q45Vehicle_VisionTimeoutMs())
        {
            return true;
        }
        Q45Vehicle_RequestStop(Q45_VEHICLE_VISION_LOST, Q45_STOP_VISION, now_ms);
        return false;
    }
    if ((s_last_processed_vision_rx_ms != 0U) &&
        (sample.rx_time_ms == s_last_processed_vision_rx_ms))
    {
        return true;
    }

    /* Record raw scoring evidence before the observer can reject a frame.
     * A raw boundary event cannot disappear behind a smooth estimated trace. */
    s_status.raw_position_cm = sample.position_cm;
    s_status.in_scoring_band =
        Q45Vehicle_Abs(sample.position_cm - s_target_cm) <= Q45_CENTER_POSITION_TOLERANCE_CM;
    s_status.center_ready = s_status.in_scoring_band;
    if (!s_status.in_scoring_band)
        s_status.band_violation = true;
    if (Q45Vehicle_Abs(sample.position_cm - s_run_start_position_cm) > s_status.max_abs_position_cm)
    {
        s_status.max_abs_position_cm = Q45Vehicle_Abs(sample.position_cm - s_run_start_position_cm);
    }
    if (Q45Vehicle_Abs(sample.position_cm - s_target_cm) > s_status.max_target_dev_cm)
    {
        s_status.max_target_dev_cm = Q45Vehicle_Abs(sample.position_cm - s_target_cm);
    }
    if (Q45Vehicle_BallSafetyFault(sample.position_cm, sample.velocity_cm_s))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_BALL_SAFETY, now_ms);
        return false;
    }
    if ((s_status.mode == Q45_VEHICLE_MODE_Q5) &&
        (s_status.route_distance_cm >= Q45_Q5_SECOND_FB_START_CM) &&
        (s_status.route_distance_cm <= Q45_Q5_SECOND_FB_END_CM) &&
        (sample.confidence_pct >= Q45_Q5_SECOND_FB_I_CONF_MIN) &&
        (Q45Vehicle_Abs(sample.position_cm) >= Q45_Q5_REVERSED_SECOND_TEST_ABORT_CM))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_BALL_SAFETY, now_ms);
        return false;
    }
    if ((s_status.mode == Q45_VEHICLE_MODE_Q5) &&
        (s_status.route_distance_cm >= Q45_Q5_FIRST_TURN_START_CM) &&
        (s_status.route_distance_cm <= Q45_Q5_FIRST_TURN_END_CM) &&
        (sample.confidence_pct >= Q45_Q5_FIRST_FB_CONF_MIN) &&
        (Q45Vehicle_Abs(sample.position_cm) >= Q45_Q5_FIRST_TRIAL_ABORT_CM))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_BALL_SAFETY, now_ms);
        return false;
    }
    BallActuator_GetStatus(&actuator);
    s_status.actual_angle_age_ms = now_ms - actuator.last_position_ms;
    if (!actuator.actual_angle_valid ||
        (Q45Vehicle_IsDynamicMode() && (Q45_Q5_ACTUAL_ANGLE_TIMEOUT_MS != 0U) &&
         (s_status.actual_angle_age_ms > Q45_Q5_ACTUAL_ANGLE_TIMEOUT_MS)))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
        return false;
    }

    dt_s = s_have_sample ? (float)(sample.rx_time_ms - s_last_vision_rx_ms) / 1000.0f : 0.02f;
    dt_s = Q45Vehicle_Clamp(dt_s, 0.01f, BALL_CONTROL_SAMPLE_MAX_S);
    if (!BallStateEstimator_Update(&s_estimator, sample.position_cm, dt_s))
    {
        /* Consume a rejected frame once.  Retrying one identical frame in the
         * foreground would incorrectly count it as three independent jumps
         * and force the alpha-beta observer to reacquire an outlier. */
        s_last_processed_vision_rx_ms = sample.rx_time_ms;
        ++s_status.rejected_frames;
        s_status.raw_position_cm = sample.position_cm;
        s_status.vision_confidence_pct = sample.confidence_pct;
        s_status.estimator_rejected_count = s_estimator.rejected_count;
        s_status.vision_sequence = sample.sequence;
        s_status.vision_age_ms = now_ms - sample.rx_time_ms;
        return true;
    }

    distance_cm = Q45Vehicle_Abs(s_estimator.position_cm - s_target_cm);
    speed_abs = Q45Vehicle_Abs(s_estimator.velocity_cm_s);
    /* Safety is the physical pipe range, not the distance-to-target: with a
     * Q6 explicit hold target up to +/-5 cm, a ball legitimately travelling
     * e.g. from -9.4 cm to +5 cm would otherwise trip a 14.4 cm "deviation"
     * immediately after lock.  Q4/Q5 keep identical behavior because their
     * target is 0, where relative and absolute checks coincide. */
    if (Q45Vehicle_BallSafetyFault(s_estimator.position_cm, s_estimator.velocity_cm_s))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_BALL_SAFETY, now_ms);
        return false;
    }

    control_error_cm = s_target_cm - s_estimator.position_cm;
    profile = Q45Vehicle_SelectProfile();
    if (profile == BALL_OUTER_PROFILE_RETURN_CRUISE)
    {
        float lead_time_s;

        if (s_status.mode == Q45_VEHICLE_MODE_Q5)
        {
            lead_time_s =
                (control_error_cm < 0.0f) ? Q45_Q5_LEAD_TIME_ERROR_NEG_S : Q45_Q5_LEAD_TIME_S;
        }
        else
        {
            lead_time_s = (control_error_cm < 0.0f) ? Q4_RECOVERY_LEAD_TIME_S_ERROR_NEG
                                                    : Q4_RECOVERY_LEAD_TIME_S;
        }
        control_error_cm -= s_estimator.velocity_cm_s * lead_time_s;
    }

    q5_moving_bend =
        (s_status.mode == Q45_VEHICLE_MODE_Q5) && (s_status.state == Q45_VEHICLE_RUNNING) &&
        ((Q45Vehicle_Abs(s_status.lateral_accel_m_s2) >= Q45_Q5_TURN_STICTION_DISABLE_LAT_M_S2) ||
         ((s_status.route_distance_cm >= Q45_Q5_SECOND_FB_START_CM) &&
          (s_status.route_distance_cm <= Q45_Q5_SECOND_TURN_START_CM)));
    if (q5_moving_bend)
    {
        s_stiction_active = false;
        s_still_since_ms = 0U;
    }
    if (!q5_moving_bend && !s_stiction_active &&
        (distance_cm > ((s_status.mode == Q45_VEHICLE_MODE_Q6)
                            ? Q6_STICTION_ENGAGE_CM
                            : Q45_CENTER_POSITION_TOLERANCE_CM)) &&
        (speed_abs <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
    {
        if (s_still_since_ms == 0U)
        {
            s_still_since_ms = now_ms;
        }
        else if (((now_ms - s_still_since_ms) >= Q45_STICTION_CONFIRM_MS) &&
                 (s_status.stiction_count < ((s_status.mode == Q45_VEHICLE_MODE_Q6)
                                                 ? Q6_STICTION_MAX_COUNT
                                                 : Q45_STICTION_MAX_COUNT)))
        {
            ++s_status.stiction_count;
            s_stiction_active = true;
            s_stiction_started_ms = now_ms;
            s_still_since_ms = 0U;
        }
    }
    else if ((distance_cm <= Q45_CENTER_POSITION_TOLERANCE_CM) ||
             (speed_abs > BALL_OUTER_STICTION_RELEASE_SPEED_CM_S))
    {
        s_still_since_ms = 0U;
    }

    if (s_stiction_active && ((now_ms - s_stiction_started_ms) >=
                              ((s_status.mode == Q45_VEHICLE_MODE_Q6) ? Q6_STICTION_PUSH_HOLD_MS
                                                                      : Q45_STICTION_HOLD_MS)))
    {
        s_stiction_active = false;
        s_still_since_ms = now_ms;
    }

    if (!Q45Vehicle_ComputeFastMotor(
            control_error_cm, s_estimator.velocity_cm_s, profile, &output_deg))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
        return false;
    }
    Q45Vehicle_ApplyNearCenterLimit(s_estimator.position_cm - s_target_cm, profile, &output_deg);

    /* Keep the bounded breakaway pulse available in Q6 HOLD so a stopped
     * ball just outside the engagement band can overcome static friction.
     * Q4/Q5 retain their existing HOLD-gated breakaway behavior. */
    if (s_stiction_active &&
        ((profile != BALL_OUTER_PROFILE_RETURN_HOLD) || (s_status.mode == Q45_VEHICLE_MODE_Q6)))
    {
        float push_deg = (s_status.mode == Q45_VEHICLE_MODE_Q6) ? Q6_STICTION_PUSH_MOTOR_DEG
                                                                : Q4_RECOVERY_DIRECT_BREAKAWAY_DEG;
        if (output_deg < 0.0f)
        {
            output_deg = -push_deg;
        }
        else if (output_deg > 0.0f)
        {
            output_deg = push_deg;
        }
        else
        {
            output_deg = (control_error_cm >= 0.0f) ? -push_deg : push_deg;
        }
    }

    static_beam_deg = PendulumGeometry_MotorDegToBeamDeg(output_deg);
    /* The launch ramp deliberately removes model feedforward here. During
     * the moving-vehicle launch, boost the existing request only until the
     * first bend. The static bench image does not move the chassis, so the
     * route-distance launch gain would otherwise affect every Q6 command;
     * the moving-vehicle image retains its existing launch behavior. */
#if (defined(Q6_STATIC_TEST_MODE) && (Q6_STATIC_TEST_MODE != 0))
#else
    if (Q45Vehicle_IsDynamicMode() && (s_status.route_distance_cm < Q45_Q5_FIRST_TURN_START_CM))
    {
        static_beam_deg *= Q45_Q5_LAUNCH_STATIC_BEAM_GAIN;
    }
#endif
    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        static_beam_deg *= Q45Vehicle_Q5LateStraightStaticGain();
    }
    integral_beam_deg = Q45Vehicle_UpdateQ5BiasIntegral(
        s_estimator.position_cm - s_target_cm, s_estimator.velocity_cm_s, dt_s);
    feedforward_beam_deg = s_stiction_active ? 0.0f : Q45Vehicle_Q5FeedforwardBeamDeg();
    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        second_turn_feedback_beam_deg = Q45Vehicle_UpdateQ5SecondTurnFeedback(
            s_estimator.position_cm, s_estimator.velocity_cm_s, sample.confidence_pct, dt_s);
        first_turn_feedback_beam_deg = Q45Vehicle_Q5FirstTurnPositionTrim(
            s_estimator.position_cm, s_estimator.velocity_cm_s, sample.confidence_pct);
    }
    else
    {
        second_turn_feedback_beam_deg = 0.0f;
        first_turn_feedback_beam_deg = 0.0f;
    }
    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        if (!PendulumGeometry_BeamDegToMotorDeg(
                static_beam_deg + integral_beam_deg + feedforward_beam_deg +
                    second_turn_feedback_beam_deg + first_turn_feedback_beam_deg,
                &output_deg))
        {
            Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
            return false;
        }
    }
    else if (!PendulumGeometry_BeamDegToMotorDeg(
                 static_beam_deg + integral_beam_deg + feedforward_beam_deg, &output_deg))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
        return false;
    }
    s_status.control_output_limited = false;
    if (Q45Vehicle_Abs(output_deg) > Q4_RECOVERY_MAX_ANGLE_DEG)
    {
        output_deg = (output_deg < 0.0f) ? -Q4_RECOVERY_MAX_ANGLE_DEG : Q4_RECOVERY_MAX_ANGLE_DEG;
        s_status.control_output_limited = true;
    }
    s_status.control_profile = (uint8_t)profile;
    /* Q5D separates the frozen static-PD request, Q5-only slow trim, and
     * acceleration feedforward so each effect is measured independently. */
    s_status.p_beam_deg = static_beam_deg;
    s_status.d_beam_deg = 0.0f;
    s_status.i_beam_deg = integral_beam_deg;
    s_status.feedforward_beam_deg = feedforward_beam_deg;
    if (s_status.mode == Q45_VEHICLE_MODE_Q5)
    {
        s_status.requested_beam_deg = static_beam_deg + integral_beam_deg + feedforward_beam_deg +
                                      second_turn_feedback_beam_deg + first_turn_feedback_beam_deg;
    }
    else
    {
        s_status.requested_beam_deg = static_beam_deg + integral_beam_deg + feedforward_beam_deg;
    }
    s_status.predicted_position_cm = s_estimator.position_cm;
    command_period_ms = Q4_RECOVERY_COMMAND_PERIOD_MS;

    if ((Q45Vehicle_Abs(output_deg - s_last_command_deg) >= BALL_CONTROL_COMMAND_DELTA_DEG) &&
        ((!s_have_sample) || ((now_ms - s_last_command_ms) >= command_period_ms)))
    {
        if (BallActuator_SetAbsoluteAngleDeg(output_deg))
        {
            s_last_command_deg = output_deg;
            s_last_command_ms = now_ms;
        }
    }

    if (s_have_sample && ((sample.rx_time_ms - s_last_vision_rx_ms) > s_status.max_frame_gap_ms))
        s_status.max_frame_gap_ms = sample.rx_time_ms - s_last_vision_rx_ms;
    s_last_vision_rx_ms = sample.rx_time_ms;
    s_last_processed_vision_rx_ms = sample.rx_time_ms;
    s_last_valid_vision_ms = sample.rx_time_ms;
    ++s_status.accepted_frames;
    s_have_sample = true;
    s_status.raw_position_cm = sample.position_cm;
    s_status.measured_cm = s_estimator.position_cm;
    s_status.measured_velocity_cm_s = s_estimator.velocity_cm_s;
    s_status.error_cm = s_target_cm - s_estimator.position_cm;
    s_status.control_error_cm = control_error_cm;
    s_status.control_output_deg = output_deg;
    s_status.commanded_angle_deg = s_last_command_deg;
    s_status.actual_angle_deg = actuator.actual_angle_deg;
    s_status.in_scoring_band =
        Q45Vehicle_Abs(sample.position_cm - s_target_cm) <= Q45_CENTER_POSITION_TOLERANCE_CM;
    s_status.center_ready = s_status.in_scoring_band;
    if (Q45Vehicle_Abs(sample.position_cm - s_run_start_position_cm) > s_status.max_abs_position_cm)
    {
        s_status.max_abs_position_cm = Q45Vehicle_Abs(sample.position_cm - s_run_start_position_cm);
    }
    if (!s_status.in_scoring_band)
    {
        s_status.band_violation = true;
    }
    s_status.vision_confidence_pct = sample.confidence_pct;
    s_status.estimator_rejected_count = s_estimator.rejected_count;
    s_status.vision_sequence = sample.sequence;
    s_status.vision_age_ms = now_ms - sample.rx_time_ms;
    return true;
}

void Q45Vehicle_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = Q45_VEHICLE_IDLE;
    s_status.acceleration_feedforward_enabled = false;
    BallStateEstimator_Init(&s_estimator);
    Q45Vehicle_ResetDynamicModel();
    s_result_log_pending = false;
    s_target_cm = 0.0f;
    s_q6_source = Q6_TARGET_CAPTURE;
    s_q6_explicit_cm = 0.0f;
    Q6Target_Init(s_q6_source, s_q6_explicit_cm);
}

bool Q45Vehicle_Q6ConfigureTarget(Q6TargetSource source, float explicit_cm)
{
    if (Q45Vehicle_IsActive() || ((source != Q6_TARGET_EXPLICIT) && (source != Q6_TARGET_CAPTURE)))
    {
        return false;
    }
    if (source == Q6_TARGET_EXPLICIT)
    {
        if (!isfinite(explicit_cm) || (Q45Vehicle_Abs(explicit_cm) > Q6_TEST_TARGET_LIMIT_CM))
        {
            return false;
        }
    }
    else
    {
        explicit_cm = 0.0f;
    }
    s_q6_source = source;
    s_q6_explicit_cm = explicit_cm;
    Q6Target_Init(s_q6_source, s_q6_explicit_cm);
    return true;
}

bool Q45Vehicle_SelectMode(Q45VehicleMode mode, uint32_t now_ms)
{
    if (((mode != Q45_VEHICLE_MODE_Q4) && (mode != Q45_VEHICLE_MODE_Q5) &&
         (mode != Q45_VEHICLE_MODE_Q6)) ||
        Q45Vehicle_IsActive())
    {
        return false;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.mode = mode;
    s_status.run_sequence = ++s_run_sequence;
    s_status.acceleration_feedforward_enabled =
        ((mode == Q45_VEHICLE_MODE_Q5) || (mode == Q45_VEHICLE_MODE_Q6)) &&
        (Q45_ACCELERATION_FEEDFORWARD_ENABLED != 0U);
    s_target_cm = 0.0f;
    if (mode == Q45_VEHICLE_MODE_Q6)
    {
        Q6Target_Init(s_q6_source, s_q6_explicit_cm);
    }
    BallStateEstimator_Init(&s_estimator);
    Q45Vehicle_ResetDynamicModel();
    s_stage_started_ms = now_ms;
    s_last_vision_rx_ms = 0U;
    s_last_processed_vision_rx_ms = 0U;
    s_last_valid_vision_ms = 0U;
    s_last_command_ms = 0U;
    s_still_since_ms = 0U;
    s_stiction_started_ms = 0U;
    s_last_command_deg = 0.0f;
    s_have_sample = false;
    s_stiction_active = false;
    s_zero_command_sent = false;
    s_origin_zero_sent = false;
    s_terminal_zero_retry_ms = 0U;
    s_terminal_actuator_paused = false;
    s_result_log_pending = false;
    s_run_start_position_cm = 0.0f;
    Q45Vehicle_Enter(Q45_VEHICLE_WAIT_ACTUATOR, now_ms);
    return true;
}

bool Q45Vehicle_StartSelected(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;

    if ((s_status.state != Q45_VEHICLE_READY) || !Q45Vehicle_UpdateCenterReady(now_ms))
    {
        return false;
    }

    BallActuator_GetStatus(&actuator);
    if (!actuator.actual_angle_valid ||
        (Q45Vehicle_Abs(actuator.actual_angle_deg) > Q4_ZERO_RETURN_TOLERANCE_DEG) ||
        !VisionLink_GetLatest(&sample))
    {
        return false;
    }

    /* Q6 locks the captured/explicit hold target exactly once per run,
     * right before formal timing starts; Q4/Q5 keep the frozen midpoint
     * target 0. */
    if (s_status.mode == Q45_VEHICLE_MODE_Q6)
    {
        if (!Q6Target_Lock(&s_target_cm))
        {
            return false;
        }
        s_status.target_cm = s_target_cm;
        s_status.target_locked = true;
    }
    else
    {
        s_target_cm = 0.0f;
    }

    /* Formal timing and vehicle motion begin only after all readiness checks
     * pass; selection, zeroing, and target-capture time are excluded. */
    Licheng_Reset();
    Control_TraceStart(0.0f);
    s_status.elapsed_ms = 0U;
    s_status.drive_elapsed_ms = 0U;
    s_status.pass_elapsed_ms = 0U;
    s_status.route_distance_cm = 0.0f;
    s_status.max_abs_position_cm = 0.0f;
    s_status.max_target_dev_cm = 0.0f;
    s_run_start_position_cm = sample.position_cm;
    s_status.route_passed = false;
    s_status.stop_requested = false;
    s_status.stop_reason = Q45_STOP_NONE;
    s_status.fault = false;
    s_last_valid_vision_ms = now_ms;
    s_last_vision_rx_ms = sample.rx_time_ms;
    s_last_processed_vision_rx_ms = sample.rx_time_ms;
    s_last_command_ms = 0U;
    s_last_command_deg = 0.0f;
    s_have_sample = false;
    s_stiction_active = false;
    s_still_since_ms = 0U;
    s_status.center_ready = true;
    BallStateEstimator_Init(&s_estimator);
    Q45Vehicle_ResetDynamicModel();
    Q45Vehicle_Enter(Q45_VEHICLE_RUNNING, now_ms);
    return true;
}

void Q45Vehicle_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;

    if (s_status.state == Q45_VEHICLE_IDLE)
    {
        return;
    }

    if (Q45Vehicle_IsTerminal(s_status.state))
    {
        BallActuator_GetStatus(&actuator);
        if (!s_terminal_actuator_paused)
        {
            if (!s_zero_command_sent ||
                ((now_ms - s_terminal_zero_retry_ms) >= BALL_CONTROL_ZERO_RETRY_PERIOD_MS))
            {
                if (BallActuator_SetAbsoluteAngleDeg(0.0f))
                {
                    s_zero_command_sent = true;
                    s_terminal_zero_retry_ms = now_ms;
                    s_status.commanded_angle_deg = 0.0f;
                }
            }

            if ((actuator.actual_angle_valid &&
                 (Q45Vehicle_Abs(actuator.actual_angle_deg) <= Q4_ZERO_RETURN_TOLERANCE_DEG)) ||
                ((now_ms - s_status.state_started_ms) >= Q45_ZERO_TIMEOUT_MS))
            {
                s_status.terminal_zero_confirmed =
                    actuator.actual_angle_valid &&
                    (Q45Vehicle_Abs(actuator.actual_angle_deg) <= Q4_ZERO_RETURN_TOLERANCE_DEG);
                (void)BallActuator_Stop();
                (void)BallActuator_Enable(false);
                BallActuator_SetCommandSession(false);
                s_terminal_actuator_paused = true;
                s_status.actuator_paused = true;
                if (!s_status.terminal_zero_confirmed && (s_status.state == Q45_VEHICLE_FINISHED))
                {
                    s_status.fault = true;
                    s_status.stop_reason = Q45_STOP_ACTUATOR;
                    Q45Vehicle_Enter(Q45_VEHICLE_FAULT, now_ms);
                    s_result_log_pending = true;
                }
            }
        }
        return;
    }

    if ((s_status.state == Q45_VEHICLE_RUNNING) &&
        (s_status.elapsed_ms >= Q45Vehicle_TimeLimitMs()) &&
        ((s_status.mode != Q45_VEHICLE_MODE_Q4) || !s_status.route_passed))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_TIMEOUT, Q45_STOP_TIME_LIMIT, now_ms);
        return;
    }

    BallActuator_GetStatus(&actuator);
    switch (s_status.state)
    {
    case Q45_VEHICLE_WAIT_ACTUATOR:
        if (actuator.online && actuator.options_valid && actuator.firmware_x &&
            actuator.foc_closed_loop)
        {
            BallActuator_SetCommandSession(true);
            if (!BallActuator_ClearProtection() || !BallActuator_Enable(true))
            {
                Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
                return;
            }
            s_stage_started_ms = now_ms;
            Q45Vehicle_Enter(Q45_VEHICLE_CONFIGURE, now_ms);
        }
        else if ((now_ms - s_status.state_started_ms) >= Q45_ACTUATOR_READY_TIMEOUT_MS)
        {
            Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
        }
        break;

    case Q45_VEHICLE_CONFIGURE:
        if ((now_ms - s_stage_started_ms) >= Q45_ENABLE_TO_PROFILE_GAP_MS)
        {
            uint16_t accel_rpm_s =
                Q45Vehicle_IsDynamicMode() ? Q45_Q5_ACTUATOR_ACCEL_RPM_S : Q4_ACCEL_RPM_S;
            uint16_t decel_rpm_s =
                Q45Vehicle_IsDynamicMode() ? Q45_Q5_ACTUATOR_DECEL_RPM_S : Q4_DECEL_RPM_S;
            float max_speed_rpm =
                Q45Vehicle_IsDynamicMode() ? Q45_Q5_ACTUATOR_MAX_SPEED_RPM : Q4_MAX_SPEED_RPM;
            uint16_t max_current_ma =
                Q45Vehicle_IsDynamicMode() ? Q45_Q5_ACTUATOR_MAX_CURRENT_MA : Q4_MAX_CURRENT_MA;
            if (!BallActuator_ClearProtection() || !BallActuator_Enable(true) ||
                !BallActuator_PrepareAbsolutePosition(
                    accel_rpm_s, decel_rpm_s, max_speed_rpm, max_current_ma))
            {
                Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
                return;
            }
            s_stage_started_ms = now_ms;
            Q45Vehicle_Enter(Q45_VEHICLE_ZERO, now_ms);
        }
        break;

    case Q45_VEHICLE_ZERO:
        if (!s_origin_zero_sent && ((now_ms - s_stage_started_ms) >= Q45_PROFILE_TO_ZERO_GAP_MS))
        {
            if (s_status.mode == Q45_VEHICLE_MODE_Q6)
            {
                /* Q6 physically returns to the motor's stored absolute
                     * zero.  Redefining zero at the current pose let any pose
                     * left behind by a previous run (e.g. an EE-rejected 0-deg
                     * command parking the arm at ~14 deg) become the "level"
                     * of the next trial.  9A/04 is the return-home path
                     * proven by the Q4 baseline 8b66fd4 sequence. */
                if (!BallActuator_ReturnAbsoluteZero())
                {
                    Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
                    return;
                }
            }
            else
            {
                /* Q4/Q5 keep the original semantics: the current
                     * physical pose is the task origin selected by first
                     * press. */
                if (!BallActuator_ZeroAtCurrentPosition())
                {
                    Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
                    return;
                }
            }
            s_stage_started_ms = now_ms;
            s_origin_zero_sent = true;
        }
        else if (s_origin_zero_sent)
        {
            if ((now_ms - s_stage_started_ms) >= ((s_status.mode == Q45_VEHICLE_MODE_Q6)
                                                      ? Q45_Q6_ZERO_RETURN_TIMEOUT_MS
                                                      : Q45_ZERO_TIMEOUT_MS))
            {
                Q45Vehicle_RequestStop(Q45_VEHICLE_FAULT, Q45_STOP_ACTUATOR, now_ms);
            }
            else if (actuator.actual_angle_valid &&
                     (Q45Vehicle_Abs(actuator.actual_angle_deg) <= Q4_ZERO_RETURN_TOLERANCE_DEG))
            {
                (void)Q45Vehicle_UpdateCenterReady(now_ms);
                Q45Vehicle_Enter(Q45_VEHICLE_READY, now_ms);
            }
        }
        break;

    case Q45_VEHICLE_READY:
        /* Only Q45Vehicle_StartSelected(), called from the matching key,
             * may leave this state. */
        (void)Q45Vehicle_UpdateCenterReady(now_ms);
        break;

    case Q45_VEHICLE_RUNNING:
        s_status.vision_age_ms = now_ms - s_last_processed_vision_rx_ms;
        s_status.accepted_age_ms = now_ms - s_last_valid_vision_ms;
        s_status.actual_angle_age_ms = now_ms - actuator.last_position_ms;
        Q45Vehicle_UpdateChassisModel(now_ms);
        (void)Q45Vehicle_UpdateBallControl(now_ms);
        break;

    default:
        break;
    }
}

void Q45Vehicle_Tick10msISR(void)
{
    float pass_distance;

    if (s_status.state != Q45_VEHICLE_RUNNING)
    {
        return;
    }
    s_status.elapsed_ms += 10U;
    s_status.drive_elapsed_ms += 10U;
    s_status.route_distance_cm = g_licheng.center_cm;
    /* Model and visual feedback execute in the foreground, not this IRQ. */
    pass_distance = Q45Vehicle_PassDistanceCm();
    if (!s_status.route_passed && (s_status.route_distance_cm >= pass_distance))
    {
        s_status.route_passed = true;
        s_status.pass_elapsed_ms = s_status.elapsed_ms;
    }

    if ((s_status.mode == Q45_VEHICLE_MODE_Q4) && !s_status.route_passed &&
        (s_status.elapsed_ms >= Q45_Q4_TIME_LIMIT_MS))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_TIMEOUT,
                               Q45_STOP_TIME_LIMIT,
                               s_status.state_started_ms + s_status.drive_elapsed_ms);
        return;
    }
    if (Q45Vehicle_IsDynamicMode() && (s_status.elapsed_ms >= Q45_Q5_TIME_LIMIT_MS))
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_TIMEOUT,
                               Q45_STOP_TIME_LIMIT,
                               s_status.state_started_ms + s_status.drive_elapsed_ms);
        return;
    }
    if (s_status.route_distance_cm >= Q45Vehicle_StopDistanceCm())
    {
        Q45Vehicle_RequestStop(Q45_VEHICLE_FINISHED,
                               Q45_STOP_ROUTE_COMPLETE,
                               s_status.state_started_ms + s_status.drive_elapsed_ms);
        return;
    }

    s_status.base_speed_mm_s = Q45Vehicle_TraceSpeedMmS();
#if Q6_STATIC_TEST_MODE
    if (s_status.mode == Q45_VEHICLE_MODE_Q6)
    {
        /* Bench image only: run the hold-position trial with the chassis
         * parked, isolating the ball loop from vehicle dynamics. */
        s_status.base_speed_mm_s = 0.0f;
    }
#endif
    Control_SetBaseSpeed(s_status.base_speed_mm_s);
}

void Q45Vehicle_Stop(uint32_t now_ms)
{
    if (!Q45Vehicle_IsActive())
    {
        return;
    }
    (void)BallActuator_SetAbsoluteAngleDeg(0.0f);
    s_status.commanded_angle_deg = 0.0f;
    s_zero_command_sent = true;
    Q45Vehicle_RequestStop(Q45_VEHICLE_STOPPED, Q45_STOP_OPERATOR, now_ms);
    BallActuator_SetCommandSession(false);
}

bool Q45Vehicle_IsActive(void)
{
    return (s_status.state == Q45_VEHICLE_WAIT_ACTUATOR) ||
           (s_status.state == Q45_VEHICLE_CONFIGURE) || (s_status.state == Q45_VEHICLE_ZERO) ||
           (s_status.state == Q45_VEHICLE_READY) || (s_status.state == Q45_VEHICLE_RUNNING);
}

bool Q45Vehicle_StopRequested(void)
{
    return s_status.stop_requested;
}

bool Q45Vehicle_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void Q45Vehicle_GetStatus(Q45VehicleStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

const char *Q45Vehicle_StateName(Q45VehicleState state)
{
    switch (state)
    {
    case Q45_VEHICLE_IDLE:
        return "IDLE";
    case Q45_VEHICLE_WAIT_ACTUATOR:
        return "ACT";
    case Q45_VEHICLE_CONFIGURE:
        return "CFG";
    case Q45_VEHICLE_ZERO:
        return "ZERO";
    case Q45_VEHICLE_READY:
        return "READY";
    case Q45_VEHICLE_RUNNING:
        return "RUN";
    case Q45_VEHICLE_FINISHED:
        return "DONE";
    case Q45_VEHICLE_TIMEOUT:
        return "TIME";
    case Q45_VEHICLE_VISION_LOST:
        return "VISION";
    case Q45_VEHICLE_FAULT:
        return "FAULT";
    case Q45_VEHICLE_STOPPED:
        return "STOP";
    default:
        return "?";
    }
}

const char *Q45Vehicle_ModeName(Q45VehicleMode mode)
{
    switch (mode)
    {
    case Q45_VEHICLE_MODE_Q4:
        return "Q4";
    case Q45_VEHICLE_MODE_Q5:
        return "Q5";
    case Q45_VEHICLE_MODE_Q6:
        return "Q6";
    default:
        return "--";
    }
}
