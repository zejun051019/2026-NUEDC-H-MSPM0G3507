#include "ball_outer_pd.h"

#include <math.h>
#include <string.h>

#include "ball_control_config.h"
#include "pendulum_geometry.h"
#include "q3_control_config.h"

static float BallOuterPd_Clip(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}

static float BallOuterPd_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void BallOuterPd_Init(BallOuterPd *controller)
{
    if (controller != NULL)
    {
        memset(controller, 0, sizeof(*controller));
        controller->profile = BALL_OUTER_PROFILE_NORMAL;
    }
}

void BallOuterPd_SetProfile(BallOuterPd *controller, BallOuterPdProfile profile)
{
    if (controller != NULL)
    {
        controller->profile = profile;
    }
}

void BallOuterPd_ResetTrajectory(BallOuterPd *controller)
{
    if (controller != NULL)
    {
        controller->initialized = false;
        controller->beam_rate_deg_s = 0.0f;
        controller->stiction_assist_active = false;
    }
}

bool BallOuterPd_Update(BallOuterPd *controller,
                        float error_cm,
                        float velocity_cm_s,
                        float dt_s,
                        float actual_motor_deg,
                        BallOuterPdOutput *output)
{
    float control_direction;
    float requested_rate;
    float rate_step_limit;
    float angle_step;
    float remaining;
    float effective_kd;
    float beam_limit;

    if ((controller == NULL) || (output == NULL) || !isfinite(error_cm) ||
        !isfinite(velocity_cm_s) || !isfinite(dt_s) || (dt_s <= 0.0f) ||
        (dt_s > BALL_CONTROL_SAMPLE_MAX_S) || !isfinite(actual_motor_deg))
    {
        return false;
    }

    memset(output, 0, sizeof(*output));
    if (!controller->initialized)
    {
        controller->beam_angle_deg = PendulumGeometry_MotorDegToBeamDeg(actual_motor_deg);
        controller->beam_rate_deg_s = 0.0f;
        controller->initialized = true;
    }

    if (fabsf(error_cm) <= BALL_OUTER_POSITION_DEADBAND_CM)
    {
        error_cm = 0.0f;
    }

    output->p_deg = Q3_POSITION_TO_MOTOR_SIGN * BALL_OUTER_KP_DEG_PER_CM * error_cm;
    /* The visual velocity includes latency/noise.  Far from the requested
     * position, retain P dominance for continuous travel; near the target,
     * return to full D for predictive braking. */
    /* Return tuning is deliberately isolated from 0 -> +5 tuning.  The
     * stronger brake D only applies after Q3 has entered RET-BRAKE; normal
     * travel and the launch phase retain their proven gains. */
    switch (controller->profile)
    {
    case BALL_OUTER_PROFILE_RETURN_BRAKE:
        effective_kd = BALL_OUTER_RETURN_BRAKE_KD_DEG_PER_CM_S;
        break;
    case BALL_OUTER_PROFILE_RETURN_HOLD:
        effective_kd = BALL_OUTER_RETURN_HOLD_KD_DEG_PER_CM_S;
        break;
    case BALL_OUTER_PROFILE_RETURN_CRUISE:
        effective_kd = BALL_OUTER_RETURN_CRUISE_KD_DEG_PER_CM_S;
        break;
    default:
        effective_kd = (BallOuterPd_Abs(error_cm) <= BALL_OUTER_BRAKE_ZONE_CM)
                           ? BALL_OUTER_KD_DEG_PER_CM_S
                           : BALL_OUTER_KD_CRUISE_DEG_PER_CM_S;
        break;
    }
    output->d_deg = Q3_POSITION_TO_MOTOR_SIGN * (-effective_kd * velocity_cm_s);
    output->raw_beam_deg = output->p_deg + output->d_deg + BALL_OUTER_FEEDFORWARD_DEG;
    switch (controller->profile)
    {
    case BALL_OUTER_PROFILE_RETURN_CRUISE:
        beam_limit = BALL_OUTER_RETURN_CRUISE_BEAM_LIMIT_DEG;
        break;
    case BALL_OUTER_PROFILE_RETURN_BRAKE:
        beam_limit = BALL_OUTER_RETURN_BRAKE_BEAM_LIMIT_DEG;
        break;
    case BALL_OUTER_PROFILE_RETURN_HOLD:
        beam_limit = BALL_OUTER_RETURN_HOLD_BEAM_LIMIT_DEG;
        break;
    default:
        beam_limit = BALL_OUTER_BEAM_LIMIT_DEG;
        break;
    }
    output->limited_beam_deg = BallOuterPd_Clip(output->raw_beam_deg, beam_limit);
    output->limited = fabsf(output->raw_beam_deg - output->limited_beam_deg) > 0.0001f;

    requested_rate =
        BallOuterPd_Clip((output->limited_beam_deg - controller->beam_angle_deg) / dt_s,
                         BALL_OUTER_BEAM_RATE_LIMIT_DEG_S);
    rate_step_limit = BALL_OUTER_BEAM_ACCEL_LIMIT_DEG_S2 * dt_s;
    controller->beam_rate_deg_s +=
        BallOuterPd_Clip(requested_rate - controller->beam_rate_deg_s, rate_step_limit);

    angle_step = controller->beam_rate_deg_s * dt_s;
    remaining = output->limited_beam_deg - controller->beam_angle_deg;
    if ((angle_step * remaining > 0.0f) && (fabsf(angle_step) >= fabsf(remaining)))
    {
        controller->beam_angle_deg = output->limited_beam_deg;
        controller->beam_rate_deg_s = 0.0f;
    }
    else
    {
        controller->beam_angle_deg += angle_step;
    }

    output->shaped_beam_deg = controller->beam_angle_deg;
    output->beam_rate_deg_s = controller->beam_rate_deg_s;
    if (!PendulumGeometry_BeamDegToMotorDeg(output->shaped_beam_deg, &output->motor_deg))
    {
        return false;
    }
    output->end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output->motor_deg);

    /* The PD output can correctly request only 3--4 motor degrees when the
     * ball is still several centimetres from target.  This mechanism needs
     * +/-7 degrees to leave its measured static-friction pocket, so preserve
     * the PD trajectory but temporarily lift the motor target to that proven
     * threshold.  It is released by detected motion, not by elapsed time. */
    if ((controller->profile == BALL_OUTER_PROFILE_RETURN_BRAKE) ||
        (controller->profile == BALL_OUTER_PROFILE_RETURN_HOLD))
    {
        controller->stiction_assist_active = false;
    }
    else if (!controller->stiction_assist_active &&
             (BallOuterPd_Abs(error_cm) > BALL_OUTER_STICTION_ERROR_CM) &&
             (BallOuterPd_Abs(velocity_cm_s) <= BALL_OUTER_STICTION_ENTER_SPEED_CM_S))
    {
        controller->stiction_assist_active = true;
    }
    else if (controller->stiction_assist_active &&
             ((BallOuterPd_Abs(error_cm) <= BALL_CONTROL_TARGET_TOLERANCE_CM) ||
              (BallOuterPd_Abs(velocity_cm_s) >= BALL_OUTER_STICTION_RELEASE_SPEED_CM_S)))
    {
        controller->stiction_assist_active = false;
    }

    if (controller->stiction_assist_active)
    {
        output->stiction_motor_deg = (output->motor_deg < 0.0f)
                                         ? BALL_OUTER_STICTION_TO_VISUAL_POS_MOTOR_DEG
                                         : BALL_OUTER_STICTION_TO_VISUAL_NEG_MOTOR_DEG;
        if (BallOuterPd_Abs(output->motor_deg) < BallOuterPd_Abs(output->stiction_motor_deg))
        {
            output->motor_deg = output->stiction_motor_deg;
            output->end_height_mm = PendulumGeometry_MotorDegToEndHeightMm(output->motor_deg);
        }
    }
    output->stiction_assist = controller->stiction_assist_active;

    /* D action is predictive braking.  Kd/Kp is its approximate look-ahead
     * time. This field is diagnostic only; it does not claim a friction model. */
    output->predicted_stop_cm = fabsf(velocity_cm_s) * (effective_kd / BALL_OUTER_KP_DEG_PER_CM);
    control_direction = Q3_POSITION_TO_MOTOR_SIGN * output->raw_beam_deg;
    output->braking = (error_cm * velocity_cm_s > 0.0f) && (control_direction * error_cm < 0.0f);
    return true;
}
