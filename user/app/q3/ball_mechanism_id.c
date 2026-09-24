#include "ball_mechanism_id.h"

#include <string.h>

#include "app/ball_actuator/ball_actuator.h"
#include "app/q3/ball_mechanism_id_config.h"
#include "code/vision_link.h"

static BallMechanismIdStatus s_status;
static bool s_result_log_pending;

static float BallMechanismId_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void BallMechanismId_Enter(BallMechanismIdState state, uint32_t now_ms)
{
    s_status.state = state;
    s_status.state_started_ms = now_ms;
    if ((state == BALL_ID_WAIT_POINT) || (state == BALL_ID_WAIT_NEGATIVE) ||
        (state == BALL_ID_COMPLETE) || (state == BALL_ID_FAULT) || (state == BALL_ID_ABORTED))
    {
        s_result_log_pending = true;
    }
}

static bool BallMechanismId_Expired(uint32_t now_ms, uint32_t interval_ms)
{
    return (now_ms - s_status.state_started_ms) >= interval_ms;
}

static bool BallMechanismId_ReadVision(uint32_t now_ms, VisionLinkSample *sample)
{
    if ((!VisionLink_GetLatest(sample)) || (!sample->valid))
    {
        s_status.vision_fresh = false;
        s_status.vision_age_ms = 0xFFFFFFFFUL;
        return false;
    }

    s_status.current_cm = sample->position_cm;
    s_status.vision_age_ms = now_ms - sample->rx_time_ms;
    s_status.vision_fresh = s_status.vision_age_ms <= BALL_ID_VISION_TIMEOUT_MS;
    return s_status.vision_fresh;
}

static void BallMechanismId_Fail(BallMechanismIdFault fault, uint32_t now_ms)
{
    s_status.fault = fault;
    s_status.target_angle_deg = 0.0f;
    (void)BallActuator_Stop();
    (void)BallActuator_Enable(false);
    BallActuator_SetCommandSession(false);
    BallMechanismId_Enter(BALL_ID_FAULT, now_ms);
}

void BallMechanismId_Init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALL_ID_IDLE;
    s_status.direction = 1;
    s_result_log_pending = false;
}

bool BallMechanismId_Start(uint32_t now_ms)
{
    if (BallMechanismId_IsActive())
    {
        return false;
    }

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = BALL_ID_IDLE;
    s_status.direction = 1;
    s_status.next_angle_deg = BALL_ID_FIRST_ANGLE_DEG;
    s_status.expected_start_cm = BALL_ID_START_POSITION_CM;
    s_status.fault = BALL_ID_FAULT_NONE;
    s_result_log_pending = false;
    BallActuator_SetCommandSession(true);
    BallMechanismId_Enter(BALL_ID_PRECHECK, now_ms);
    return true;
}

bool BallMechanismId_TriggerPoint(uint32_t now_ms)
{
    VisionLinkSample sample;

    if (s_status.state == BALL_ID_WAIT_NEGATIVE)
    {
        /* This press changes only the requested starting side.  The next
         * press is still required to apply the first negative-angle point. */
        s_status.direction = -1;
        s_status.next_angle_deg = BALL_ID_FIRST_ANGLE_DEG;
        s_status.expected_start_cm = -BALL_ID_START_POSITION_CM;
        BallMechanismId_Enter(BALL_ID_WAIT_POINT, now_ms);
        return true;
    }

    if (s_status.state != BALL_ID_WAIT_POINT)
    {
        return false;
    }

    /* Safety-sweep mode accepts any stable visible ball position.  The later
     * distance-response mapping mode will re-enable the strict +/-5 cm gate. */
    if ((!BallMechanismId_ReadVision(now_ms, &sample)) || (!sample.stable))
    {
        ++s_status.trigger_rejects;
        return false;
    }

    s_status.start_cm = sample.position_cm;
    s_status.end_cm = sample.position_cm;
    s_status.delta_cm = 0.0f;
    s_status.moved = false;
    s_status.target_angle_deg = (float)s_status.direction * s_status.next_angle_deg;
    if (!BallActuator_SetAbsoluteAngleDeg(s_status.target_angle_deg))
    {
        BallMechanismId_Fail(BALL_ID_FAULT_POSITION_COMMAND, now_ms);
        return false;
    }

    BallMechanismId_Enter(BALL_ID_HOLD_POINT, now_ms);
    return true;
}

void BallMechanismId_Abort(uint32_t now_ms)
{
    if (BallMechanismId_IsActive())
    {
        (void)BallActuator_Stop();
        (void)BallActuator_Enable(false);
        BallActuator_SetCommandSession(false);
        s_status.target_angle_deg = 0.0f;
        BallMechanismId_Enter(BALL_ID_ABORTED, now_ms);
    }
}

bool BallMechanismId_IsActive(void)
{
    return (s_status.state >= BALL_ID_PRECHECK) && (s_status.state <= BALL_ID_WAIT_NEGATIVE);
}

bool BallMechanismId_IsWaitingForTrigger(void)
{
    return (s_status.state == BALL_ID_WAIT_POINT) || (s_status.state == BALL_ID_WAIT_NEGATIVE);
}

bool BallMechanismId_ConsumeResultLog(void)
{
    bool pending = s_result_log_pending;
    s_result_log_pending = false;
    return pending;
}

void BallMechanismId_Task(uint32_t now_ms)
{
    BallActuatorStatus actuator;
    VisionLinkSample sample;
    bool fresh_vision;

    if (!BallMechanismId_IsActive())
    {
        return;
    }

    fresh_vision = BallMechanismId_ReadVision(now_ms, &sample);
    BallActuator_GetStatus(&actuator);

    switch (s_status.state)
    {
    case BALL_ID_PRECHECK:
        if (!actuator.online || !actuator.firmware_x || !actuator.foc_closed_loop)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_ACTUATOR_NOT_READY, now_ms);
        }
        else if (!fresh_vision)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_VISION_STALE, now_ms);
        }
        else if (!BallActuator_Enable(true))
        {
            BallMechanismId_Fail(BALL_ID_FAULT_ENABLE_COMMAND, now_ms);
        }
        else
        {
            BallMechanismId_Enter(BALL_ID_ENABLE_SETTLE, now_ms);
        }
        break;

    case BALL_ID_ENABLE_SETTLE:
        if (!fresh_vision)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_VISION_STALE, now_ms);
        }
        else if (BallMechanismId_Expired(now_ms, BALL_ID_ENABLE_SETTLE_MS))
        {
            if (!BallActuator_ZeroAtCurrentPosition())
            {
                BallMechanismId_Fail(BALL_ID_FAULT_ZERO_COMMAND, now_ms);
            }
            else
            {
                BallMechanismId_Enter(BALL_ID_ZERO_GAP, now_ms);
            }
        }
        break;

    case BALL_ID_ZERO_GAP:
        if (!fresh_vision)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_VISION_STALE, now_ms);
        }
        else if (BallMechanismId_Expired(now_ms, BALL_ID_COMMAND_GAP_MS))
        {
            if (!BallActuator_PrepareAbsolutePosition(BALL_ID_ACCEL_RPM_S,
                                                      BALL_ID_DECEL_RPM_S,
                                                      BALL_ID_MAX_SPEED_RPM,
                                                      BALL_ID_MAX_CURRENT_MA))
            {
                BallMechanismId_Fail(BALL_ID_FAULT_PARAMETER_COMMAND, now_ms);
            }
            else
            {
                BallMechanismId_Enter(BALL_ID_PARAM_GAP, now_ms);
            }
        }
        break;

    case BALL_ID_PARAM_GAP:
        if (!fresh_vision)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_VISION_STALE, now_ms);
        }
        else if (BallMechanismId_Expired(now_ms, BALL_ID_COMMAND_GAP_MS))
        {
            BallMechanismId_Enter(BALL_ID_WAIT_POINT, now_ms);
        }
        break;

    case BALL_ID_WAIT_POINT:
    case BALL_ID_WAIT_NEGATIVE:
        /* User places the ball at the requested starting position, then
             * PA31 calls TriggerPoint.  No motion occurs while waiting. */
        break;

    case BALL_ID_HOLD_POINT:
        if (!fresh_vision)
        {
            BallMechanismId_Fail(BALL_ID_FAULT_VISION_STALE, now_ms);
        }
        else if (BallMechanismId_Expired(now_ms, BALL_ID_HOLD_MS))
        {
            s_status.end_cm = sample.position_cm;
            s_status.delta_cm = s_status.end_cm - s_status.start_cm;
            s_status.moved = BallMechanismId_Abs(s_status.delta_cm) >= BALL_ID_MOVED_THRESHOLD_CM;
            ++s_status.point_sequence;
            if (s_status.direction > 0)
            {
                s_status.positive_safe_deg = s_status.next_angle_deg;
            }
            else
            {
                s_status.negative_safe_deg = s_status.next_angle_deg;
            }
            if ((s_status.next_angle_deg + BALL_ID_ANGLE_STEP_DEG) > BALL_ID_OPERATOR_CEILING_DEG)
            {
                /* Reaching the hard guard always returns the pendulum
                     * to zero before changing direction or ending. */
                s_status.target_angle_deg = 0.0f;
                if (!BallActuator_SetAbsoluteAngleDeg(0.0f))
                {
                    BallMechanismId_Fail(BALL_ID_FAULT_POSITION_COMMAND, now_ms);
                }
                else
                {
                    s_result_log_pending = true;
                    BallMechanismId_Enter(BALL_ID_RETURN_SETTLE, now_ms);
                }
            }
            else
            {
                /* Keep the current angle applied.  The next PA31 press
                     * commands the next absolute angle, e.g. +1 to +2 deg. */
                s_status.next_angle_deg += BALL_ID_ANGLE_STEP_DEG;
                s_result_log_pending = true;
                BallMechanismId_Enter(BALL_ID_WAIT_POINT, now_ms);
            }
        }
        break;

    case BALL_ID_RETURN_SETTLE:
        if (BallMechanismId_Expired(now_ms, BALL_ID_RETURN_SETTLE_MS))
        {
            if (s_status.direction > 0)
            {
                BallMechanismId_Enter(BALL_ID_WAIT_NEGATIVE, now_ms);
            }
            else
            {
                (void)BallActuator_Stop();
                (void)BallActuator_Enable(false);
                BallActuator_SetCommandSession(false);
                BallMechanismId_Enter(BALL_ID_COMPLETE, now_ms);
            }
        }
        break;

    default:
        BallMechanismId_Fail(BALL_ID_FAULT_POSITION_COMMAND, now_ms);
        break;
    }
}

void BallMechanismId_GetStatus(BallMechanismIdStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

const char *BallMechanismId_StateName(BallMechanismIdState state)
{
    switch (state)
    {
    case BALL_ID_IDLE:
        return "IDLE";
    case BALL_ID_PRECHECK:
        return "CHECK";
    case BALL_ID_ENABLE_SETTLE:
        return "ENABLE";
    case BALL_ID_ZERO_GAP:
        return "ZERO";
    case BALL_ID_PARAM_GAP:
        return "PARAM";
    case BALL_ID_WAIT_POINT:
        return "WAIT";
    case BALL_ID_HOLD_POINT:
        return "HOLD";
    case BALL_ID_RETURN_SETTLE:
        return "RETURN";
    case BALL_ID_WAIT_NEGATIVE:
        return "NEGWAIT";
    case BALL_ID_COMPLETE:
        return "DONE";
    case BALL_ID_FAULT:
        return "FAULT";
    case BALL_ID_ABORTED:
        return "ABORT";
    default:
        return "UNKNOWN";
    }
}
