#include "ball_actuator.h"

#include <string.h>

#include "code/x42s.h"

#define BALL_ACTUATOR_DEFAULT_ADDRESS 1U
#define BALL_ACTUATOR_OPTION_PERIOD_MS 500U
#define BALL_ACTUATOR_POSITION_PERIOD_MS 50U
#define BALL_ACTUATOR_REPLY_TIMEOUT_MS 100U
#define BALL_ACTUATOR_ONLINE_TIMEOUT_MS 1500U

typedef enum
{
    BALL_ACTUATOR_REQUEST_NONE = 0,
    BALL_ACTUATOR_REQUEST_OPTIONS,
    BALL_ACTUATOR_REQUEST_POSITION
} BallActuatorRequest;

static BallActuatorStatus s_status;
static uint32_t s_last_request_ms;
static uint32_t s_last_position_request_ms;
static uint32_t s_last_frame_count;
static uint32_t s_last_option_frame_count;
static uint32_t s_last_position_frame_count;
static BallActuatorRequest s_pending_request;

void BallActuator_Init(uint8_t address)
{
    uint8_t selected_address = (address == 0U) ? BALL_ACTUATOR_DEFAULT_ADDRESS : address;

    memset(&s_status, 0, sizeof(s_status));
    s_status.address = selected_address;
    /* Make the first read-only request occur on the first 10 ms scheduler
     * tick instead of waiting one complete request period after reset. */
    s_last_request_ms = (uint32_t)(0U - BALL_ACTUATOR_OPTION_PERIOD_MS);
    s_last_position_request_ms = (uint32_t)(0U - BALL_ACTUATOR_POSITION_PERIOD_MS);
    s_last_frame_count = 0U;
    s_last_option_frame_count = 0U;
    s_last_position_frame_count = 0U;
    s_pending_request = BALL_ACTUATOR_REQUEST_NONE;

    /* X42S_Init only resets receive state and enables UART0 RX IRQ. */
    X42S_Init(selected_address);
}

void BallActuator_Task(uint32_t now_ms)
{
    const X42S_State *motor = X42S_GetState();
    bool has_new_frame;
    bool has_new_options;
    bool has_new_position;

    X42S_Process();

    has_new_frame = motor->frames_received != s_last_frame_count;
    if (has_new_frame)
    {
        s_last_frame_count = motor->frames_received;
        s_status.last_reply_ms = now_ms;
        s_status.last_function = motor->last_function;
        s_status.last_reply = motor->last_reply;
        s_status.status_flags = motor->status_flags;
    }

    has_new_options = ((motor->valid_mask & X42S_VALID_OPTIONS) != 0U) &&
                      (motor->option_frames_received != s_last_option_frame_count);
    if (has_new_options)
    {
        s_last_option_frame_count = motor->option_frames_received;
        s_status.option_flags = motor->option_flags;
        s_status.options_valid = true;
        s_status.firmware_x = ((motor->option_flags & X42S_OPTION_FIRMWARE_EMM) == 0U);
        s_status.foc_closed_loop = ((motor->option_flags & X42S_OPTION_CLOSED_LOOP) != 0U);
        s_status.last_option_ms = now_ms;
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }

    has_new_position = ((motor->valid_mask & X42S_VALID_POSITION) != 0U) &&
                       (motor->position_frames_received != s_last_position_frame_count);
    if (has_new_position)
    {
        s_last_position_frame_count = motor->position_frames_received;
        s_status.actual_angle_deg = motor->position_deg;
        s_status.actual_angle_valid = true;
        s_status.last_position_ms = now_ms;
        if (s_pending_request == BALL_ACTUATOR_REQUEST_POSITION)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }

    if ((s_pending_request != BALL_ACTUATOR_REQUEST_NONE) &&
        ((now_ms - s_last_request_ms) >= BALL_ACTUATOR_REPLY_TIMEOUT_MS))
    {
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            ++s_status.request_timeouts;
        }
        else
        {
            ++s_status.position_request_timeouts;
        }
        s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
    }

    /* Configuration is checked before position polling starts.  Afterwards
     * the two request kinds share one pending slot, so replies cannot be
     * mistaken for each other.  During motion only position is polled; this
     * avoids injecting a slow option query into a control command stream. */
    if (s_pending_request == BALL_ACTUATOR_REQUEST_NONE)
    {
        if ((!s_status.command_session_active) &&
            ((!s_status.options_valid) ||
             ((now_ms - s_status.last_option_ms) >= BALL_ACTUATOR_OPTION_PERIOD_MS)))
        {
            if (X42S_RequestOptions())
            {
                ++s_status.requests_sent;
                s_last_request_ms = now_ms;
                s_pending_request = BALL_ACTUATOR_REQUEST_OPTIONS;
            }
        }
        else if (s_status.options_valid &&
                 ((now_ms - s_last_position_request_ms) >= BALL_ACTUATOR_POSITION_PERIOD_MS))
        {
            if (X42S_RequestFeedback(X42S_FEEDBACK_POSITION))
            {
                ++s_status.position_requests_sent;
                s_last_request_ms = now_ms;
                s_last_position_request_ms = now_ms;
                s_pending_request = BALL_ACTUATOR_REQUEST_POSITION;
            }
        }
    }

    s_status.online = s_status.options_valid &&
                      ((now_ms - s_status.last_reply_ms) <= BALL_ACTUATOR_ONLINE_TIMEOUT_MS);
}

void BallActuator_GetStatus(BallActuatorStatus *status)
{
    if (status != NULL)
    {
        *status = s_status;
    }
}

void BallActuator_SetCommandSession(bool active)
{
    s_status.command_session_active = active;
    if (active)
    {
        /* A delayed option reply is harmless during a commanded sequence;
         * do not later count it as a timeout caused by intentional polling
         * suspension. */
        if (s_pending_request == BALL_ACTUATOR_REQUEST_OPTIONS)
        {
            s_pending_request = BALL_ACTUATOR_REQUEST_NONE;
        }
    }
}

static bool BallActuator_CommandAllowed(void)
{
    /* During a motion session, option polling is intentionally paused so a
     * query cannot interleave with F1/FC movement frames.  `online` still
     * reports the fresh-reply watchdog truthfully, but the X-firmware identity
     * established before the session is sufficient to issue an F3 command. */
    return (s_status.online || s_status.command_session_active) && s_status.options_valid &&
           s_status.firmware_x;
}

static bool BallActuator_MotionAllowed(void)
{
    return BallActuator_CommandAllowed() && s_status.foc_closed_loop;
}

bool BallActuator_Enable(bool enable)
{
    if (!enable)
    {
        /* A fault stop must be attempted even if the last status is stale. */
        return X42S_Enable(false);
    }
    /* F3 is the command which restores an intentionally disabled X42S.
     * Requiring the already-enabled FOC bit here made a completed Q3 run
     * impossible to prepare again after reset.  Position/zero commands still
     * retain the stricter MotionAllowed closed-loop guard. */
    return BallActuator_CommandAllowed() && X42S_Enable(true);
}

bool BallActuator_ClearProtection(void)
{
    /* X42S manual: 0E 52 clears stall/over-current protection.  It is safe
     * to issue before each motion attempt; no position command is generated. */
    return BallActuator_MotionAllowed() && X42S_ClearProtection();
}

bool BallActuator_Stop(void)
{
    return X42S_Stop();
}

bool BallActuator_ZeroAtCurrentPosition(void)
{
    if (BallActuator_MotionAllowed() && X42S_ZeroCurrentPosition())
    {
        s_status.commanded_angle_deg = 0.0f;
        s_status.commanded_angle_valid = true;
        return true;
    }
    return false;
}

bool BallActuator_ReturnAbsoluteZero(void)
{
    return BallActuator_MotionAllowed() && X42S_ReturnAbsoluteZero();
}

bool BallActuator_PrepareAbsolutePosition(uint16_t acceleration_rpm_s,
                                          uint16_t deceleration_rpm_s,
                                          float max_speed_rpm,
                                          uint16_t max_current_ma)
{
    return BallActuator_MotionAllowed() && X42S_SetQuickPositionParams(acceleration_rpm_s,
                                                                       deceleration_rpm_s,
                                                                       max_speed_rpm,
                                                                       X42S_POSITION_ABSOLUTE,
                                                                       max_current_ma);
}

bool BallActuator_SetAbsoluteAngleDeg(float target_angle_deg)
{
    if (BallActuator_MotionAllowed() && X42S_SetQuickPosition(target_angle_deg))
    {
        s_status.commanded_angle_deg = target_angle_deg;
        s_status.commanded_angle_valid = true;
        return true;
    }
    return false;
}
