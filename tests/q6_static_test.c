/* Q6 bench image check: with Q6_STATIC_TEST_MODE=1 the Q6 ISR path must
 * keep the chassis parked (base speed 0) while the ball loop still runs.
 * Compiled with -DQ6_STATIC_TEST_MODE=1 by run_q6_checks.ps1. */
#include <math.h>
#include <string.h>
#include "app/q4/q45_vehicle_control.h"
#include "app/q4/q45_vehicle_config.h"
#include "app/ball_actuator/ball_actuator.h"
#include "code/vision_link.h"
#include "code/licheng.h"
#include "app/q6/q6_target.h"

#ifndef Q6_STATIC_TEST_MODE
#error "this image must be compiled with -DQ6_STATIC_TEST_MODE=1"
#endif

static VisionLinkSample vision;
static BallActuatorStatus actuator;
static float chassis_speed;
static unsigned failures, first_failure_line, checks;
unsigned failure_lines[32];
unsigned failure_lines_count;
float E_speed[2];
LichengData_t g_licheng;
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        ++checks;                                                                                  \
        if (!(x))                                                                                  \
        {                                                                                          \
            if (failure_lines_count < 32U)                                                         \
                failure_lines[failure_lines_count] = __LINE__;                                     \
            ++failure_lines_count;                                                                 \
            ++failures;                                                                            \
            if (!first_failure_line)                                                               \
                first_failure_line = __LINE__;                                                     \
        }                                                                                          \
    } while (0)
bool VisionLink_GetLatest(VisionLinkSample *s)
{
    *s = vision;
    return vision.valid;
}
void BallActuator_GetStatus(BallActuatorStatus *s)
{
    *s = actuator;
}
bool BallActuator_ClearProtection(void)
{
    return true;
}
bool BallActuator_Enable(bool e)
{
    (void)e;
    return true;
}
bool BallActuator_Stop(void)
{
    return true;
}
bool BallActuator_ZeroAtCurrentPosition(void)
{
    actuator.actual_angle_deg = 0;
    return true;
}
bool BallActuator_ReturnAbsoluteZero(void)
{
    actuator.actual_angle_deg = 0;
    return true;
}
void BallActuator_SetCommandSession(bool e)
{
    (void)e;
}
bool BallActuator_PrepareAbsolutePosition(uint16_t a, uint16_t d, float v, uint16_t i)
{
    (void)a;
    (void)d;
    (void)v;
    (void)i;
    return true;
}
bool BallActuator_SetAbsoluteAngleDeg(float a)
{
    actuator.actual_angle_deg = a;
    return true;
}
void Control_TraceStart(float v)
{
    chassis_speed = v;
}
void Control_SetBaseSpeed(float v)
{
    chassis_speed = v;
}
void Control_Stop(void)
{
    chassis_speed = 0.0f;
}
void Licheng_Reset(void)
{
    memset(&g_licheng, 0, sizeof(g_licheng));
}

static unsigned stamp_ms;
static void Feed(unsigned t, float position)
{
    vision.rx_time_ms = t;
    vision.position_cm = position;
    stamp_ms += 33U;
    vision.source_timestamp_ms = (uint16_t)stamp_ms;
    ++vision.sequence;
    actuator.last_position_ms = t;
    Q45Vehicle_Task(t);
}

int main(void)
{
    Q45VehicleStatus s;
    int i, n;

    memset(&vision, 0, sizeof(vision));
    memset(&actuator, 0, sizeof(actuator));
    vision.valid = true;
    vision.confidence_pct = 95;
    vision.pixel_cy = 165U;
    vision.rx_time_ms = 1000;
    actuator.online = true;
    actuator.options_valid = true;
    actuator.firmware_x = true;
    actuator.foc_closed_loop = true;
    actuator.actual_angle_valid = true;
    actuator.last_position_ms = 1000;
    E_speed[0] = E_speed[1] = 0;
    chassis_speed = -1;
    stamp_ms = 500;
    Q45Vehicle_Init();
    CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 3.0f));
    CHECK(Q45Vehicle_SelectMode(Q45_VEHICLE_MODE_Q6, 1000));
    Q45Vehicle_Task(1000);
    Q45Vehicle_Task(1100);
    Q45Vehicle_Task(1120);
    Q45Vehicle_Task(1130);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_READY);
    for (i = 0; i < 16; i++)
        Feed(1160 + i * 40, 3.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && s.target_cm == 3.0f);

    /* Parked chassis: the static image zeroes the Q6 ISR base speed while
     * the ball hold loop keeps producing nonzero commands. */
    for (n = 0; n < 30; n++)
    {
        Q45Vehicle_Tick10msISR();
        CHECK(chassis_speed == 0.0f);
    }
    Feed(1810, 3.0f);
    Feed(1840, 2.0f);
    Feed(1870, 2.4f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING);
    CHECK(s.elapsed_ms == 300U);
    Q45Vehicle_Tick10msISR();
    CHECK(chassis_speed == 0.0f);
    /* The ball loop still tracks: parked 0.6cm off target in band. */
    CHECK(s.in_scoring_band);

    return failures ? 1 : 0;
}
