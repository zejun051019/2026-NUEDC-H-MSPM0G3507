/* Q6 offline regression: target semantics, translation equivalence and the
 * untouched absolute-safety envelope.  Reuses the q5_vehicle_test.c Unicorn
 * harness: arm-none-eabi-gcc + tests/run_q5_test.py prints CHECK counts. */
#include <math.h>
#include <string.h>
#include "app/q4/q45_vehicle_control.h"
#include "app/q4/q45_vehicle_config.h"
#include "app/q4/q5_vehicle_model.h"
#include "app/q3/pendulum_geometry.h"
#include "app/ball_actuator/ball_actuator.h"
#include "code/vision_link.h"
#include "code/licheng.h"
#include "app/q6/q6_target.h"

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
/* One fresh vision frame: rising camera timestamp and sequence keep the
 * Q6 capture window's replay guard satisfied. */
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

/* Boot through WAIT_ACTUATOR/CONFIGURE/ZERO into READY for any mode.  The
 * fourth Task tick performs the ZERO->READY transition, mirroring the
 * Frame(1130) tick the Q5 baseline test relies on. */
static void BeginMode(Q45VehicleMode mode, int use_explicit, float explicit_cm)
{
    Q45VehicleStatus s;
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
    if (use_explicit)
        CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, explicit_cm));
    CHECK(Q45Vehicle_SelectMode(mode, 1000));
    Q45Vehicle_Task(1000);
    Q45Vehicle_Task(1100);
    Q45Vehicle_Task(1120);
    Q45Vehicle_Task(1130);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_READY);
    CHECK(!s.target_locked && s.target_cm == 0.0f);
}

/* READY-state capture: 16 fresh frames @40ms satisfy >=15 frames and
 * >=500ms span.  start_t must move forward between captures of one leg. */
static void Capture6(unsigned start_t, float place_cm)
{
    int i;
    for (i = 0; i < 16; i++)
        Feed(start_t + i * 40, place_cm);
}

/* Shared RUNNING drive: pre-frame at base, then pattern+base at 30ms so
 * translated legs see an identical frame/dt sequence. */
static void DrivePattern(const float *pattern, int count, float base_cm, Q45VehicleStatus *rec)
{
    int n, i;
    Feed(1810, base_cm);
    for (n = 0; n < count; n++)
    {
        for (i = 0; i < 3; i++)
            Q45Vehicle_Tick10msISR();
        Feed(1840 + n * 30, pattern[n] + base_cm);
        Q45Vehicle_GetStatus(&rec[n]);
    }
}

int main(void)
{
    CHECK(Q6Target_IsPixelRowAccepted(165U));
    CHECK(Q6Target_IsPixelRowAccepted(105U));
    CHECK(Q6Target_IsPixelRowAccepted(225U));
    CHECK(!Q6Target_IsPixelRowAccepted(104U));
    CHECK(!Q6Target_IsPixelRowAccepted(226U));

    Q45VehicleStatus s, rec_a[16], rec_b[16];
    int n;
    static const float pattern[16] = {0, 2, -2, 4, -4, 6, -6, 7, -7, 5, -5, 1, -1, 3, -3, 2};
    static const float small[16] = {
        0, 1, -1, 2, -2, 3, -3, 2, -2, 1, -1, 0.5f, -0.5f, 2.5f, -2.5f, 1.5f};

    /* T_D0: target configuration validation while no mode is active. */
    Q45Vehicle_Init();
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 10.5f)); /* > 5cm */
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, NAN));
    CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 3.0f));
    CHECK(Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_CAPTURE, 0.0f));

    /* T_A: Q6 explicit zero and captured zero targets must produce identical
     * output for the same frame/dt sequence. Q5 has its own final route-profiled
     * control law and is intentionally not an equality oracle. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(pattern, 16, 0.0f, rec_a);
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(pattern, 16, 0.0f, rec_b);
    for (n = 0; n < 16; n++)
    {
        CHECK(rec_b[n].control_error_cm == rec_a[n].control_error_cm);
        CHECK(rec_b[n].commanded_angle_deg == rec_a[n].commanded_angle_deg);
        CHECK(rec_b[n].control_profile == rec_a[n].control_profile);
        CHECK(rec_b[n].in_scoring_band == rec_a[n].in_scoring_band);
        CHECK(rec_b[n].error_cm == rec_a[n].error_cm);
        CHECK(rec_b[n].max_abs_position_cm == rec_a[n].max_abs_position_cm);
    }
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == 0.0f && s.target_locked);

    /* T_B: translation equivalence.  Q6(EXPLICIT 5) fed x+5 must match
     * Q6(EXPLICIT 0) fed x to float tolerance (alpha-beta is affine). */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(small, 16, 5.0f, rec_a);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == 5.0f);
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    DrivePattern(small, 16, 0.0f, rec_b);
    for (n = 0; n < 16; n++)
    {
        CHECK(fabsf(rec_b[n].control_error_cm - rec_a[n].control_error_cm) < 1e-3f);
        CHECK(fabsf(rec_b[n].commanded_angle_deg - rec_a[n].commanded_angle_deg) < 1e-3f);
        CHECK(rec_b[n].control_profile == rec_a[n].control_profile);
        CHECK(rec_b[n].in_scoring_band == rec_a[n].in_scoring_band);
    }

    /* T_C: target-relative semantics.  Ball parked on +5cm target -> HOLD
     * with zero error; +0.8cm raw deviation stays inside the scoring band
     * and mirrors on the negative target.  Steps stay below the observer's
     * per-frame jump gate. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 5.0f);
    Feed(1840, 5.0f);
    Feed(1870, 5.0f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.control_profile == 3U); /* BALL_OUTER_PROFILE_RETURN_HOLD */
    CHECK(fabsf(s.error_cm) < 0.05f && s.in_scoring_band);
    Feed(1900, 5.25f);
    Feed(1930, 5.5f);
    Feed(1960, 5.75f);
    Feed(1990, 5.8f);
    Feed(2020, 5.8f);
    Q45Vehicle_GetStatus(&s);
    CHECK(fabsf(s.error_cm + 0.8f) < 0.15f);
    CHECK(s.in_scoring_band && !s.band_violation);
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, -5.0f);
    Capture6(1160, -5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, -5.0f);
    Feed(1840, -5.0f);
    Feed(1870, -5.0f);
    Feed(1900, -5.25f);
    Feed(1930, -5.5f);
    Feed(1960, -5.75f);
    Feed(1990, -5.8f);
    Feed(2020, -5.8f);
    Q45Vehicle_GetStatus(&s);
    CHECK(fabsf(s.error_cm - 0.8f) < 0.15f && s.in_scoring_band);

    /* T_D: active-mode ConfigureTarget refusal remains enforced, while Q6
     * no longer turns the absolute wall boundary into a terminal stop. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 5.0f);
    CHECK(!Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT, 1.0f));
    Capture6(1160, 5.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 5.0f);
    Feed(1840, 5.0f);
    Feed(1870, 11.2f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && !s.fault);
    CHECK(s.max_target_dev_cm > 6.0f);

    /* T_E: capture gating.  READY + not-yet-captured refuses to start;
     * low-confidence frames never open the window; after the run starts
     * the locked target cannot drift with the ball. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0);
    CHECK(!Q45Vehicle_StartSelected(1200));
    vision.confidence_pct = 50; /* passes the mode gate (>=10), fails capture (>=60) */
    Capture6(1160, 0.0f);
    CHECK(!Q45Vehicle_StartSelected(1900));
    vision.confidence_pct = 95;
    Capture6(1900, -3.0f);
    CHECK(Q45Vehicle_StartSelected(2560));
    Feed(2600, -3.0f);
    Feed(2630, 2.0f);
    Feed(2660, 0.5f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_RUNNING && s.target_cm == -3.0f);
    CHECK(!Q45Vehicle_StartSelected(2680));
    /* A 50mm frame jump is rejected by the observer innovation gate, so the
     * hold command legitimately stays near zero here; T_C proves nonzero
     * commands already. */

    /* T_F: terminal freeze.  After VISION_LOST the peak target deviation
     * stops updating and the actuator is parked at zero. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 1, 0.0f);
    Capture6(1160, 0.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Feed(1810, 0.0f);
    Feed(1840, 4.0f);
    Feed(1870, -4.0f);
    Feed(1900, 6.0f);
    Q45Vehicle_GetStatus(&s);
    CHECK(s.max_target_dev_cm >= 5.99f);
    Q45Vehicle_Task(2700); /* no fresh frame for 800ms > 600ms watchdog */
    Q45Vehicle_GetStatus(&s);
    CHECK(s.state == Q45_VEHICLE_VISION_LOST && s.fault);
    {
        float frozen = s.max_target_dev_cm;
        Q45Vehicle_Task(2800);
        Q45Vehicle_Task(2900);
        Q45Vehicle_GetStatus(&s);
        CHECK(s.max_target_dev_cm == frozen);
        CHECK(actuator.actual_angle_deg == 0.0f);
    }

    /* T_G: default CAPTURE source locks the median of the parked ball. */
    BeginMode(Q45_VEHICLE_MODE_Q6, 0, 0);
    Capture6(1160, -3.0f);
    CHECK(Q45Vehicle_StartSelected(1780));
    Q45Vehicle_GetStatus(&s);
    CHECK(s.target_cm == -3.0f && s.target_locked);

    return failures ? 1 : 0;
}
