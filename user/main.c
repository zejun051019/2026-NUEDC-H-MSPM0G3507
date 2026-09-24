#include "ti_msp_dl_config.h"

#include <math.h>
#include <stdio.h>

#include "bsp/control.h"
#include "bsp/app_clock.h"
#include "bsp/interrupt.h"
#include "bsp/lap_trace.h"
#include "code/delay.h"
#include "code/debug_log.h"
#include "code/encoder.h"
#include "code/licheng.h"
#include "code/motor.h"
#include "code/oled.h"
#include "code/vision_link.h"
#include "app/ball_actuator/ball_actuator.h"
#include "app/ball_actuator/pendulum_calibration.h"
#include "app/app_key_input.h"
#include "app/app_status_display.h"
#include "app/app_telemetry.h"
#include "app/q3/ball_position_control.h"
#include "app/q3/ball_control_config.h"
#include "app/q3/ball_mechanism_id.h"
#include "app/q3/pendulum_geometry_calibration.h"
#include "app/q3/ball_pulse_id.h"
#include "app/q3/q3_sequence.h"
#include "app/q4/q45_vehicle_control.h"
#include "app/q6/q6_config.h"

/*
 * 10 ms cooperative scheduler: chassis functions and ball-position control
 * are both application tasks.  Motor UART RX remains interrupt driven.
 */

static uint8_t g_stop_handled;

typedef enum
{
    APP_TASK_NONE = 0,
    APP_TASK_Q2,
    APP_TASK_Q3,
    APP_TASK_Q4,
    APP_TASK_Q5,
    APP_TASK_Q6
} AppTaskMode;

/* Q3 keeps its controller and parameters frozen.  This small UI flag only
 * implements the common Q3/Q4/Q5 two-press launch contract. */
static AppTaskMode g_selected_task;
static bool g_q3_launch_armed;
#if Q6_TASK_ENABLE && Q6_BOOT_AUTO_SELECT
static bool g_q6_boot_selected;
#endif
#if Q6_TASK_ENABLE
static bool g_q6_pb24_start_pending;
#endif
/* First PA31 press latches a request.  It survives until the X42S has
 * reported ready and acknowledged the fresh current-position zero mark, so a
 * reset followed by one early key press never requires a second setup press. */
static bool g_q3_zero_requested;
static uint32_t g_q3_started_ms;
static uint32_t g_q3_elapsed_ms;
static bool g_q3_timer_running;

static bool Q3Key_ConsumePress(uint32_t now_ms)
{
    static bool candidate;
    static bool stable;
    static uint32_t changed_ms;
    bool pressed = DL_GPIO_readPins(KEY_KEY_T4_PORT, KEY_KEY_T4_PIN) == 0U;
    g_key_t4 = 0U;
    if (pressed != candidate)
    {
        candidate = pressed;
        changed_ms = now_ms;
    }
    if (stable != candidate && now_ms - changed_ms >= 20U)
    {
        stable = candidate;
        return stable;
    }
    return false;
}

static const char *Lap_StateText(void)
{
    switch (LapTrace_GetState())
    {
    case LAP_TRACE_RUNNING:
        return "Q2 RUN";
    case LAP_TRACE_FINISHED:
        return "Q2 PASS";
    case LAP_TRACE_TIMEOUT:
        return "Q2 TIMEOUT";
    default:
        return "READY";
    }
}

static bool AppVisionAtCenter(uint32_t now_ms)
{
    VisionLinkSample sample;

    return VisionLink_GetLatest(&sample) && sample.valid && isfinite(sample.position_cm) &&
           (sample.confidence_pct >= BALL_CONTROL_CONFIDENCE_MIN) &&
           ((now_ms - sample.rx_time_ms) <= BALL_CONTROL_VISION_TIMEOUT_MS) &&
           (fabsf(sample.position_cm) <= BALL_CONTROL_START_WINDOW_CM);
}

static void Show_Status(void)
{
    Q45VehicleStatus vehicle;
    Q3SequenceStatus q3;
    char line[APP_STATUS_DISPLAY_LINE_CAPACITY];

    Q45Vehicle_GetStatus(&vehicle);
    OLED_Clear();
    OLED_ShowString(0, 0, (u8 *)"MSPM0 26HT", 12);
    if (((g_selected_task == APP_TASK_Q4) || (g_selected_task == APP_TASK_Q5) ||
         (g_selected_task == APP_TASK_Q6)) &&
        (vehicle.mode != Q45_VEHICLE_MODE_NONE))
    {
        AppStatusDisplay_ShowVehicle(&vehicle, g_selected_task == APP_TASK_Q6, line);
    }
    else if (g_selected_task == APP_TASK_Q3)
    {
        Q3Sequence_GetStatus(&q3);
        OLED_ShowString(0, 16, (u8 *)"Q3", 12);
        OLED_ShowString(32,
                        16,
                        (u8 *)(g_q3_zero_requested ? "ZEROING"
                               : g_q3_launch_armed
                                   ? (AppVisionAtCenter(AppClock_NowMs()) ? "READY" : "CENTER")
                                   : Q3Sequence_StateName(q3.state)),
                        12);
        (void)snprintf(line,
                       sizeof(line),
                       "T:%lus %s",
                       (unsigned long)(g_q3_elapsed_ms / 1000U),
                       g_q3_zero_requested ? "WAIT" : (g_q3_launch_armed ? "2ND KEY" : "1ST KEY"));
        OLED_ShowString(0, 32, (u8 *)line, 12);
    }
    else
    {
        OLED_ShowString(0, 16, (u8 *)"Q2 TRACE", 12);
        OLED_ShowString(0, 32, (u8 *)Lap_StateText(), 12);
    }
    OLED_Refresh();
}

static void Stop_All(void)
{
    if (control_enable != 0U)
    {
        Control_Stop();
    }
    LapTrace_Reset();
    if (Q45Vehicle_IsActive())
    {
        Q45Vehicle_Stop(AppClock_NowMs());
    }
    g_stop_handled = 1U;
}

static void Start_Q2(void)
{
    Stop_All();
    Q3Sequence_Stop(AppClock_NowMs());
    BallPositionControl_Stop(AppClock_NowMs());
    g_q3_launch_armed = false;
    g_q3_zero_requested = false;
    g_q3_started_ms = 0U;
    g_q3_elapsed_ms = 0U;
    g_q3_timer_running = false;
    g_selected_task = APP_TASK_Q2;
    g_stop_handled = 0U;
    Licheng_Reset();
    LapTrace_StartToDistance(LAP_TRACE_STOP_DISTANCE_CM, LAP_TRACE_TIME_LIMIT_MS);
    Control_TraceStart(LAP_TRACE_TARGET_SPEED);
}

static void Start_Q45(Q45VehicleMode mode, uint32_t now_ms)
{
    Q45VehicleStatus vehicle;

    Q45Vehicle_GetStatus(&vehicle);
    if ((vehicle.mode == mode) && Q45Vehicle_IsActive())
    {
        /* The same selected key has exactly one second action: launch.  A
         * press before READY or while running is intentionally ignored. */
        if (vehicle.state == Q45_VEHICLE_READY)
        {
            (void)Q45Vehicle_StartSelected(now_ms);
        }
        return;
    }

    /* Q3 and the vehicle controller must never send concurrent X42S targets. */
    Stop_All();
    Q3Sequence_Stop(now_ms);
    BallPositionControl_Stop(now_ms);
    BallMechanismId_Abort(now_ms);
    PendulumGeometryCalibration_Abort(now_ms);
    BallPulseId_Abort(now_ms);
    g_q3_launch_armed = false;
    g_q3_zero_requested = false;
    g_q3_started_ms = 0U;
    g_q3_elapsed_ms = 0U;
    g_q3_timer_running = false;
    g_selected_task = (mode == Q45_VEHICLE_MODE_Q4)
                          ? APP_TASK_Q4
                          : ((mode == Q45_VEHICLE_MODE_Q6) ? APP_TASK_Q6 : APP_TASK_Q5);
    if (Q45Vehicle_SelectMode(mode, now_ms))
    {
        g_stop_handled = 0U;
    }
}

int main(void)
{
    uint32_t display_last_ms = 0U;
    uint32_t app_time_ms = 0U;
    uint32_t key_start_last_ms = 0U;
    uint32_t key_t1_last_ms = 0U;
    uint32_t key_t2_last_ms = 0U;
    uint32_t key_t3_last_ms = 0U;
    uint32_t key_t4_last_ms = 0U;

    SYSCFG_DL_init();
    Motor_Init();
    Encoder_Init();
    Licheng_Init();
    Control_TimerInit();
    OLED_Init();
    VisionLink_Init();
    DebugLog_Init();
    BallActuator_Init(1U);
    PendulumCalibration_Init();
    BallPositionControl_Init();
    BallMechanismId_Init();
    PendulumGeometryCalibration_Init();
    BallPulseId_Init();
    Q3Sequence_Init();
    Q45Vehicle_Init();
    LapTrace_Reset();

    enable_group1_irq = 1U;
    Interrupt_Init();

    while (1)
    {
        app_time_ms = AppClock_NowMs();
        AppTelemetry_BeginCycle();
        VisionLink_SetNow(app_time_ms);
        VisionLink_Poll();
        /* Fast path: UART3 RX has already assembled the newest vision frame.
         * Refresh X42S feedback then calculate/send the outer position loop
         * before any optional identification or calibration task executes. */
        BallActuator_Task(app_time_ms);
        BallPositionControl_Task(app_time_ms);
        Q3Sequence_Task(app_time_ms);
        if (g_q3_timer_running)
        {
            Q3SequenceStatus q3;

            Q3Sequence_GetStatus(&q3);
            if (q3.completion_ms != 0U)
            {
                /* The contest time ends at first stationary entry into the
                 * measured -5 +/- 1 cm band.  X42S can return to its PA31
                 * zero afterwards without extending the OLED stopwatch. */
                g_q3_elapsed_ms = q3.completion_ms;
                g_q3_timer_running = false;
            }
            else
            {
                g_q3_elapsed_ms = app_time_ms - g_q3_started_ms;
                if (!Q3Sequence_IsActive())
                {
                    /* Fault/STOPPED before a qualified final position also
                     * freezes the stopwatch rather than leaving it running. */
                    g_q3_timer_running = false;
                }
            }
        }
        /* Q4/Q5/Q6 vehicle modes own X42S only after their task-key request.
         * The archived static-recovery test is compiled out of this image. */
#if Q6_TASK_ENABLE && Q6_BOOT_AUTO_SELECT
        /* Bench image only: enter Q6 without a key press so a flash reset
         * starts the whole hold trial autonomously. */
        if ((app_time_ms >= 1000U) && !g_q6_boot_selected)
        {
            g_q6_boot_selected = true;
#if (Q6_BENCH_EXPLICIT_TARGET_MM != -32768)
            /* Bench image only: pin the hold target to the requested
             * value; the move profiles carry the ball there after start. */
            (void)Q45Vehicle_Q6ConfigureTarget(Q6_TARGET_EXPLICIT,
                                               (float)Q6_BENCH_EXPLICIT_TARGET_MM / 10.0f);
#endif
            Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
        }
#endif
        Q45Vehicle_Task(app_time_ms);
        DebugLog_PublishControlFrame(app_time_ms);

        /* These independent test modules are normally inactive during Q3.
         * Keeping them after the fast path prevents them adding control
         * latency when a new camera sample arrives. */
        PendulumCalibration_Task(app_time_ms);
        BallMechanismId_Task(app_time_ms);
        PendulumGeometryCalibration_Task(app_time_ms);
        BallPulseId_Task(app_time_ms);
        DebugLog_Task();

        /* Keep slow diagnostic publication after the per-loop queue drain. */
        AppTelemetry_PublishDue(app_time_ms);

        if (AppKeyInput_ConsumePress(KEY_KEY_START_PORT,
                                     KEY_KEY_START_PIN,
                                     &g_key_start,
                                     app_time_ms,
                                     &key_start_last_ms))
        {
            Stop_All();
            Q3Sequence_Stop(app_time_ms);
            BallPositionControl_Stop(app_time_ms);
            BallMechanismId_Abort(app_time_ms);
            PendulumGeometryCalibration_Abort(app_time_ms);
            BallPulseId_Abort(app_time_ms);
            g_q3_launch_armed = false;
            g_q3_zero_requested = false;
            g_q3_started_ms = 0U;
            g_q3_elapsed_ms = 0U;
            g_q3_timer_running = false;
        }

        if (AppKeyInput_ConsumePress(
                KEY_KEY_T1_PORT, KEY_KEY_T1_PIN, &g_key_t1, app_time_ms, &key_t1_last_ms))
        {
            /* PB11 remains the Q5 start key in every build. */
            Start_Q45(Q45_VEHICLE_MODE_Q5, app_time_ms);
        }

        if (AppKeyInput_ConsumePress(
                KEY_KEY_T2_PORT, KEY_KEY_T2_PIN, &g_key_t2, app_time_ms, &key_t2_last_ms))
        {
#if Q6_TASK_ENABLE
            DebugLog_PublishQ6Event(app_time_ms, 1U, 0);
            /* PB24 requests Q6 start; retain it while CAPTURE gathers the
             * stable target window, then launch exactly once. */
            Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
            g_q6_pb24_start_pending = true;
#else
            /* PB24 keeps its standalone Q2 chassis exercise in other images. */
            Start_Q2();
#endif
        }

        /* PA28: contest Q4, A to B while the ball holds O. */
        if (AppKeyInput_ConsumePress(
                KEY_KEY_T3_PORT, KEY_KEY_T3_PIN, &g_key_t3, app_time_ms, &key_t3_last_ms))
        {
            Start_Q45(Q45_VEHICLE_MODE_Q4, app_time_ms);
        }

        /* A first PA31 press requests the new physical zero once.  The
         * request remains active through motor boot/enable feedback, avoiding
         * a race where the user presses PA31 before the first X42S status
         * poll after a reset. */
        if (g_q3_zero_requested && !Q3Sequence_IsActive())
        {
            if (Q3Sequence_ZeroReference(app_time_ms))
            {
                g_q3_zero_requested = false;
                g_q3_launch_armed = true;
                g_q3_started_ms = 0U;
                g_q3_elapsed_ms = 0U;
                g_q3_timer_running = false;
                g_stop_handled = 0U;
            }
        }

        if (Q3Key_ConsumePress(app_time_ms) && !Q3Sequence_IsActive())
        {
            if (Q45Vehicle_IsActive())
            {
                Stop_All();
            }
            g_selected_task = APP_TASK_Q3;
            if (!g_q3_launch_armed)
            {
                /* First PA31 press always creates a new session origin from
                 * the present X42S pose, even after a previous Q3 run. */
                Q3Sequence_Stop(app_time_ms);
                BallPositionControl_Stop(app_time_ms);
                BallMechanismId_Abort(app_time_ms);
                PendulumGeometryCalibration_Abort(app_time_ms);
                BallPulseId_Abort(app_time_ms);
                g_q3_zero_requested = true;
            }
            else if (AppVisionAtCenter(app_time_ms) && Q3Sequence_Start(app_time_ms))
            {
                /* Formal Q3 execution begins only on the second PA31 press
                 * and only after vision has confirmed |x| <= 1 cm. */
                Q3Sequence_EnableAutoCycle(false);
                g_q3_launch_armed = false;
                g_q3_started_ms = app_time_ms;
                g_q3_elapsed_ms = 0U;
                g_q3_timer_running = true;
                g_stop_handled = 0U;
            }
        }

        if ((LapTrace_StopRequested() != 0U) && (g_stop_handled == 0U))
        {
            if (control_enable != 0U)
            {
                Control_Stop();
            }
            LapTrace_Reset();
            g_stop_handled = 1U;
        }

        if (Q45Vehicle_StopRequested() && (g_stop_handled == 0U))
        {
            if (control_enable != 0U)
            {
                Control_Stop();
            }
            g_stop_handled = 1U;
        }

#if Q6_TASK_ENABLE
        if (g_q6_pb24_start_pending)
        {
            Q45VehicleStatus vehicle;

            Q45Vehicle_GetStatus(&vehicle);
            if ((vehicle.mode != Q45_VEHICLE_MODE_Q6) || (vehicle.state >= Q45_VEHICLE_RUNNING))
            {
                g_q6_pb24_start_pending = false;
            }
            else if ((vehicle.state == Q45_VEHICLE_READY) && vehicle.center_ready)
            {
                Start_Q45(Q45_VEHICLE_MODE_Q6, app_time_ms);
                Q45Vehicle_GetStatus(&vehicle);
                g_q6_pb24_start_pending = (vehicle.state == Q45_VEHICLE_READY);
            }
        }
#endif

        /* OLED 刷新保持低频，不影响 10 ms 控制定时。 */
#if Q6_TASK_ENABLE && Q6_BENCH_AUTO_START
        if (g_selected_task == APP_TASK_Q6)
        {
            Q45VehicleStatus vehicle;

            Q45Vehicle_GetStatus(&vehicle);
            if ((vehicle.mode == Q45_VEHICLE_MODE_Q6) && (vehicle.state == Q45_VEHICLE_READY))
            {
                /* Bench image only: a captured target launches the run by
                 * itself; StartSelected() re-checks readiness internally. */
                (void)Q45Vehicle_StartSelected(app_time_ms);
            }
        }
#endif

        if ((app_time_ms - display_last_ms) >= 1000U)
        {
            display_last_ms = app_time_ms;
            Show_Status();
        }
    }
}
