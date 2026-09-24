#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

/* Chassis motion-control facade for Q2 route following and vehicle tasks.
 * Speed arguments use mm/s; heading arguments use degrees. Timer callbacks
 * execute the upper control loop, so only Control_QuickBrake() is intended
 * for immediate use in that interrupt context. Control_Stop() busy-waits and
 * is foreground-only.
 */

/* 速度内环参数。 */
#define SPEED_P 5.0f
#define SPEED_I 0.5f
#define SPEED_D 0.30f

/* 上层环参数。 */
#define YAW_P 15.0f
#define YAW_D 4.0f
#define YAW_K 0.50f
#define YAW_DEADBAND 0.8f
#define TRACE_P 12.0f
#define TRACE_D 4.0f
#define TRACE_FILTER_ALPHA 0.35f

#define MOTOR_TARGET_MAX 800.0f
#define STOP_BRAKE_MS 80U

#define SENSOR_ALL_WHITE 0x00U
#define WHITE_CONFIRM_COUNT 10U
#define BLACK_CONFIRM_COUNT 10U
#define STRAIGHT_GUARD_COUNT 50U

#define CONTROL_EVENT_NONE 0U
#define CONTROL_EVENT_BLACK 1U
#define CONTROL_EVENT_WHITE 2U

extern volatile uint8_t control_enable;
extern volatile uint8_t control_auto_enable;
extern volatile uint8_t control_yaw_enable;
extern volatile uint8_t control_trace_enable;
extern volatile float control_target_yaw;
extern volatile float control_trace_error;

/* Start wheel-speed control with the requested heading (deg) and base speed
 * (mm/s); upper loops remain disabled until selected by the caller. */
void Control_Start(float yaw, float speed);
/* Bounded PWM/GPIO short-brake action; safe for the existing timer ISR. */
void Control_QuickBrake(void);
/* Stop and then release the motor outputs; foreground-only due to delay_ms. */
void Control_Stop(void);
void Control_YawStart(float yaw, float speed);
void Control_AutoStart(float speed);
void Control_TraceStart(float speed);
void Control_SetBaseSpeed(float speed);

/* Run one upper-loop update. These functions are called by the existing
 * control timer ISR; return 1 when the corresponding line transition is seen. */
uint8_t Control_StraightRun(void);
uint8_t Control_TraceRun(void);
void Control_ToTrace(void);
void Control_ToStraight(void);
void Control_UpperRun(void);
uint8_t Control_GetEvent(void);

/* Enable and start the existing SysConfig control timer. */
void Control_TimerInit(void);

#endif
