#ifndef LAP_TRACE_H
#define LAP_TRACE_H

#include <stdint.h>

/* Q2 lap-distance/time state machine. Start/reset are foreground operations;
 * LapTrace_Tick10ms is called from the existing 10 ms timer ISR. Distances
 * are cm, elapsed/limit values are ms. ISR-shared state is not a task queue.
 */

/* 第二问：八路灰度循迹一圈。速度单位与 E_speed 一致，均为 mm/s。 */
#define LAP_TRACE_TARGET_SPEED 400.0f

/* 图纸一圈中心线：2×1.5 m + 2π×0.5 m = 614.16 cm。 */
#define LAP_TRACE_LENGTH_CM 584.16f
/* 实车停车超过 A 线就增大本值，停在 A 线前就减小本值。 */
#define LAP_TRACE_BRAKE_COMP_CM 3.00f
#define LAP_TRACE_STOP_DISTANCE_CM (LAP_TRACE_LENGTH_CM - LAP_TRACE_BRAKE_COMP_CM)
#define LAP_TRACE_TIME_LIMIT_MS 20000UL

typedef enum
{
    LAP_TRACE_IDLE = 0,
    LAP_TRACE_RUNNING,
    LAP_TRACE_FINISHED,
    LAP_TRACE_TIMEOUT
} LapTraceState_t;

/* Return the state machine to IDLE and restore configured defaults. */
void LapTrace_Reset(void);
void LapTrace_Start(void);
/* Start with explicit limits; non-positive distance or zero time selects the
 * module defaults, matching the existing Q2 call contract. */
void LapTrace_StartToDistance(float stop_distance_cm, uint32_t time_limit_ms);
/* Advance one 10 ms scheduler step from interrupt context. */
void LapTrace_Tick10ms(void);

uint32_t LapTrace_GetElapsedMs(void);
LapTraceState_t LapTrace_GetState(void);
/* One while the terminal state requests chassis braking. */
uint8_t LapTrace_StopRequested(void);

#endif
