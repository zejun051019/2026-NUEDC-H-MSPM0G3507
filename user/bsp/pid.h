#ifndef PID_H
#define PID_H

#include <stdint.h>

/* Fixed-state PID primitives and the existing chassis speed-loop state.
 * pid_control() runs from the encoder speed timer ISR; calculation helpers
 * update the caller-owned pid_ctrl_t in place and allocate no memory.
 */

enum
{
    POSITION_PID = 0,
    DELTA_PID
};

typedef struct
{
    float target;
    float now;
    float error[3];
    float p;
    float i;
    float d;
    float pout;
    float iout;
    float dout;
    float out;
    uint32_t pid_mode;
} pid_ctrl_t;

/* 三类控制环：速度环、航向环、灰度环。 */
extern pid_ctrl_t motorA;
extern pid_ctrl_t motorB;
extern pid_ctrl_t angle;
extern pid_ctrl_t trace_pid;

/* Initialize controller state and gains. */
void pid_init(pid_ctrl_t *pid, uint32_t mode, float p, float i, float d);
void pid_cal(pid_ctrl_t *pid);
void pid_cal_angle(pid_ctrl_t *pid);
void pid_cal_trace(pid_ctrl_t *pid);
void pidout_limit(pid_ctrl_t *pid);

/* Called by the existing speed timer ISR; reads encoder speeds and writes PWM. */
void pid_control(void);
void motor_target_set(float left, float right);

/* 计算最短航向角误差。 */
float Yaw_error_zzk(float target, float now);

#endif
