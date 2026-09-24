#ifndef Q5_DYNAMIC_BALL_CONTROL_H
#define Q5_DYNAMIC_BALL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* Stateful Q5 ball controller, independent of peripherals. Inputs use cm,
 * cm/s, m/s^2, seconds, and motor degrees; outputs use beam/motor degrees
 * and predicted cm. Update mutates caller-owned controller state and returns
 * false for invalid input or an unrepresentable geometry result.
 */
typedef struct
{
    float beam_angle_deg;
    float beam_rate_deg_s;
    float integral_beam_deg;
    bool initialized;
} Q5DynamicBallController;

typedef struct
{
    float measured_position_cm;
    float error_cm;
    float velocity_cm_s;
    float parallel_accel_m_s2;
    float dt_s;
    float actual_motor_deg;
} Q5DynamicBallInput;

typedef struct
{
    float p_beam_deg;
    float d_beam_deg;
    float i_beam_deg;
    float feedforward_beam_deg;
    float raw_beam_deg;
    float limited_beam_deg;
    float shaped_beam_deg;
    float predicted_position_cm;
    float motor_deg;
    uint8_t profile; /* 0 near-centre, 1 drive, 2 brake (smoothly blended). */
    bool limited;
} Q5DynamicBallOutput;

void Q5DynamicBallControl_Init(Q5DynamicBallController *controller);
/* Computes one bounded control step; no UART, PWM, or global state access. */
bool Q5DynamicBallControl_Update(Q5DynamicBallController *controller,
                                 const Q5DynamicBallInput *input,
                                 Q5DynamicBallOutput *output);

#endif
