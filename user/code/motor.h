#ifndef MOTOR_H
#define MOTOR_H

#include "headfile.h"

/* Two-channel chassis H-bridge/PWM adapter. Duty is signed, clamped by the
 * driver to the configured PWM period; zero coasts, Motor_Brake() applies
 * the existing short-brake pin state. Calls perform bounded register writes.
 */

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define MOTOR_FORWARD 1
#define MOTOR_BACKWARD 0
#define MOTOR_STOPS 2

void Motor_Init(void);
/* Signed PWM command: positive forward, negative reverse, zero coast. */
void Motor_Setduty(uint8_t motor, int16_t speed);
void Motor_Stop(void);
void Motor_Brake(void);

#endif
