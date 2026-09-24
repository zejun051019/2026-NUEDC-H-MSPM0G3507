#ifndef BALL_ACTUATOR_H
#define BALL_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Application-level guard for the X42S pendulum actuator.
 *
 * Option polling verifies UART, firmware and control mode before any motion.
 * Motion calls remain guarded by that verified state.
 */

typedef struct
{
    uint8_t address;
    uint16_t option_flags;
    uint8_t last_function;
    uint8_t last_reply;
    uint8_t status_flags;
    uint32_t requests_sent;
    uint32_t request_timeouts;
    uint32_t position_requests_sent;
    uint32_t position_request_timeouts;
    uint32_t last_option_ms;
    uint32_t last_position_ms;
    uint32_t last_reply_ms;
    float commanded_angle_deg;
    float actual_angle_deg;
    bool options_valid;
    bool commanded_angle_valid;
    bool actual_angle_valid;
    bool online;
    bool firmware_x;
    bool foc_closed_loop;
    bool command_session_active;
} BallActuatorStatus;

/* Initializes the X42S protocol driver.  This function never moves or
 * enables the motor.  Address 0 selects the documented default, address 1. */
void BallActuator_Init(uint8_t address);

/* Call from the 10 ms main-loop scheduler.  It serializes slow option
 * checks with 20 Hz actual-position feedback requests. */
void BallActuator_Task(uint32_t now_ms);

/* Returns a snapshot of the verified read-only actuator state. */
void BallActuator_GetStatus(BallActuatorStatus *status);

/* Pauses background 0x1A option polling while a time-critical sequence is
 * issuing motor commands.  Existing configuration remains valid and every
 * received X42S reply still refreshes the online watchdog. */
void BallActuator_SetCommandSession(bool active);

/* The following commands are guarded by the verified X/FOC configuration.
 * During a commanded session `online` can age out because option polling is
 * deliberately paused; the previously verified configuration still permits
 * a new motion command.  Mechanical limits stay in the controller layer. */
bool BallActuator_Enable(bool enable);
bool BallActuator_ClearProtection(void);
bool BallActuator_Stop(void);
bool BallActuator_ZeroAtCurrentPosition(void);
bool BallActuator_ReturnAbsoluteZero(void);
bool BallActuator_PrepareAbsolutePosition(uint16_t acceleration_rpm_s,
                                          uint16_t deceleration_rpm_s,
                                          float max_speed_rpm,
                                          uint16_t max_current_ma);
bool BallActuator_SetAbsoluteAngleDeg(float target_angle_deg);

#endif
