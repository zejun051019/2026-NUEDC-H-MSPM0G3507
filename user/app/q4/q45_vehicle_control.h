#ifndef Q45_VEHICLE_CONTROL_H
#define Q45_VEHICLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "app/q6/q6_target.h"

/*
 * Shared Q4/Q5/Q6 vehicle task and ball-control adapter.
 *
 * Units: API timestamps are milliseconds; position and target values are cm;
 * velocity is cm/s; actuator angles are degrees. The caller owns a valid
 * output pointer for status snapshots. Init/SelectMode/Start/Task/Stop run
 * in the main-loop context. Q45Vehicle_Tick10msISR is the only ISR entry and
 * must remain bounded and free of UART transmission or blocking calls.
 * Task and mode-selection calls may issue actuator/chassis commands.
 */

typedef enum
{
    Q45_VEHICLE_MODE_NONE = 0,
    Q45_VEHICLE_MODE_Q4,
    Q45_VEHICLE_MODE_Q5,
    Q45_VEHICLE_MODE_Q6
} Q45VehicleMode;

typedef enum
{
    Q45_VEHICLE_IDLE = 0,
    Q45_VEHICLE_WAIT_ACTUATOR,
    Q45_VEHICLE_CONFIGURE,
    Q45_VEHICLE_ZERO,
    /* The selected mode's actuator zero sequence is complete. Q4/Q5 mark the
     * current pose; Q6 returns to its stored absolute origin. Q4/Q5 await a
     * start request; the normal Q6 key request remains pending through target
     * capture and starts when readiness is reached. */
    Q45_VEHICLE_READY,
    Q45_VEHICLE_RUNNING,
    Q45_VEHICLE_FINISHED,
    Q45_VEHICLE_TIMEOUT,
    Q45_VEHICLE_VISION_LOST,
    Q45_VEHICLE_FAULT,
    Q45_VEHICLE_STOPPED
} Q45VehicleState;

typedef struct
{
    Q45VehicleState state;
    Q45VehicleMode mode;
    uint32_t state_started_ms;
    uint32_t run_sequence;
    uint32_t elapsed_ms;       /* Formal run time, from accepted StartSelected(). */
    uint32_t drive_elapsed_ms; /* From the first chassis command. */
    uint32_t pass_elapsed_ms;  /* B for Q4, A for Q5. */
    float route_distance_cm;
    float base_speed_mm_s;
    float actual_speed_mm_s;
    float longitudinal_accel_m_s2;
    float modeled_turn_rate_deg_s;
    float lateral_accel_m_s2;
    float parallel_accel_m_s2;
    float raw_position_cm;
    float measured_cm;
    float measured_velocity_cm_s;
    float predicted_position_cm;
    /* Peak absolute displacement from the formal run's start sample; retained
     * after a terminal state for result reporting. */
    float max_abs_position_cm;
    float error_cm;
    float control_error_cm;
    float p_beam_deg;
    float d_beam_deg;
    float i_beam_deg;
    float feedforward_beam_deg;
    float requested_beam_deg;
    float control_output_deg;
    float commanded_angle_deg;
    float actual_angle_deg;
    uint8_t control_profile;
    uint8_t stiction_count;
    uint8_t speed_governor_level;
    uint8_t vision_confidence_pct;
    uint8_t estimator_rejected_count;
    uint8_t vision_sequence;
    uint32_t vision_age_ms;
    uint32_t accepted_age_ms;
    uint32_t actual_angle_age_ms;
    uint32_t max_frame_gap_ms;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    uint8_t stop_reason;
    bool route_passed;
    bool center_ready;
    bool in_scoring_band;
    bool band_violation;
    bool control_output_limited;
    bool terminal_zero_confirmed;
    bool actuator_paused;
    bool acceleration_feedforward_enabled;
    bool stop_requested;
    bool fault;
    /* Q6 only: locked hold target and peak raw deviation from it.  Q4/Q5
     * keep target_cm 0 and never raise target_locked. */
    float target_cm;
    float max_target_dev_cm;
    bool target_locked;
} Q45VehicleStatus;

/* Initialize software state; call once before selecting a vehicle mode. */
void Q45Vehicle_Init(void);
/* Selects Q4/Q5/Q6 only while inactive and enters WAIT_ACTUATOR.
 * now_ms is the monotonic millisecond timestamp; false means rejected mode
 * or an already-active run. */
bool Q45Vehicle_SelectMode(Q45VehicleMode mode, uint32_t now_ms);
/* Configure Q6 target source before selecting Q6. Refused while active, for
 * an unsupported source, or for a non-finite/out-of-range explicit target. */
bool Q45Vehicle_Q6ConfigureTarget(Q6TargetSource source, float explicit_cm);
/* Starts formal timing/control from READY after center, actuator-zero, and
 * (for Q6) target-lock preconditions are met. May issue hardware commands. */
bool Q45Vehicle_StartSelected(uint32_t now_ms);
/* Advance foreground state/control logic; call regularly from the main loop.
 * May issue actuator UART and chassis commands. */
void Q45Vehicle_Task(uint32_t now_ms);
/* Called only by the existing 10 ms chassis timer ISR. Bounded bookkeeping
 * and speed update only; must not perform UART transmission. */
void Q45Vehicle_Tick10msISR(void);
/* Request operator stop from foreground context; inactive modes are ignored. */
void Q45Vehicle_Stop(uint32_t now_ms);
/* Read-only state queries; StopRequested remains set until reinitialization or
 * the next mode selection. */
bool Q45Vehicle_IsActive(void);
bool Q45Vehicle_StopRequested(void);
/* Returns and clears the one-shot terminal-result logging request. */
bool Q45Vehicle_ConsumeResultLog(void);
/* Copies a status snapshot when status is non-null. */
void Q45Vehicle_GetStatus(Q45VehicleStatus *status);
/* Return stable, null-terminated display labels for enum values. */
const char *Q45Vehicle_StateName(Q45VehicleState state);
const char *Q45Vehicle_ModeName(Q45VehicleMode mode);

#endif
