#ifndef Q6_TARGET_H
#define Q6_TARGET_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Q6 stable-target selection and capture logic; this module does not drive
 * hardware. Positions are cm, velocities cm/s, and timestamps ms. Call
 * Init/Update/Lock/GetStatus from the main loop, not an ISR. Update consumes
 * one caller-provided frame snapshot; callers must not substitute stale
 * cached vision data for a new frame. Lock returns false until the candidate
 * window satisfies the configured freshness/stability criteria.
 */

typedef enum
{
    Q6_TARGET_EXPLICIT = 0,
    Q6_TARGET_CAPTURE = 1
} Q6TargetSource;

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    uint32_t rx_time_ms;
    uint16_t source_timestamp_ms;
    uint8_t confidence_pct;
    uint8_t sequence;
    bool valid;
} Q6TargetFrame;

typedef struct
{
    Q6TargetSource source;
    float requested_cm;
    float candidate_cm;
    float locked_cm;
    uint32_t independent_frames;
    uint32_t span_ms;
    bool ready;
    bool locked;
    uint32_t rejected_frames;
} Q6TargetStatus;

/* Resets the capture window. explicit_target_cm is used only for EXPLICIT. */
void Q6Target_Init(Q6TargetSource source, float explicit_target_cm);
/* Q6 visual admission policy: accept rows within the configured inclusive
 * pixel band around Q6_BALL_PIXEL_CY_CENTER. Mode selection stays with caller. */
bool Q6Target_IsPixelRowAccepted(uint16_t pixel_cy);
/* Consumes one new frame snapshot; invalid, stale, replayed, or unstable
 * samples cannot advance the lock-ready window. */
void Q6Target_Update(const Q6TargetFrame *sample, uint32_t now_ms);
/* Locks once and writes target_cm on success; false means not ready, already
 * locked, or a null output pointer. */
bool Q6Target_Lock(float *target_cm);
/* Copies the current module snapshot when status is non-null. */
void Q6Target_GetStatus(Q6TargetStatus *status);

#endif
