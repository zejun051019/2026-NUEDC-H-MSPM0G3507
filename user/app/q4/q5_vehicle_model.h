#ifndef Q5_VEHICLE_MODEL_H
#define Q5_VEHICLE_MODEL_H

#include <stdbool.h>
#include <stdint.h>

/* Pure caller-owned Q5 vehicle-motion model. Speeds are mm/s, distances cm,
 * accelerations m/s^2, turn blend dimensionless, and timestamps ms. */
typedef struct
{
    uint32_t sampled_ms;
    float speed_mm_s;
    float longitudinal_accel_m_s2;
    float lateral_accel_m_s2;
    float parallel_accel_m_s2;
    float turn_blend;
    bool initialized;
} Q5VehicleModel;

void Q5VehicleModel_Init(Q5VehicleModel *model);
/* Update from one foreground sample. False means invalid inputs or that the
 * configured minimum sampling interval has not elapsed. */
bool Q5VehicleModel_Update(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm);
/* Update with the finalized Q5 route-shaped lateral feedforward. The legacy
 * entry point remains available for modes that intentionally retain its law. */
bool Q5VehicleModel_UpdateFinalQ5(
    Q5VehicleModel *model, uint32_t now_ms, float left_mm_s, float right_mm_s, float distance_cm);
/* Deterministic route profiles; return speed in mm/s and signed blend in [-1,1]. */
float Q5VehicleModel_TraceSpeed(uint32_t elapsed_ms);
float Q5VehicleModel_TurnBlend(float distance_cm, float speed_mm_s);

#endif
