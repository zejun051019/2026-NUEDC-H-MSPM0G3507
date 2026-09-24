#ifndef BALL_STATE_ESTIMATOR_H
#define BALL_STATE_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    float innovation_cm;
    uint8_t rejected_count;
    int8_t rejected_direction;
    bool initialized;
} BallStateEstimator;

void BallStateEstimator_Init(BallStateEstimator *estimator);

/* Alpha-beta observer.  A single implausible position jump is ignored.  Three
 * consecutive jumps in the same direction are treated as real motion and
 * re-acquire the measurement, so the gate cannot permanently lose the ball. */
bool BallStateEstimator_Update(BallStateEstimator *estimator,
                               float measured_position_cm,
                               float dt_s);

#endif
