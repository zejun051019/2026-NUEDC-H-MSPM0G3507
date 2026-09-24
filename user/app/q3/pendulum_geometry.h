#ifndef PENDULUM_GEOMETRY_H
#define PENDULUM_GEOMETRY_H

#include <stdbool.h>

/* Converts the motor-angle calibration into physical beam geometry.
 * Sign is preserved: positive motor angle means the measured right end is
 * above its zero position. */
float PendulumGeometry_MotorDegToEndHeightMm(float motor_angle_deg);
float PendulumGeometry_MotorDegToBeamDeg(float motor_angle_deg);
/* Measured 0..18 deg table only. Negative side assumes odd symmetry. */
bool PendulumGeometry_EndHeightMmToMotorDeg(float height_mm, float *motor_deg);

/* Inverse lookup for future feedforward/model use.  Returns false when the
 * requested beam angle exceeds the measured mechanical safety envelope. */
bool PendulumGeometry_BeamDegToMotorDeg(float beam_angle_deg, float *motor_angle_deg);

#endif
