#ifndef LICHENG_H
#define LICHENG_H

#include "headfile.h"

/* Incremental chassis odometry accumulator. Update inputs are signed wheel
 * pulse deltas for one speed-sampling interval; distances are accumulated in
 * cm. Encoder_SpeedCalc() owns periodic updates in the current firmware.
 */

/* 左轮、右轮和车体中心累计行驶距离，单位：cm。 */
typedef struct
{
    float left_cm;
    float right_cm;
    float center_cm;
} LichengData_t;

extern LichengData_t g_licheng;

void Licheng_Init(void);
void Licheng_Reset(void);
void Licheng_Update(int32_t left_pulse, int32_t right_pulse);

#endif
