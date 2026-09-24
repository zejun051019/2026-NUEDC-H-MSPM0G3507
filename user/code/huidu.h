#ifndef HUIDU_H
#define HUIDU_H

#include "headfile.h"

/* Eight-channel active-low line sensor adapter. A set bit means the
 * corresponding sensor sees the dark line; geometric error is in the
 * sensor's configured relative units, not a physical distance. */

/* 黑线循迹：检测到黑线时置 1，bit0~bit7 对应 S1~S8。 */
uint8_t Huidu_Read(void);

#define HUIDU_S1(v) (((v) >> 0) & 0x01U)
#define HUIDU_S2(v) (((v) >> 1) & 0x01U)
#define HUIDU_S3(v) (((v) >> 2) & 0x01U)
#define HUIDU_S4(v) (((v) >> 3) & 0x01U)
#define HUIDU_S5(v) (((v) >> 4) & 0x01U)
#define HUIDU_S6(v) (((v) >> 5) & 0x01U)
#define HUIDU_S7(v) (((v) >> 6) & 0x01U)
#define HUIDU_S8(v) (((v) >> 7) & 0x01U)

typedef enum
{
    HUIDU_TRACKING = 0,
    HUIDU_ALL_WHITE = 1,
    HUIDU_ALL_BLACK = 2
} HuiduState_t;

float Huidu_GetError(uint8_t sensor_data);
/* S4/S5 form the geometric centre of the eight-channel array. */
uint8_t Huidu_IsCentered(uint8_t sensor_data);
HuiduState_t Huidu_GetState(uint8_t sensor_data);

#endif
