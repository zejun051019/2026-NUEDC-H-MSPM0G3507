#ifndef __GYRO_H__
#define __GYRO_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* UART2 inertial-sensor interface. The receive parser runs from the UART ISR;
 * Yaw() returns degrees and GyroZ() returns degrees/s. Calibration transmit
 * helpers busy-wait/delay and must be called only from foreground code.
 */
struct SAngle
{
    float Yaw;
};
extern volatile struct SAngle stcAngle;

struct SGyro
{
    float wz;
};
extern volatile struct SGyro stcGyro;

void CopeSerial2Data(unsigned char ucData);
/* Read the most recently parsed sensor sample; zero means no sample received
 * since initialization. */
float GyroZ(void);
float Yaw(void);
/* Sends blocking calibration sequences; performCaliBias() blocks for ~21 s. */
void sendCaliYawCommand(void);
void performCaliBias(void);
void gyro_send_cmd(uint8_t *data, uint32_t len);

#endif
