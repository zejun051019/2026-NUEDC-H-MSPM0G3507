#ifndef __OLED_H
#define __OLED_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

/*
 * 128 x 64 OLED drawing API. Primitive/text drawing updates the framebuffer;
 * call OLED_Refresh() to transfer it. OLED_Clear() clears and refreshes.
 * The current transport is blocking SysConfig I2C and uses a bounded wait;
 * display functions are foreground-only and must not run from an ISR.
 * Coordinates are pixels unless an API explicitly takes page indices.
 */

#define OLED_CMD 0
#define OLED_DATA 1

/* Legacy public aliases retained for existing call sites and font tables. */
typedef unsigned char u8;
typedef unsigned int u32;

void OLED_ClearPoint(u8 x, u8 y);
void OLED_ColorTurn(u8 i);   /* 0: normal, 1: inverse. */
void OLED_DisplayTurn(u8 i); /* 0: normal orientation, 1: rotate 180 degrees. */

/* Legacy software-I2C declarations retained for source compatibility. They
 * have no implementation in the current hardware-I2C OLED module. */
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(u8 dat);

/* Send one command/data byte over the current I2C transport. mode is
 * OLED_CMD or OLED_DATA. The bounded transport wait has no status return. */
void OLED_WR_Byte(u8 dat, u8 mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);

/* Framebuffer drawing primitives; caller must keep coordinates in range. */
void OLED_DrawPoint(u8 x, u8 y);
void OLED_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2);
void OLED_DrawCircle(u8 x, u8 y, u8 r);
void OLED_ShowChar(u8 x, u8 y, u8 chr, u8 size1);
void OLED_ShowString(u8 x, u8 y, u8 *chr, u8 size1);
void OLED_ShowNum(u8 x, u8 y, u32 num, u8 len, u8 size1);
void OLED_ShowChinese(u8 x, u8 y, u8 num, u8 size1);

/* Legacy declaration retained for source compatibility; no implementation
 * exists in the current OLED module. */
void OLED_ScrollDisplay(u8 num, u8 space);

/* Page-addressed direct transfer: x is a column, y is a page (8 pixel rows).
 * OLED_ShowPicture writes the rectangle [x0,x1) x [y0,y1) directly to the
 * device, using page-major bytes from BMP; it does not update the framebuffer.
 */
void OLED_WR_BP(u8 x, u8 y);
void OLED_ShowPicture(u8 x0, u8 y0, u8 x1, u8 y1, u8 BMP[]);

/* Initialize the display and clear/refresh its framebuffer. */
void OLED_Init(void);

#endif
