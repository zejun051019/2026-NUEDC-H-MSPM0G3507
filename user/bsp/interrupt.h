#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "headfile.h"

/* Board interrupt glue. ISRs publish key events through the volatile bytes
 * below; foreground code owns event consumption and application actions.
 * Keep interrupt handlers bounded and free of blocking serial/OLED work.
 */
extern uint8_t enable_group1_irq;
extern volatile uint8_t g_key_t1;
extern volatile uint8_t g_key_t2;
extern volatile uint8_t g_key_t3;
extern volatile uint8_t g_key_t4;
extern volatile uint8_t g_key_start;
void Interrupt_Init(void);

#endif
