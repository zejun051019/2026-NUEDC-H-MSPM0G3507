#ifndef APP_CLOCK_H
#define APP_CLOCK_H

#include <stdint.h>

/* Millisecond timebase advanced by the existing TIMG6 10 ms ISR. Read from
 * foreground code; unsigned elapsed-time subtraction supports counter wrap. */
void AppClock_Tick10msISR(void);
uint32_t AppClock_NowMs(void);

#endif
