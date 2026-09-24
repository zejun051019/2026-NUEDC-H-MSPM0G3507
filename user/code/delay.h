#ifndef DELAY_H
#define DELAY_H

#include "ti_msp_dl_config.h"

/* Busy-wait delay using the configured CPU clock. Foreground-only: it blocks
 * its caller and must not be used from an ISR or timing-critical loop. */
void delay_ms(uint32_t ms);

#endif // DELAY_H
