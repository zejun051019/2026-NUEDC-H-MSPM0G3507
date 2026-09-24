#ifndef APP_KEY_INPUT_H
#define APP_KEY_INPUT_H

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

/*
 * Foreground helper for consuming GPIO key events recorded by the ISR.
 * `flag` is shared with interrupt context and must point to the existing
 * volatile event byte. `last_press_ms` is caller-owned per-key debounce state.
 * The function clears the pending GPIO interrupt status and applies the
 * existing 15 ms debounce interval; it does not choose an application action.
 * Call only from the foreground loop, never from an ISR.
 */
bool AppKeyInput_ConsumePress(GPIO_Regs *port,
                              uint32_t pin,
                              volatile uint8_t *flag,
                              uint32_t now_ms,
                              uint32_t *last_press_ms);

#endif
