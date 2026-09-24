#ifndef UART_H
#define UART_H

#include "ti_msp_dl_config.h"

/* Generic blocking UART TX helpers. Use only from foreground code; they wait
 * for each byte and are not safe for ISR or control-timer use. RX dispatch is
 * owned by the generated per-UART interrupt handlers and protocol modules.
 */
void uart_send_char(UART_Regs *uart, const uint8_t chr);
void uart_send_string(UART_Regs *uart, const char *str);

#endif
