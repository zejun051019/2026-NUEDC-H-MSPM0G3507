#ifndef _KEY_H_
#define _KEY_H_

#include "headfile.h"

/* Legacy blocking key scanner. It delays and waits for key release, so it is
 * not suitable for the current nonblocking main-loop scheduler or an ISR. */
uint8_t Key_Scan(void);

#endif /* _KEY_H_ */
