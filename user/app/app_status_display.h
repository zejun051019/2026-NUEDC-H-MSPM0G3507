#ifndef APP_STATUS_DISPLAY_H
#define APP_STATUS_DISPLAY_H

#include <stdbool.h>

#include "app/q4/q45_vehicle_control.h"

#define APP_STATUS_DISPLAY_LINE_CAPACITY 24U

/*
 * Renders the Q4/Q5/Q6 status body into the already-cleared OLED buffer.
 * Preconditions: vehicle and line_buffer are non-null, and line_buffer points
 * to at least APP_STATUS_DISPLAY_LINE_CAPACITY writable bytes. The caller
 * supplies the latest vehicle snapshot and whether the selected app task is
 * Q6. This foreground-only function formats and draws the status body; it
 * does not clear/refresh the display or query/update control state.
 */
void AppStatusDisplay_ShowVehicle(const Q45VehicleStatus *vehicle, bool q6_task, char *line_buffer);

#endif
