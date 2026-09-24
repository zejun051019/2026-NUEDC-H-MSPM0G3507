#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

/*
 * Foreground scheduler for the existing periodic diagnostic records.
 * BeginCycle snapshots the current Q6 run state before vision/control work so
 * per-mode periods match the prior main-loop timing. PublishDue runs later in
 * that same loop, after DebugLog_Task, and queues due records only; it never
 * transmits directly. Timestamps are monotonic milliseconds. Never call from
 * an ISR.
 */
void AppTelemetry_BeginCycle(void);
void AppTelemetry_PublishDue(uint32_t now_ms);

#endif
