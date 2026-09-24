#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdint.h>

/*
 * Fixed-buffer debug telemetry serializer and UART1 TX queue. Publish calls
 * snapshot module state, format the existing wire records, and enqueue them;
 * a full queue drops the new record. DebugLog_Task drains a bounded number of
 * bytes per foreground call without waiting for UART space. All APIs are
 * foreground-only: formatting, state snapshots, and queue mutation are not
 * ISR-safe. Timestamps are milliseconds; payload units and field order are
 * part of the existing log protocol and must remain stable.
 */

void DebugLog_Init(void);
void DebugLog_PublishQ6Event(uint32_t now_ms, uint8_t code, int32_t value);
void DebugLog_PublishVision(uint32_t now_ms);
void DebugLog_PublishActuator(uint32_t now_ms);
void DebugLog_PublishPendulumCalibration(uint32_t now_ms);
void DebugLog_PublishBallPositionControl(uint32_t now_ms);
void DebugLog_PublishBallMechanismId(uint32_t now_ms);
void DebugLog_PublishPendulumGeometryCalibration(uint32_t now_ms);
void DebugLog_PublishBallPulseId(uint32_t now_ms);
void DebugLog_PublishQ3Sequence(uint32_t now_ms);
void DebugLog_PublishQ45Vehicle(uint32_t now_ms);
void DebugLog_PublishControlFrame(uint32_t now_ms);
void DebugLog_Task(void);

#endif
