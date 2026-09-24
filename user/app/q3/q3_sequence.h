#ifndef Q3_SEQUENCE_H
#define Q3_SEQUENCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    Q3_SEQUENCE_IDLE = 0,
    Q3_SEQUENCE_TO_PLUS5,
    Q3_SEQUENCE_BRAKE_PLUS5,
    Q3_SEQUENCE_TO_MINUS2_5,
    Q3_SEQUENCE_TO_MINUS5,
    Q3_SEQUENCE_RETURN_LEVEL,
    Q3_SEQUENCE_FINAL_HOLD,
    Q3_SEQUENCE_WAIT_CENTER,
    Q3_SEQUENCE_FAULT,
    Q3_SEQUENCE_STOPPED,
    Q3_SEQUENCE_CENTER,
    Q3_SEQUENCE_SINGLE_TARGET,
    Q3_SEQUENCE_RETURN_CRUISE,
    Q3_SEQUENCE_RETURN_BRAKE,
    /* Ball reached -5, X42S was confirmed back at the PA31 origin, and the
     * position session was explicitly paused. */
    Q3_SEQUENCE_FINISHED
} Q3SequenceState;

typedef struct
{
    Q3SequenceState state;
    uint32_t state_started_ms;
    uint32_t run_sequence;
    bool fault;
    uint32_t motion_started_ms;
    uint32_t completion_ms;
    bool time_pass;
    /* True after the first PA31 zero-mark.  This is deliberately exposed in
     * telemetry so we can prove that later rounds do not redefine zero. */
    bool zero_reference_ready;
} Q3SequenceStatus;

void Q3Sequence_Init(void);
bool Q3Sequence_Start(uint32_t now_ms);
/* First PA31 press: mark the motor's current physical pose as angle zero. */
bool Q3Sequence_ZeroReference(uint32_t now_ms);
bool Q3Sequence_IsZeroReferenceReady(void);
void Q3Sequence_EnableAutoCycle(bool enabled);
void Q3Sequence_Stop(uint32_t now_ms);
void Q3Sequence_Task(uint32_t now_ms);
bool Q3Sequence_IsActive(void);
bool Q3Sequence_ConsumeResultLog(void);
void Q3Sequence_GetStatus(Q3SequenceStatus *status);
const char *Q3Sequence_StateName(Q3SequenceState state);

#endif
