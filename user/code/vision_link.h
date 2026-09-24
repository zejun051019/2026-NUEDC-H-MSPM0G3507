#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * UART3 vision framing and latest-sample interface. RxIRQ is called from the
 * UART ISR and performs bounded byte/frame bookkeeping only; Poll, SetNow,
 * GetLatest and GetStats are main-loop APIs. Sample position/velocity use
 * cm and cm/s, source and receive times use ms, and confidence is percent.
 * GetLatest returns true only when an accepted sample is available; consumers
 * must still apply their own age/sequence policy before using it for control.
 */

/* MaixCam H 题视觉协议：AA 55 01 type seq len payload crc16-modbus。 */
#define VISION_LINK_VERSION 0x01U
#define VISION_LINK_TYPE_HEARTBEAT 0x01U
#define VISION_LINK_TYPE_DETECT 0x02U
#define VISION_LINK_TYPE_STATUS 0x03U
#define VISION_LINK_DETECT_PAYLOAD_LEN 12U
#define VISION_LINK_MAX_PAYLOAD_LEN 16U

typedef struct
{
    float position_cm;
    float velocity_cm_s;
    uint16_t source_timestamp_ms;
    uint8_t stable;
    uint8_t confidence_pct;
    uint16_t pixel_cx;
    uint16_t pixel_cy;
    uint8_t sequence;
    uint32_t rx_time_ms;
    bool valid;
} VisionLinkSample;

typedef struct
{
    uint32_t bytes_received;
    uint32_t irq_count;
    uint32_t poll_bytes;
    uint32_t frames_ok;
    uint32_t detect_frames;
    uint32_t heartbeat_frames;
    uint32_t status_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
    uint32_t type_errors;
    uint32_t sequence_lost;
    uint8_t last_sequence;
    bool have_sequence;
} VisionLinkStats;

void VisionLink_Init(void);
void VisionLink_SetNow(uint32_t now_ms);
void VisionLink_RxIRQ(void);
/* Main-loop fallback.  It drains UART3 FIFO if an RX interrupt was missed. */
void VisionLink_Poll(void);
bool VisionLink_GetLatest(VisionLinkSample *out);
void VisionLink_GetStats(VisionLinkStats *out);

#endif
