#include "x42s.h"

#include <string.h>

#include "ti_msp_dl_config.h"

/* This project is wired and configured for the supplied X firmware.  The
 * F1/FC quick-position pair is documented by the vendor, but FC carries no
 * explicit reference-mode byte.  A real actuator run showed an FC target of
 * about -6.5 deg being interpreted as a continuing move to about -53 deg.
 * Balance control therefore uses the complete CD trapezoid packet for every
 * target: direction, magnitude and absolute-mode are present in each frame.
 * Emm and X firmware use incompatible quick-position units; never mix them. */
#define X42S_CHECK_BYTE 0x6BU
#define X42S_MAX_SPEED_RPM 3000.0f
#define X42S_MAX_CURRENT_MA 6000U

static uint8_t g_motor_address = 1U;
static X42S_State g_motor_state;
static uint8_t g_rx_frame[32];
static uint8_t g_rx_count;
static uint16_t g_quick_accel;
static uint16_t g_quick_decel;
static float g_quick_speed;
static X42S_PositionMode g_quick_mode;
static uint16_t g_quick_current;

static bool X42S_UsesEmmFirmware(void)
{
    return ((g_motor_state.valid_mask & X42S_VALID_OPTIONS) != 0U) &&
           ((g_motor_state.option_flags & X42S_OPTION_FIRMWARE_EMM) != 0U);
}

static void X42S_PutU16BE(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}

static void X42S_PutU32BE(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value >> 24);
    dst[1] = (uint8_t)(value >> 16);
    dst[2] = (uint8_t)(value >> 8);
    dst[3] = (uint8_t)value;
}

static bool X42S_Send(const uint8_t *data, uint8_t length)
{
    uint8_t index;
    if ((data == NULL) || (length == 0U))
        return false;
    for (index = 0U; index < length; ++index)
    {
        /* PRINT_INST is UART0, configured by SysConfig for PA0/PA1. */
        DL_UART_Main_transmitDataBlocking(PRINT_INST, data[index]);
    }
    return true;
}

static bool X42S_ValidateMotion(float position_deg, float speed_rpm, uint16_t current_ma)
{
    float abs_position = (position_deg < 0.0f) ? -position_deg : position_deg;
    float abs_speed = (speed_rpm < 0.0f) ? -speed_rpm : speed_rpm;
    return (abs_position <= 429496729.0f) && (abs_speed <= X42S_MAX_SPEED_RPM) &&
           (current_ma <= X42S_MAX_CURRENT_MA);
}

static uint32_t X42S_DegreesToTenths(float degrees)
{
    float magnitude = (degrees < 0.0f) ? -degrees : degrees;
    return (uint32_t)(magnitude * 10.0f + 0.5f);
}

static uint16_t X42S_RpmToTenths(float rpm)
{
    float magnitude = (rpm < 0.0f) ? -rpm : rpm;
    return (uint16_t)(magnitude * 10.0f + 0.5f);
}

static void X42S_ParseFrame(const uint8_t *frame, uint8_t length)
{
    uint32_t magnitude;
    if ((length < 4U) || (frame[0] != g_motor_address) || (frame[length - 1U] != X42S_CHECK_BYTE))
    {
        ++g_motor_state.frame_errors;
        return;
    }

    g_motor_state.last_function = frame[1];
    ++g_motor_state.frames_received;
    if ((frame[1] == 0x1AU) && (length == 5U))
    {
        /* Recent X42S firmware returns the 10 option bits as two bytes.
         * Keep the complete word: the documented firmware bit is bit1. */
        g_motor_state.option_flags = ((uint16_t)frame[2] << 8) | frame[3];
        g_motor_state.valid_mask |= X42S_VALID_OPTIONS;
        ++g_motor_state.option_frames_received;
        return;
    }
    if (length == 4U)
    {
        g_motor_state.last_reply = frame[2];
        if (frame[1] == 0x1AU)
        {
            /* 0x1A is common to X and Emm: bit1 selects Emm firmware. */
            g_motor_state.option_flags = frame[2];
            g_motor_state.valid_mask |= X42S_VALID_OPTIONS;
            ++g_motor_state.option_frames_received;
        }
        if (frame[1] == (uint8_t)X42S_FEEDBACK_STATUS)
        {
            g_motor_state.status_flags = frame[2];
            g_motor_state.valid_mask |= X42S_VALID_STATUS;
        }
        return;
    }

    /* The supplied "read actual position" example defines this 8-byte reply. */
    if ((frame[1] == (uint8_t)X42S_FEEDBACK_POSITION) && (length == 8U))
    {
        magnitude = ((uint32_t)frame[3] << 24) | ((uint32_t)frame[4] << 16) |
                    ((uint32_t)frame[5] << 8) | frame[6];
        /* The two firmware families use different position units. */
        g_motor_state.position_deg = X42S_UsesEmmFirmware() ? ((float)magnitude * 360.0f / 65536.0f)
                                                            : ((float)magnitude / 10.0f);
        if (frame[2] != 0U)
            g_motor_state.position_deg = -g_motor_state.position_deg;
        g_motor_state.valid_mask |= X42S_VALID_POSITION;
        ++g_motor_state.position_frames_received;
    }
}

void X42S_Init(uint8_t address)
{
    g_motor_address = (address == 0U) ? 1U : address;
    g_quick_accel = 400U;
    g_quick_decel = 700U;
    g_quick_speed = 6.0f;
    g_quick_mode = X42S_POSITION_ABSOLUTE;
    g_quick_current = 1200U;
    X42S_ClearState();
    /* Pinmux is owned exclusively by empty.syscfg: UART0 TX=PA0, RX=PA1.
     * Do not remap PA10/PA11 here; they are grey-sensor inputs in this car. */
    NVIC_ClearPendingIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
}

void X42S_ClearState(void)
{
    memset(&g_motor_state, 0, sizeof(g_motor_state));
    g_rx_count = 0U;
}

const X42S_State *X42S_GetState(void)
{
    return &g_motor_state;
}

void X42S_RxIRQ(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(PRINT_INST))
    {
        uint8_t byte = DL_UART_Main_receiveData(PRINT_INST);
        uint8_t expected;
        ++g_motor_state.raw_bytes_received;
        if (g_motor_state.first_rx_count < sizeof(g_motor_state.first_rx_bytes))
        {
            g_motor_state.first_rx_bytes[g_motor_state.first_rx_count++] = byte;
        }
        if (g_rx_count >= sizeof(g_rx_frame))
        {
            ++g_motor_state.rx_overflows;
            g_rx_count = 0U;
        }
        if (g_rx_count == 0U && byte != g_motor_address)
            continue;
        g_rx_frame[g_rx_count++] = byte;
        if (g_rx_count < 2U)
            continue;
        expected = (g_rx_frame[1] == (uint8_t)X42S_FEEDBACK_POSITION)
                       ? 8U
                       : ((g_rx_frame[1] == 0x1AU) ? 5U : 4U);
        /* X position payload may itself contain 0x6B (e.g. +10.7 deg).
         * A terminator-only parser truncated those perfectly valid replies.
         * This application polls only X option/position plus 4-byte ACKs. */
        if (g_rx_count >= expected)
        {
            X42S_ParseFrame(g_rx_frame, g_rx_count);
            g_rx_count = 0U;
        }
    }
}

void X42S_FlushRx(void)
{
    NVIC_DisableIRQ(PRINT_INST_INT_IRQN);
    while (!DL_UART_Main_isRXFIFOEmpty(PRINT_INST))
        (void)DL_UART_Main_receiveData(PRINT_INST);
    g_rx_count = 0U;
    NVIC_ClearPendingIRQ(PRINT_INST_INT_IRQN);
    NVIC_EnableIRQ(PRINT_INST_INT_IRQN);
}

void X42S_Process(void) {}

bool X42S_Enable(bool enable)
{
    uint8_t command[] = {g_motor_address, 0xF3U, 0xABU, enable ? 1U : 0U, 0U, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_Stop(void)
{
    uint8_t command[] = {g_motor_address, 0xFEU, 0x98U, 0U, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_ClearProtection(void)
{
    uint8_t command[] = {g_motor_address, 0x0EU, 0x52U, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetMicrostep256(void)
{
    /* 0x84 is Emm-only.  Do not send it to an identified X motor. */
    if (!X42S_UsesEmmFirmware())
        return true;
    {
        uint8_t command[] = {g_motor_address, 0x84U, 0x8AU, 0U, 0U, X42S_CHECK_BYTE};
        return X42S_Send(command, sizeof(command));
    }
}

bool X42S_ZeroCurrentPosition(void)
{
    uint8_t command[] = {g_motor_address, 0x0AU, 0x6DU, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_ReturnAbsoluteZero(void)
{
    /* 9A mode 04: return to the motor's stored absolute coordinate zero. */
    uint8_t command[] = {g_motor_address, 0x9AU, 0x04U, 0x00U, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetTorqueLimitedSpeed(float current_ma, uint16_t current_ramp_ma_s, float max_speed_rpm)
{
    uint8_t command[11] = {g_motor_address, 0xC5U};
    uint16_t current;
    if (!X42S_ValidateMotion(
            0.0f, max_speed_rpm, (uint16_t)((current_ma < 0.0f) ? -current_ma : current_ma)))
        return false;
    current = (uint16_t)((current_ma < 0.0f) ? -current_ma : current_ma);
    command[2] = (current_ma < 0.0f) ? 1U : 0U;
    X42S_PutU16BE(&command[3], current_ramp_ma_s);
    X42S_PutU16BE(&command[5], current);
    X42S_PutU16BE(&command[8], X42S_RpmToTenths(max_speed_rpm));
    command[7] = 0U;
    command[10] = X42S_CHECK_BYTE;
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetVelocityLimitedCurrent(float speed_rpm,
                                    uint16_t acceleration_rpm_s,
                                    uint16_t max_current_ma)
{
    uint8_t command[11] = {g_motor_address, 0xC6U};
    if (!X42S_ValidateMotion(0.0f, speed_rpm, max_current_ma))
        return false;
    command[2] = (speed_rpm < 0.0f) ? 1U : 0U;
    X42S_PutU16BE(&command[3], acceleration_rpm_s);
    X42S_PutU16BE(&command[5], X42S_RpmToTenths(speed_rpm));
    command[7] = 0U;
    X42S_PutU16BE(&command[8], max_current_ma);
    command[10] = X42S_CHECK_BYTE;
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetEmmVelocity(float speed_rpm, uint8_t acceleration_grade)
{
    uint16_t rpm;
    uint8_t command[8];

    if (!X42S_UsesEmmFirmware())
        return false;
    if ((speed_rpm > 3000.0f) || (speed_rpm < -3000.0f))
        return false;
    rpm = (uint16_t)((speed_rpm < 0.0f) ? -speed_rpm : speed_rpm);
    command[0] = g_motor_address;
    command[1] = 0xF6U;
    command[2] = (speed_rpm < 0.0f) ? 1U : 0U;
    X42S_PutU16BE(&command[3], rpm);
    command[5] = acceleration_grade;
    command[6] = 0U;
    command[7] = X42S_CHECK_BYTE;
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetBypassPosition(float position_deg,
                            float max_speed_rpm,
                            X42S_PositionMode mode,
                            uint16_t max_current_ma)
{
    uint8_t command[14] = {g_motor_address, 0xCBU};
    if ((mode > X42S_POSITION_RELATIVE_CURRENT) ||
        !X42S_ValidateMotion(position_deg, max_speed_rpm, max_current_ma))
        return false;
    command[2] = (position_deg < 0.0f) ? 1U : 0U;
    X42S_PutU16BE(&command[3], X42S_RpmToTenths(max_speed_rpm));
    X42S_PutU32BE(&command[5], X42S_DegreesToTenths(position_deg));
    command[9] = (uint8_t)mode;
    command[10] = 0U;
    X42S_PutU16BE(&command[11], max_current_ma);
    command[13] = X42S_CHECK_BYTE;
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetTrapezoidPosition(float position_deg,
                               uint16_t acceleration_rpm_s,
                               uint16_t deceleration_rpm_s,
                               float max_speed_rpm,
                               X42S_PositionMode mode,
                               uint16_t max_current_ma)
{
    uint8_t command[18] = {g_motor_address, 0xCDU};
    if ((mode > X42S_POSITION_RELATIVE_CURRENT) ||
        !X42S_ValidateMotion(position_deg, max_speed_rpm, max_current_ma))
        return false;
    command[2] = (position_deg < 0.0f) ? 1U : 0U;
    X42S_PutU16BE(&command[3], acceleration_rpm_s);
    X42S_PutU16BE(&command[5], deceleration_rpm_s);
    X42S_PutU16BE(&command[7], X42S_RpmToTenths(max_speed_rpm));
    X42S_PutU32BE(&command[9], X42S_DegreesToTenths(position_deg));
    command[13] = (uint8_t)mode;
    command[14] = 0U;
    X42S_PutU16BE(&command[15], max_current_ma);
    command[17] = X42S_CHECK_BYTE;
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetQuickPositionParams(uint16_t acceleration_rpm_s,
                                 uint16_t deceleration_rpm_s,
                                 float max_speed_rpm,
                                 X42S_PositionMode mode,
                                 uint16_t max_current_ma)
{
    if ((mode > X42S_POSITION_RELATIVE_CURRENT) ||
        !X42S_ValidateMotion(0.0f, max_speed_rpm, max_current_ma))
        return false;
    g_quick_accel = acceleration_rpm_s;
    g_quick_decel = deceleration_rpm_s;
    g_quick_speed = max_speed_rpm;
    g_quick_mode = mode;
    g_quick_current = max_current_ma;
    if (X42S_UsesEmmFirmware())
    {
        /* Emm F1: RPM, acceleration grade, reference mode, sync. */
        uint8_t command[8] = {g_motor_address, 0xF1U};
        X42S_PutU16BE(&command[2], (uint16_t)(max_speed_rpm + 0.5f));
        command[4] = (acceleration_rpm_s > 255U) ? 255U : (uint8_t)acceleration_rpm_s;
        command[5] = (uint8_t)mode;
        command[6] = 0U;
        command[7] = X42S_CHECK_BYTE;
        return X42S_Send(command, sizeof(command));
    }
    else
    {
        /* X F1 has separate acceleration/deceleration, 0.1-RPM speed,
         * reference mode and current limit. */
        uint8_t command[13] = {g_motor_address, 0xF1U};
        X42S_PutU16BE(&command[2], acceleration_rpm_s);
        X42S_PutU16BE(&command[4], deceleration_rpm_s);
        X42S_PutU16BE(&command[6], X42S_RpmToTenths(max_speed_rpm));
        command[8] = (uint8_t)mode;
        command[9] = 0U;
        X42S_PutU16BE(&command[10], max_current_ma);
        command[12] = X42S_CHECK_BYTE;
        return X42S_Send(command, sizeof(command));
    }
}

bool X42S_SetQuickPosition(float position_deg)
{
    /*
     * Keep this public wrapper so existing application code remains simple,
     * but do not emit FC here.  CD is 11 bytes longer, yet at the 40 Hz beam
     * update rate it takes only about 1.6 ms on the 115200-baud bus.  Its
     * explicit mode byte makes every command an unambiguous absolute target.
     */
    if ((g_quick_mode > X42S_POSITION_RELATIVE_CURRENT) ||
        !X42S_ValidateMotion(position_deg, g_quick_speed, g_quick_current))
        return false;
    return X42S_SetTrapezoidPosition(
        position_deg, g_quick_accel, g_quick_decel, g_quick_speed, g_quick_mode, g_quick_current);
}

bool X42S_RequestFeedback(X42S_Feedback feedback)
{
    uint8_t command[] = {g_motor_address, (uint8_t)feedback, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_RequestOptions(void)
{
    /* This command is common to both firmware families and must be the
     * first protocol transaction. */
    uint8_t command[] = {g_motor_address, 0x1AU, X42S_CHECK_BYTE};
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetTimedFeedback(X42S_Feedback feedback, uint16_t period_ms)
{
    uint8_t command[] = {g_motor_address, 0x11U, 0x18U, (uint8_t)feedback, 0U, 0U, X42S_CHECK_BYTE};
    X42S_PutU16BE(&command[4], period_ms);
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetHeartbeat(uint32_t timeout_ms, bool save)
{
    uint8_t command[] = {
        g_motor_address, 0x68U, 0x38U, save ? 1U : 0U, 0U, 0U, 0U, 0U, X42S_CHECK_BYTE};
    X42S_PutU32BE(&command[4], timeout_ms);
    return X42S_Send(command, sizeof(command));
}

bool X42S_SetClosedLoop(bool closed_loop, bool save)
{
    (void)closed_loop;
    (void)save;
    return false;
}
