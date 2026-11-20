#include "yummy_imu.h"
#include <stddef.h>
#include <string.h>

#define RX_SIZE 1024U

static struct {
    UART_HandleTypeDef *uart;
    YummyIMU_Status_t status;
    uint8_t bytes[RX_SIZE], frame[96];
    uint32_t at[RX_SIZE];
    volatile uint32_t head, tail, rx_bytes, overflows;
    uint32_t seen_overflows;
    unsigned used;
    bool discard_line;
} imu;

static void receive(UART_HandleTypeDef *uart, const uint8_t *data,
                    uint16_t length, void *context)
{
    (void)uart; (void)context;
    imu.rx_bytes += length;
    if (length > RX_SIZE - (imu.head - imu.tail)) {
        /* Discard the whole broken stream boundary, never splice two frames. */
        imu.tail = imu.head;
        ++imu.overflows;
        return;
    }
    const uint32_t now = HAL_GetTick();
    for (unsigned i = 0U; i < length; ++i) {
        const unsigned index = (imu.head + i) % RX_SIZE;
        imu.bytes[index] = data[i];
        imu.at[index] = now;
    }
    __DMB();
    imu.head += length;
}

static void publish(const float angles[3], uint32_t at)
{
    imu.status.pitch_deg = angles[0];
    imu.status.roll_deg = angles[1];
    imu.status.yaw_deg = angles[2];
    imu.status.received_at_ms = at;
    ++imu.status.sequence;
}

static bool number(const char **cursor, float *result)
{
    const char *p = *cursor;
    while (*p == ' ' || *p == '\t') { ++p; }
    const bool negative = *p == '-';
    if (*p == '-' || *p == '+') { ++p; }
    unsigned digits = 0U;
    float value = 0.0f, scale = 0.1f;
    while (*p >= '0' && *p <= '9') {
        value = value * 10.0f + (float)(*p++ - '0'); ++digits;
    }
    if (*p == '.') {
        ++p;
        while (*p >= '0' && *p <= '9') {
            value += (float)(*p++ - '0') * scale;
            scale *= 0.1f; ++digits;
        }
    }
    while (*p == ' ' || *p == '\t') { ++p; }
    if (digits == 0U || !(value <= 180.0f)) { return false; }
    *result = negative ? -value : value;
    *cursor = p;
    return true;
}

static void text_frame(uint32_t at)
{
    imu.frame[imu.used] = 0U;
    const char *p = (const char *)imu.frame;
    float angles[3];
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!number(&p, &angles[i]) || (i < 2U && *p++ != ',')) {
            ++imu.status.bad_frames; return;
        }
    }
    if (*p != '\0') { ++imu.status.bad_frames; return; }
    publish(angles, at);
}

static void consume(uint8_t byte, uint32_t at)
{
    if (imu.status.format == YUMMY_IMU_USE) {
        if (byte == '\r' || byte == '\n') {
            if (!imu.discard_line && imu.used != 0U) { text_frame(at); }
            imu.used = 0U; imu.discard_line = false;
        } else if (!imu.discard_line) {
            if (imu.used == sizeof(imu.frame) - 1U || byte == 0U) {
                ++imu.status.bad_frames;
                imu.used = 0U; imu.discard_line = true;
            } else { imu.frame[imu.used++] = byte; }
        }
        return;
    }
    if (imu.used == 0U && byte != 0xA5U) { return; }
    imu.frame[imu.used++] = byte;
    if (imu.used != 8U) { return; }
    uint8_t sum = 0U;
    for (unsigned i = 0U; i < 7U; ++i) { sum = (uint8_t)(sum + imu.frame[i]); }
    float angles[3];
    bool valid = sum == imu.frame[7];
    for (unsigned i = 0U; i < 3U; ++i) {
        const unsigned index = 1U + i * 2U;
        const uint16_t bits = (uint16_t)(imu.frame[index] |
            (uint16_t)imu.frame[index + 1U] << 8);
        const int32_t value = bits >= 0x8000U ? (int32_t)bits - 65536 : bits;
        angles[i] = (float)value / 100.0f;
        valid = valid && value >= -18000 && value <= 18000;
    }
    if (valid) { publish(angles, at); imu.used = 0U; }
    else {
        ++imu.status.bad_frames;
        do { memmove(imu.frame, imu.frame + 1U, --imu.used); }
        while (imu.used != 0U && imu.frame[0] != 0xA5U);
    }
}

HAL_StatusTypeDef YummyIMU_Init(UART_HandleTypeDef *uart, YummyIMU_Format_t format)
{
    if (uart == NULL || __get_IPSR() != 0U ||
        (format != YUMMY_IMU_USE && format != YUMMY_IMU_BINARY)) { return HAL_ERROR; }
    if (imu.uart != NULL) { return HAL_BUSY; }
    imu.uart = uart;
    imu.status.format = format;
    return BSP_UART_Init(uart, receive, NULL);
}

void YummyIMU_Run(void)
{
    if (imu.uart == NULL || __get_IPSR() != 0U) { return; }
    /* At most 256 bytes/tick; parsing/floating point never runs in the ISR. */
    for (unsigned budget = 0U; budget < 256U; ++budget) {
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (imu.seen_overflows != imu.overflows) {
            imu.seen_overflows = imu.overflows;
            imu.used = 0U;
            imu.discard_line = imu.status.format == YUMMY_IMU_USE;
        }
        if (imu.tail == imu.head) { __set_PRIMASK(mask); break; }
        const unsigned index = imu.tail % RX_SIZE;
        const uint8_t byte = imu.bytes[index];
        const uint32_t at = imu.at[index];
        ++imu.tail;
        __set_PRIMASK(mask);
        consume(byte, at);
    }
}

void YummyIMU_GetStatus(YummyIMU_Status_t *status)
{
    if (status == NULL || __get_IPSR() != 0U) { return; }
    *status = imu.status;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    status->rx_bytes = imu.rx_bytes;
    status->overflows = imu.overflows;
    __set_PRIMASK(mask);
    BSP_UART_Status_t uart = {0};
    /* Historical errors stay visible, but do not mean RX has stopped.
     * Frame validity and age are checked separately by the consumer. */
    status->rx_running = imu.uart != NULL &&
        BSP_UART_GetStatus(imu.uart, &uart) == HAL_OK && uart.rx_running &&
        uart.rx_rearm == HAL_OK;
    status->uart_error = uart.rx_error;
}
