#include "motor_usb.h"
#include "bsp_usb.h"
#include "motor_tuning.h"
#include "motor_telemetry.h"
#include <string.h>

static struct {
    bool initialized, session_valid;
    uint8_t frame[31], used, session[8], last_request[24], last_response[20];
    uint8_t response[27], response_length;
    uint32_t epoch, last_byte_ms, last_id;
} link;

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 |
        (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}

static bool allowed(const uint8_t *rpc)
{
    const uint8_t op = rpc[1], scope = rpc[2], field = rpc[3];
    return (op == 0U && scope == 0U && field == 0U) ||
        (op == 6U && scope == 0U && field <= 1U) ||
        (op == 7U && scope == 0U && (field == 2U || field == 3U)) ||
        (op == 12U && ((scope == 0U &&
            (field == 0U || (field >= 5U && field <= 8U) || field == 255U)) ||
            ((scope == 1U || scope == 2U) && field >= 11U && field <= 14U)));
}

static void request(uint32_t now_ms)
{
    const uint8_t *p = &link.frame[5];
    uint8_t type = 0x92U, length = 8U;
    if (link.frame[3] == 0x12U) {
        if (!link.session_valid || memcmp(link.session, p, 8U) != 0) {
            memcpy(link.session, p, 8U);
            link.last_id = 0U;
            link.session_valid = true;
        }
        memcpy(&link.response[5], p, 8U);
    } else {
        type = 0x91U; length = 20U;
        uint8_t *response = &link.response[5], *reply = &response[12];
        const uint8_t *rpc = &p[16];
        const uint32_t id = get32(&p[8]);
        memcpy(response, p, 12U);
        memset(reply, 0, 8U);
        reply[0] = rpc[0]; reply[1] = BSP_CAN_NOT_READY;
        reply[2] = rpc[2]; reply[3] = rpc[3];
        if (link.session_valid && memcmp(p, link.session, 8U) == 0 && id != 0U) {
            if (id == link.last_id) {
                if (memcmp(p, link.last_request, 24U) == 0) {
                    memcpy(response, link.last_response, 20U);
                } else { reply[1] = BSP_CAN_INVALID_ARG; }
            } else if (link.last_id != UINT32_MAX && id == link.last_id + 1U) {
                if (!allowed(rpc)) { reply[1] = BSP_CAN_INVALID_ARG; }
                else if (MotorTuning_HandleRequest(rpc, get32(&p[12]), now_ms, reply) != BSP_CAN_OK) {
                    reply[1] = BSP_CAN_ERROR;
                }
                /* Cache even rejected requests before TX; retry cannot execute
                 * a command twice, or change an expired intent's timestamp. */
                memcpy(link.last_request, p, 24U);
                memcpy(link.last_response, response, 20U);
                link.last_id = id;
            }
        }
    }
    uint8_t *out = link.response;
    out[0] = 0xA5U; out[1] = 0x5AU; out[2] = 1U;
    out[3] = type; out[4] = length;
    const uint16_t crc = MotorTelemetry_CRC16(&out[2], (unsigned)length + 3U);
    out[5U + length] = (uint8_t)crc;
    out[6U + length] = (uint8_t)(crc >> 8);
    link.response_length = (uint8_t)(length + 7U);
}

void MotorUSB_Init(void) { memset(&link, 0, sizeof(link)); link.initialized = true; }

bool MotorUSB_Run(void)
{
    if (!link.initialized || __get_IPSR() != 0U) { return true; }
    /* Single consumer; ISR only enqueues bytes. Work bounded to 64 bytes/tick.
     * Check the RX epoch before sending a cached reply too. */
    for (unsigned budget = 0U; budget < 64U; ++budget) {
        uint8_t byte;
        uint32_t at, epoch;
        /* A pending reply must not consume the following command. */
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (BSP_USB_RxEpoch() != link.epoch) {
            link.epoch = BSP_USB_RxEpoch();
            link.used = 0U; link.session_valid = false; link.response_length = 0U;
        }
        if (link.response_length != 0U) {
            BSP_USB_Diagnostics_t diag = {0};
            (void)BSP_USB_GetDiagnostics(&diag);
            if (!diag.dtr || !diag.configured) {
                link.response_length = 0U; link.session_valid = false; link.used = 0U;
            } else {
                const BSP_USB_Status_t sent = BSP_USB_Send(link.response, link.response_length);
                if (sent == BSP_USB_OK) { link.response_length = 0U; }
                __set_PRIMASK(mask);
                return false;
            }
        }
        const BSP_USB_Status_t got = BSP_USB_Read(&byte, &at, &epoch);
        if (epoch != link.epoch) {
            link.epoch = epoch; link.used = 0U; link.session_valid = false;
        }
        __set_PRIMASK(mask);
        if (got != BSP_USB_OK) { break; }
        if (link.used != 0U && at - link.last_byte_ms > 100U) { link.used = 0U; }
        link.last_byte_ms = at;
        link.frame[link.used++] = byte;
        while (link.used >= 5U) {
            const uint8_t *p = link.frame;
            const bool header = p[0] == 0xA5U && p[1] == 0x5AU && p[2] == 1U &&
                ((p[3] == 0x12U && p[4] == 8U) || (p[3] == 0x11U && p[4] == 24U));
            const unsigned total = (unsigned)p[4] + 7U;
            if (header && link.used < total) { break; }
            if (header && MotorTelemetry_CRC16(&p[2], total - 4U) ==
                (uint16_t)(p[total-2U] | (uint16_t)p[total-1U]<<8)) {
                /* Accept atomically with epoch check, but dispatch outside ISR
                 * and outside IRQ masking. Later overflow invalidates next input. */
                __disable_irq();
                const bool accept = BSP_USB_RxEpoch() == link.epoch;
                __set_PRIMASK(mask);
                const uint32_t now_ms = HAL_GetTick();
                if (accept && now_ms - at <= 500U) { request(now_ms); }
                link.used = 0U;
                return link.response_length == 0U;
            }
            memmove(link.frame, link.frame + 1, --link.used);
        }
    }
    return true;
}
