#include "motor_serial.h"
#include "motor_tuning.h"
#include "serial_config.h"
#include "motor_telemetry.h"

#include <stddef.h>
#include <string.h>

#define RX_SIZE 2048U
#define REQUEST_SIZE 31U
_Static_assert(MOTOR_TASK_JOINT_COUNT <= MOTOR_TELEMETRY_MAX_JOINTS,
               "Telemetry joint capacity exceeded");
_Static_assert(MOTOR_TELEMETRY_FRAME_SIZE(MOTOR_TELEMETRY_MAX_JOINTS) <=
               BSP_UART_TX_CAPACITY, "Telemetry must fit DMA TX");

static struct {
    UART_HandleTypeDef *uart;
    uint8_t rx[RX_SIZE];
    uint32_t rx_at[RX_SIZE];
    volatile uint16_t head, tail;
    volatile uint32_t rx_overflows;
    uint32_t seen_overflows, crc_errors, tx_drops;
    uint8_t request[REQUEST_SIZE], used;
    uint32_t request_at;
    bool have_request;
    uint32_t sequence, last_telemetry_ms;
    uint8_t session[8], last_request[24], last_response[20];
    uint32_t last_id;
    bool session_valid;
} link;

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static HAL_StatusTypeDef send_frame(uint8_t type, uint8_t *frame, uint8_t length)
{
    frame[0] = 0xA5U; frame[1] = 0x5AU;
    frame[2] = 1U; frame[3] = type; frame[4] = length;
    const uint16_t crc = MotorTelemetry_CRC16(&frame[2], (unsigned)length + 3U);
    frame[5U + length] = (uint8_t)crc;
    frame[6U + length] = (uint8_t)(crc >> 8);
    return BSP_UART_Send(link.uart, frame, (uint16_t)(length + 7U));
}

static void receive(UART_HandleTypeDef *uart, const uint8_t *data,
                    uint16_t length, void *context)
{
    (void)uart; (void)context;
    const uint32_t arrived = HAL_GetTick();
    for (uint16_t i = 0U; i < length; ++i) {
        const uint16_t next = (uint16_t)((link.head + 1U) % RX_SIZE);
        if (next == link.tail) { ++link.rx_overflows; return; }
        link.rx[link.head] = data[i];
        link.rx_at[link.head] = arrived;
        __DMB();
        link.head = next;
    }
}

/* Caller masks RX IRQs: discard both the ring and any unaccepted request. */
static void discard_input(void)
{
    link.tail = link.head;
    link.seen_overflows = link.rx_overflows;
    link.used = 0U;
    link.have_request = false;
}

static void parse_window(void)
{
    while (link.used >= 5U) {
        const uint8_t *frame = link.request;
        const bool header = frame[0] == 0xA5U && frame[1] == 0x5AU &&
            frame[2] == 1U &&
            (((frame[3] == 0x10U || frame[3] == 0x12U) && frame[4] == 8U) ||
             (frame[3] == 0x11U && frame[4] == 24U));
        const unsigned total = (unsigned)frame[4] + 7U;
        if (header) {
            if (link.used < total) { return; }
            const uint16_t expected = (uint16_t)(
                frame[total - 2U] | ((uint16_t)frame[total - 1U] << 8));
            if (MotorTelemetry_CRC16(&frame[2], total - 4U) == expected) {
                link.have_request = true;
                return;
            }
            ++link.crc_errors;
        }
        memmove(link.request, &link.request[1], (size_t)link.used - 1U);
        --link.used;
    }
}

/* Sliding bounded request window handles noise, split/multiple frames and a bad
 * CRC without treating an UART IDLE callback as a protocol boundary. */
static void parse_requests(void)
{
    if (link.rx_overflows != link.seen_overflows) {
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        discard_input();
        __set_PRIMASK(mask);
        return;
    }
    if (!link.have_request) { parse_window(); }
    for (unsigned budget = 0U; budget < 64U && !link.have_request; ++budget) {
        if (link.tail == link.head) { break; }
        __DMB();
        const uint8_t byte = link.rx[link.tail];
        const uint32_t arrived = link.rx_at[link.tail];
        if (link.used != 0U && arrived - link.request_at > 100U) { link.used = 0U; }
        link.request_at = arrived;
        __DMB(); /* Finish reading the slot before the ISR may reuse it. */
        link.tail = (uint16_t)((link.tail + 1U) % RX_SIZE);
        link.request[link.used++] = byte;
        parse_window();
    }
}

static bool take_request(uint8_t frame[REQUEST_SIZE], uint32_t *arrived)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Parsing and dispatch are separated by IRQ-enabled work. Check overflow
     * atomically with acceptance; later overflow only invalidates future input.
     * Copy at most 31 bytes here; never run the RPC with interrupts masked. */
    if (link.rx_overflows != link.seen_overflows) { discard_input(); }
    const bool accepted = link.have_request;
    if (accepted) {
        const uint8_t consumed = (uint8_t)(link.request[4] + 7U);
        memcpy(frame, link.request, consumed);
        *arrived = link.request_at;
        link.have_request = false;
        link.used = (uint8_t)(link.used - consumed);
        memmove(link.request, &link.request[consumed], link.used);
    }
    __set_PRIMASK(mask);
    return accepted;
}

/* Stop-and-wait: one live session, strictly increasing IDs, last result cached.
 * A duplicate must be byte-identical. Older IDs can never execute again.
 * Cache before TX so a dropped reply cannot cause an action to run twice. */
static BSP_CAN_Status_t reliable_request(const uint8_t request[24],
    uint32_t now_ms, uint8_t response[20])
{
    memcpy(response, request, 12U);
    uint8_t *const reply = &response[12];
    const uint8_t *const rpc = &request[16];
    const uint32_t id = get_u32(&request[8]);
    reply[0] = rpc[0]; reply[1] = BSP_CAN_NOT_READY;
    reply[2] = rpc[2]; reply[3] = rpc[3];
    memset(&reply[4], 0, 4U);
    if (!link.session_valid || memcmp(request, link.session, 8U) != 0 ||
        id == 0U) { return BSP_CAN_OK; }
    if (id == link.last_id) {
        if (memcmp(request, link.last_request, 24U) == 0) {
            memcpy(response, link.last_response, 20U);
        } else { reply[1] = BSP_CAN_INVALID_ARG; }
        return BSP_CAN_OK;
    }
    if (link.last_id == UINT32_MAX || id != link.last_id + 1U) { return BSP_CAN_OK; }
    /* Host includes a recent MCU telemetry tick: retries must not extend the
     * existing 500ms intent lifetime. Future/previous-boot ticks also fail. */
    const BSP_CAN_Status_t result = MotorTuning_HandleRequest(
        rpc, get_u32(&request[12]), now_ms, reply);
    if (result != BSP_CAN_OK) { return result; }
    memcpy(link.last_request, request, 24U);
    memcpy(link.last_response, response, 20U);
    link.last_id = id;
    return BSP_CAN_OK;
}

static HAL_StatusTypeDef send_telemetry(uint32_t now_ms, const BSP_UART_Status_t *uart)
{
    MotorTask_Status_t status;
    if (MotorTask_GetStatus(&status) != BSP_CAN_OK) { return HAL_ERROR; }
    uint8_t frame[MOTOR_TELEMETRY_FRAME_SIZE(MOTOR_TASK_JOINT_COUNT)];
    MotorTelemetry_Header_t header = {
        .tick_ms = now_ms, .sequence = link.sequence,
        .state = (uint8_t)status.state, .reason = (uint8_t)status.reason,
        .policy = (uint8_t)status.policy_ready,
        .mode = (uint8_t)status.mode, .phase = (uint8_t)status.mode_phase,
        .last_transport = (uint8_t)status.last_transport,
        .rx_overflows = link.rx_overflows, .crc_errors = link.crc_errors,
        .tx_drops = link.tx_drops, .uart_error = uart->rx_error,
        .can_flags = (uint8_t)((uart->rx_running ? 16U : 0U) |
                              (uart->rx_rearm != HAL_OK ? 32U : 0U))
    };
    MotorTelemetry_Joint_t joints[MOTOR_TASK_JOINT_COUNT];
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const DM_Motor_Feedback_t *fb = &status.feedback[i];
        joints[i] = (MotorTelemetry_Joint_t) {
            .id = (uint8_t)(i + 1U),
            .flags = (uint8_t)(((status.feedback_valid_mask >> i) & 1U) |
                     (((status.disable_submitted_mask >> i) & 1U) << 1)),
            .can_id = g_motor_config.joint[i].device.can_id,
            .feedback_id = g_motor_config.joint[i].device.master_id,
            .disable_status = (uint8_t)status.disable_status[i],
            .received_ms = fb->received_at_ms, .received_count = fb->received_count,
            .position = fb->position_rad, .velocity = fb->velocity_rad_s,
            .torque = fb->torque_nm, .state = fb->state,
            .mos_c = fb->mos_temperature_c, .rotor_c = fb->rotor_temperature_c,
            .reference = status.reference_rad[i], .target = status.target_rad[i],
            .target_velocity = status.target_velocity_rad_s[i],
            .kp = status.target_kp[i], .kd = status.target_kd[i],
            .ff = status.target_torque_ff_nm[i],
            .target_reference = status.target_reference_rad[i],
            .tracking_error = status.target_reference_rad[i] - status.reference_rad[i]
        };
    }
    BSP_CAN_Diagnostics_t can;
    if (BSP_CAN_GetDiagnostics(g_motor_config.joint[0].device.bus, &can) == BSP_CAN_OK) {
        header.can_observations = can.error_observations;
        header.can_flags |= (uint8_t)(can.first_valid | (can.bus_off_valid << 1) |
            (can.first.counters_valid << 2) | (can.bus_off.counters_valid << 3));
        header.first_lec = can.first.lec; header.first_tec = can.first.tx_errors;
        header.first_rec = can.first.rx_errors; header.first_ms = can.first.timestamp_ms;
        header.bo_ms = can.bus_off.timestamp_ms; header.bo_lec = can.bus_off.lec;
        header.bo_tec = can.bus_off.tx_errors; header.bo_rec = can.bus_off.rx_errors;
        header.pending = can.bus_off.tx_pending_mask;
    }
    const size_t length = MotorTelemetry_Encode(frame, sizeof(frame), &header,
                                                joints, MOTOR_TASK_JOINT_COUNT);
    return length == 0U ? HAL_ERROR : BSP_UART_Send(link.uart, frame, (uint16_t)length);
}

HAL_StatusTypeDef MotorSerial_Init(UART_HandleTypeDef *uart)
{
    if (__get_IPSR() != 0U || uart == NULL ||
        uart->Init.BaudRate != YLAMP_SERIAL_BAUD_RATE ||
        uart->Init.WordLength != UART_WORDLENGTH_8B ||
        uart->Init.StopBits != UART_STOPBITS_1 || uart->Init.Parity != UART_PARITY_NONE ||
        uart->Init.Mode != UART_MODE_TX_RX) { return HAL_ERROR; }
    if (link.uart != NULL) { return HAL_BUSY; }
    const HAL_StatusTypeDef result = BSP_UART_InitDMA(uart, receive, NULL);
    if (result == HAL_OK) {
        link.uart = uart;
        link.last_telemetry_ms = HAL_GetTick();
    }
    return result;
}

HAL_StatusTypeDef MotorSerial_Run(uint32_t now_ms)
{
    if (__get_IPSR() != 0U || link.uart == NULL) { return HAL_ERROR; }
    BSP_UART_Status_t uart;
    if (BSP_UART_GetStatus(link.uart, &uart) != HAL_OK) { return HAL_ERROR; }
    parse_requests();
    /* RX IRQs can timestamp bytes after the caller sampled now_ms. */
    now_ms = HAL_GetTick();
    /* 8N1 wire budget: telemetry uses at most 60%, retaining room for RPC.
     * Two joints remain 50Hz; six records automatically need at least 33ms. */
    const uint32_t wire_period = (uint32_t)(
        (MOTOR_TELEMETRY_FRAME_SIZE(MOTOR_TASK_JOINT_COUNT) * 100000U +
         YLAMP_SERIAL_BAUD_RATE * 6U - 1U) / (YLAMP_SERIAL_BAUD_RATE * 6U));
    const uint32_t period = wire_period > YLAMP_SERIAL_PERIOD_MS ?
        wire_period : YLAMP_SERIAL_PERIOD_MS;
    const bool due = now_ms - link.last_telemetry_ms >= period;
    if (due) { link.last_telemetry_ms = now_ms; ++link.sequence; }
    uint8_t request[REQUEST_SIZE];
    uint32_t arrived;
    if (!uart.tx_busy && take_request(request, &arrived)) {
        uint8_t response[27];
        uint8_t type = 0x90U, length = 8U;
        BSP_CAN_Status_t result = BSP_CAN_OK;
        if (request[3] == 0x12U) {
            /* Repeated HELLO for the same session must not clear dedup state. */
            if (!link.session_valid || memcmp(link.session, &request[5], 8U) != 0) {
                memcpy(link.session, &request[5], 8U);
                link.session_valid = true;
                link.last_id = 0U;
            }
            type = 0x92U;
            memcpy(&response[5], link.session, 8U);
        } else if (request[3] == 0x11U) {
            type = 0x91U; length = 20U;
            result = reliable_request(&request[5], now_ms, &response[5]);
        } else {
            result = MotorTuning_HandleRequest(
                &request[5], arrived, now_ms, &response[5]);
        }
        if (due) { ++link.tx_drops; }
        if (result != BSP_CAN_OK) { return HAL_ERROR; }
        const HAL_StatusTypeDef sent = send_frame(type, response, length);
        if (sent != HAL_OK) { ++link.tx_drops; }
        return sent;
    }
    if (!due) { return HAL_OK; }
    if (uart.tx_busy) { ++link.tx_drops; return HAL_BUSY; }
    const HAL_StatusTypeDef sent = send_telemetry(now_ms, &uart);
    if (sent != HAL_OK) { ++link.tx_drops; }
    return sent;
}
