#include "bsp_can.h"

#include <stdbool.h>
#include <stddef.h>

#define CAN_CALLBACK_COUNT 16U
#define CAN_RX_ISR_BUDGET 16U
#define CAN_ERROR_NOTIFICATIONS (FDCAN_IT_ARB_PROTOCOL_ERROR | \
    FDCAN_IT_DATA_PROTOCOL_ERROR | FDCAN_IT_ERROR_WARNING | \
    FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF)
#define CAN_NOTIFICATIONS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | CAN_ERROR_NOTIFICATIONS)

typedef struct {
    FDCAN_HandleTypeDef *bus;
    uint32_t id;
    BSP_CAN_RxCallback_t callback;
    void *context;
} CAN_RxEntry;

static CAN_RxEntry rx_entries[CAN_CALLBACK_COUNT];
static bool initialized[2];
static bool init_attempted[2];
/* Explicit task-driven recovery: 0 normal, 1 flush verification, 2 bus sync. */
static uint8_t recovery[2];
/* Temporary fault capture: remove with IRQ sampling/UART branch once the
 * Bus-Off root cause is fixed and verified. Do not turn this into recovery. */
static BSP_CAN_Diagnostics_t diagnostics[2];
#ifdef YLAMP_CAN_CAPTURE_ONLY
/* Bench-only SWD ring, before ID dispatch. No new bus traffic or filtering.
 * One receive-ISR writer per slot; both CAN IRQs have the same preempt priority.
 * Stable slot: sequence==sequence_end and even. Stores last 64 accepted frames. */
typedef struct {
    uint32_t sequence, tick_ms, bus_number, id, len;
    uint8_t data[8];
    uint32_t sequence_end;
} CAN_BenchCapture;
volatile CAN_BenchCapture g_can_bench_capture[64];
volatile uint32_t g_can_bench_capture_count;

static void capture_frame(unsigned index, const BSP_CAN_Frame_t *frame)
{
    const uint32_t number = g_can_bench_capture_count;
    const uint32_t even = (number + 1U) * 2U;
    volatile CAN_BenchCapture *slot = &g_can_bench_capture[number % 64U];
    slot->sequence = even - 1U;
    slot->tick_ms = HAL_GetTick();
    slot->bus_number = index + 1U;
    slot->id = frame->id;
    slot->len = frame->len;
    for (unsigned i = 0U; i < 8U; ++i) { slot->data[i] = frame->data[i]; }
    slot->sequence_end = even;
    slot->sequence = even;
    g_can_bench_capture_count = number + 1U;
}
#endif
static const uint32_t dlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
};

static int bus_index(const FDCAN_HandleTypeDef *bus)
{
    if (bus == &hfdcan1) { return 0; }
    if (bus == &hfdcan2) { return 1; }
    return -1;
}

static bool protocol_error(uint32_t code)
{
    return code >= 1U && code <= 6U;
}

/* HAL reads PSR once, consuming LEC. Serialize all readers so an IRQ cannot
 * replace the evidence between that read and its RAM copy. Both HAL getters
 * below are bounded register reads in the current H723 SDK, with no waits. */
static HAL_StatusTypeDef sample_protocol(FDCAN_HandleTypeDef *bus, int index,
                                          FDCAN_ProtocolStatusTypeDef *protocol)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t interrupts = bus->Instance->IR & bus->Instance->IE &
                                CAN_ERROR_NOTIFICATIONS;
    const HAL_StatusTypeDef result = HAL_FDCAN_GetProtocolStatus(bus, protocol);
    BSP_CAN_Diagnostics_t *diag = &diagnostics[index];
    if (result == HAL_OK &&
        (protocol_error(protocol->LastErrorCode) ||
         protocol_error(protocol->DataLastErrorCode) || interrupts != 0U ||
         (protocol->BusOff != 0U && !diag->bus_off_valid))) {
        FDCAN_ErrorCountersTypeDef counters = {0};
        const bool valid = HAL_FDCAN_GetErrorCounters(bus, &counters) == HAL_OK;
        const BSP_CAN_ErrorSnapshot_t snapshot = {
            .timestamp_ms = HAL_GetTick(),
            .interrupts = interrupts,
            .tx_pending_mask = bus->Instance->TXBRP,
            .lec = (uint8_t)protocol->LastErrorCode,
            .dlec = (uint8_t)protocol->DataLastErrorCode,
            .tx_errors = (uint8_t)counters.TxErrorCnt,
            .rx_errors = (uint8_t)counters.RxErrorCnt,
            .logged_errors = (uint8_t)counters.ErrorLogging,
            .bus_off = (uint8_t)protocol->BusOff,
            .error_passive = (uint8_t)protocol->ErrorPassive,
            .warning = (uint8_t)protocol->Warning,
            .rx_passive = (uint8_t)counters.RxErrorPassive,
            .counters_valid = (uint8_t)valid
        };
        if (diag->error_observations != UINT32_MAX) { ++diag->error_observations; }
        if (!diag->first_valid) {
            diag->first = snapshot;
            diag->first_valid = 1U;
        }
        diag->latest = snapshot;
        if (protocol->BusOff != 0U && !diag->bus_off_valid) {
            diag->bus_off = snapshot;
            diag->bus_off_valid = 1U;
        }
    }
    __set_PRIMASK(mask);
    return result;
}

void BSP_CAN_IRQHandler(FDCAN_HandleTypeDef *bus)
{
    const int index = bus_index(bus);
    if (index < 0 || !init_attempted[index]) { return; }
    if (bus->State == HAL_FDCAN_STATE_BUSY &&
        (bus->Instance->IR & bus->Instance->IE & CAN_ERROR_NOTIFICATIONS) != 0U) {
        FDCAN_ProtocolStatusTypeDef protocol = {0};
        (void)sample_protocol(bus, index, &protocol);
    }
    HAL_FDCAN_IRQHandler(bus);
}

BSP_CAN_Status_t BSP_CAN_GetDiagnostics(FDCAN_HandleTypeDef *bus,
                                        BSP_CAN_Diagnostics_t *output)
{
    const int index = bus_index(bus);
    if (index < 0 || output == NULL || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    if (!initialized[index]) { return BSP_CAN_NOT_READY; }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *output = diagnostics[index];
    __set_PRIMASK(mask);
    return BSP_CAN_OK;
}

BSP_CAN_Status_t BSP_CAN_Init(FDCAN_HandleTypeDef *bus)
{
    const int index = bus_index(bus);
    if (index < 0 || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    if (initialized[index]) {
        return BSP_CAN_BUSY;
    }
    /* 启动失败后不隐式重试；由调用者检查 HAL 状态并决定后续处理。 */
    if (init_attempted[index]) {
        return BSP_CAN_ERROR;
    }
    if (bus->State != HAL_FDCAN_STATE_READY ||
        bus->Init.FrameFormat != FDCAN_FRAME_CLASSIC ||
        bus->Init.Mode != FDCAN_MODE_NORMAL ||
        bus->Init.StdFiltersNbr == 0U ||
        bus->Init.RxFifo0ElmtsNbr == 0U ||
        bus->Init.RxFifo0ElmtsNbr > CAN_RX_ISR_BUDGET ||
        bus->Init.TxFifoQueueElmtsNbr == 0U) {
        return BSP_CAN_ERROR;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    /* 一个硬件掩码过滤器接收标准帧，精确 ID 在下方注册表分发。 */
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;
    init_attempted[index] = true;
    if (HAL_FDCAN_ConfigFilter(bus, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(bus, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(bus, CAN_NOTIFICATIONS, 0U) != HAL_OK) {
        return BSP_CAN_ERROR;
    }
    /* HAL clears INIT inside Start: an RX IRQ can run before Start returns.
     * Publish the receive path first so that IRQ drains/acks FIFO0 normally. */
    initialized[index] = true;
    __DMB();
    if (HAL_FDCAN_Start(bus) != HAL_OK) {
        initialized[index] = false;
        (void)HAL_FDCAN_DeactivateNotification(bus, CAN_NOTIFICATIONS);
        return BSP_CAN_ERROR;
    }
    return BSP_CAN_OK;
}

static BSP_CAN_Status_t submit(FDCAN_HandleTypeDef *bus, int index,
    uint32_t id, const uint8_t *data, uint32_t length_code)
{
    if (!initialized[index] || recovery[index] != 0U ||
        bus->State != HAL_FDCAN_STATE_BUSY) {
        return BSP_CAN_NOT_READY;
    }
    /* Healthy TX needs no PSR read/global IRQ masking. Error IRQs own capture;
     * sample here only if an error is pending or hardware entered INIT before
     * its IRQ ran. Do not consume LEC on every control/query frame. */
    if ((bus->Instance->IR & bus->Instance->IE & CAN_ERROR_NOTIFICATIONS) != 0U ||
        (bus->Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
        FDCAN_ProtocolStatusTypeDef protocol = {0};
        if (sample_protocol(bus, index, &protocol) != HAL_OK) {
            return BSP_CAN_ERROR;
        }
        /* HAL's BUSY state alone does not prove hardware left INIT. */
        if (protocol.BusOff != 0U ||
            (bus->Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
            return BSP_CAN_NOT_READY;
        }
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(bus) == 0U) {
        return BSP_CAN_BUSY;
    }
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = length_code;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    const HAL_StatusTypeDef result =
        HAL_FDCAN_AddMessageToTxFifoQ(bus, &header, data);
    if (result == HAL_OK) { return BSP_CAN_OK; }
    if (result == HAL_BUSY) { return BSP_CAN_BUSY; }
    return BSP_CAN_ERROR;
}

BSP_CAN_Status_t BSP_CAN_Send(FDCAN_HandleTypeDef *bus,
                              const BSP_CAN_Frame_t *frame)
{
    const int index = bus_index(bus);
    if (index < 0 || frame == NULL || frame->id > BSP_CAN_MAX_STD_ID ||
        frame->len > BSP_CAN_MAX_DATA_LENGTH || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    return submit(bus, index, frame->id, frame->data, dlc[frame->len]);
}

BSP_CAN_Status_t BSP_CAN_RecoverBusOff(FDCAN_HandleTypeDef *bus)
{
    const int index = bus_index(bus);
    if (index < 0 || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!initialized[index]) { return BSP_CAN_NOT_READY; }
    if (recovery[index] == 0U) {
        if (bus->State != HAL_FDCAN_STATE_BUSY ||
            (bus->Instance->CCCR & FDCAN_CCCR_INIT) == 0U) {
            return BSP_CAN_NOT_READY;
        }
        FDCAN_ProtocolStatusTypeDef protocol = {0};
        if (sample_protocol(bus, index, &protocol) != HAL_OK) { return BSP_CAN_ERROR; }
        if (protocol.BusOff == 0U) { return BSP_CAN_NOT_READY; }
        /* INIT is already acknowledged; reject sleep so HAL_Stop's two wait
         * conditions are already satisfied. No shared RCC reset or RAM clear.
         * CCE set by Stop resets RX FIFO status/TX requests (M_CAN 3.1.1). */
        if ((bus->Instance->CCCR & (FDCAN_CCCR_CSR | FDCAN_CCCR_CSA)) != 0U) {
            return BSP_CAN_ERROR;
        }
        recovery[index] = 1U; /* Block all submissions before touching queues. */
        if (HAL_FDCAN_Stop(bus) != HAL_OK) { return BSP_CAN_ERROR; }
        return BSP_CAN_BUSY;
    }
    if (recovery[index] == 1U) {
        if (bus->State != HAL_FDCAN_STATE_READY ||
            (bus->Instance->CCCR & (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE)) !=
                (FDCAN_CCCR_INIT | FDCAN_CCCR_CCE)) { return BSP_CAN_ERROR; }
        /* CCE resets TXFQS as well as TXBRP/RX status (M_CAN initialization).
         * Its handlers are idle in CCE: do not wait here for free-level=N.
         * TXBRP=0 proves no old request survives; verify free space AFTER Start. */
        if (bus->Instance->TXBRP != 0U ||
            HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0) != 0U) {
            return BSP_CAN_BUSY;
        }
        if (HAL_FDCAN_Start(bus) != HAL_OK) { return BSP_CAN_ERROR; }
        recovery[index] = 2U;
        return BSP_CAN_BUSY; /* Clearing INIT is not Bus-Off recovery complete. */
    }
    if ((bus->Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
        recovery[index] = 0U; /* A new Bus-Off can be retried by the owner. */
        return BSP_CAN_ERROR;
    }
    if (bus->State != HAL_FDCAN_STATE_BUSY) { return BSP_CAN_ERROR; }
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    if (sample_protocol(bus, index, &protocol) != HAL_OK) { return BSP_CAN_ERROR; }
    if (protocol.BusOff != 0U || bus->Instance->TXBRP != 0U ||
        HAL_FDCAN_GetTxFifoFreeLevel(bus) != bus->Init.TxFifoQueueElmtsNbr) {
        return BSP_CAN_BUSY;
    }
    recovery[index] = 0U;
    return BSP_CAN_OK;
}

BSP_CAN_Status_t BSP_CAN_RegisterRxCallback(FDCAN_HandleTypeDef *bus,
                                            uint32_t id,
                                            BSP_CAN_RxCallback_t callback,
                                            void *context)
{
    const int index = bus_index(bus);
    if (index < 0 || id > BSP_CAN_MAX_STD_ID || callback == NULL ||
        __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    if (!initialized[index]) {
        return BSP_CAN_NOT_READY;
    }
    CAN_RxEntry *free_entry = NULL;
    for (unsigned i = 0; i < CAN_CALLBACK_COUNT; ++i) {
        if (rx_entries[i].callback == NULL) {
            if (free_entry == NULL) { free_entry = &rx_entries[i]; }
        } else if (rx_entries[i].bus == bus && rx_entries[i].id == id) {
            return BSP_CAN_BUSY;
        }
    }
    if (free_entry == NULL) {
        return BSP_CAN_NO_RESOURCE;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    free_entry->bus = bus;
    free_entry->id = id;
    free_entry->context = context;
    free_entry->callback = callback;
    __DMB();
    __set_PRIMASK(mask);
    return BSP_CAN_OK;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *bus, uint32_t interrupts)
{
    const int index = bus_index(bus);
    if (index < 0 || !initialized[index] ||
        (interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }
    uint32_t count = HAL_FDCAN_GetRxFifoFillLevel(bus, FDCAN_RX_FIFO0);
    if (count > CAN_RX_ISR_BUDGET) { count = CAN_RX_ISR_BUDGET; }
    for (uint32_t i = 0; i < count; ++i) {
        FDCAN_RxHeaderTypeDef header = {0};
        /* 按最大 FD 长度接收，校验前也不会被畸形 DLC 写穿 8 字节栈缓冲。 */
        uint8_t data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(bus, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
            break;
        }
        if (header.IdType != FDCAN_STANDARD_ID ||
            header.RxFrameType != FDCAN_DATA_FRAME ||
            header.FDFormat != FDCAN_CLASSIC_CAN ||
            header.Identifier > BSP_CAN_MAX_STD_ID) {
            continue;
        }
        uint8_t len = 0U;
        while (len <= BSP_CAN_MAX_DATA_LENGTH && dlc[len] != header.DataLength) {
            ++len;
        }
        if (len > BSP_CAN_MAX_DATA_LENGTH) { continue; }
        BSP_CAN_Frame_t frame = {.id = header.Identifier, .len = len};
        for (uint8_t j = 0; j < len; ++j) { frame.data[j] = data[j]; }
#ifdef YLAMP_CAN_CAPTURE_ONLY
        capture_frame((unsigned)index, &frame);
#endif
        for (unsigned j = 0; j < CAN_CALLBACK_COUNT; ++j) {
            const CAN_RxEntry *entry = &rx_entries[j];
            if (entry->callback != NULL && entry->bus == bus &&
                entry->id == frame.id) {
                entry->callback(bus, &frame, entry->context);
                break;
            }
        }
    }
}
