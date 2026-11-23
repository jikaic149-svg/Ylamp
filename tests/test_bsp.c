#include "bsp_can.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "bsp_rgb.h"
#include "bsp_uart.h"
#include "dm_motor.h"
#include "five_mit.h"
#include "five_control.h"
#include "five_support.h"
#include "five_workspace.h"
#include "joint_calibration.h"
#include "serial_config.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpioa, test_gpioc;
uint32_t test_ipsr, test_primask, test_tick, test_spi6_instance, test_spi2_instance;
FDCAN_HandleTypeDef hfdcan1, hfdcan2;
UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5;
UART_HandleTypeDef huart6, huart7, huart8, huart9, huart10;
SPI_HandleTypeDef hspi6, hspi2;
static USART_TypeDef uart_instances[10];

static unsigned gpio_writes, gpio_high_writes;
static HAL_StatusTypeDef can_config_result = HAL_OK, can_start_result = HAL_OK;
static HAL_StatusTypeDef can_tx_result = HAL_OK, can_rx_result = HAL_OK;
static unsigned can_config_calls, can_start_calls, can_tx_calls, can_rx_calls;
static unsigned can_stop_calls;
static HAL_StatusTypeDef can_stop_result = HAL_OK;
static bool can_stop_flush = true;
static unsigned can_deactivate_calls, can_callbacks;
static unsigned can_irq_calls, can_protocol_reads, can_counter_reads;
static FDCAN_GlobalTypeDef can_instances[2];
static FDCAN_ProtocolStatusTypeDef can_protocol[2];
static FDCAN_ErrorCountersTypeDef can_counters[2];
static HAL_StatusTypeDef can_protocol_result = HAL_OK, can_counter_result = HAL_OK;
typedef struct {
    uint32_t free_slots, rx_pending;
    FDCAN_RxHeaderTypeDef rx_header;
    bool custom_rx;
    uint8_t rx_data[8];
    unsigned tx_calls, rx_calls;
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[64];
} FakeCAN_IO;
static FakeCAN_IO can_io[2];
/* Existing single-bus cases keep their CAN1 fixture names. */
#define can_free (can_io[0].free_slots)
#define can_rx_pending (can_io[0].rx_pending)
#define can_rx_header (can_io[0].rx_header)
static uint32_t can_fail_mit_id;
static FDCAN_TxHeaderTypeDef can_tx_header;
static uint8_t can_tx_data[64];
static void (*can_start_hook)(FDCAN_HandleTypeDef *bus);
static unsigned motor_enable_calls[16], motor_disable_calls[16];
static unsigned motor_query_calls[16], motor_mit_calls[16];
static uint8_t motor_last_mit[16][8];
static HAL_StatusTypeDef uart_rx_result = HAL_OK, uart_tx_result = HAL_OK;
static HAL_StatusTypeDef uart_abort_result = HAL_OK;
static unsigned uart_rx_calls, uart_tx_calls, uart_abort_calls, uart_callbacks;
static uint8_t *uart_rx_data;
static const uint8_t *uart_tx_data;
static uint16_t uart_tx_len;
static uint32_t spi_clock = 50000000U;
static HAL_StatusTypeDef spi_result = HAL_OK;
static uint8_t spi_wire[524];
static unsigned spi_calls;
static void (*tick_hook)(void);


static unsigned fake_can_index(const FDCAN_HandleTypeDef *bus)
{
    assert(bus == &hfdcan1 || bus == &hfdcan2);
    return bus == &hfdcan1 ? 0U : 1U;
}

uint32_t HAL_GetTick(void)
{
    void (*hook)(void) = tick_hook;
    tick_hook = NULL; /* A simulated IRQ may itself timestamp incoming data. */
    if (hook != NULL) { hook(); }
    return test_tick;
}
void HAL_Delay(uint32_t ms) { test_tick += ms; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout)
{
    (void)spi; (void)tx; (void)rx; (void)size; (void)timeout;
    return HAL_ERROR; /* IMU absent in the existing motor-only regressions. */
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    (void)spi; (void)tx; (void)rx; (void)size;
    return HAL_ERROR;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    ++gpio_writes;
    if (state == GPIO_PIN_SET) {
        ++gpio_high_writes;
        port->ODR |= pin;
    } else {
        port->ODR &= ~(uint32_t)pin;
    }
}

HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *bus,
                                         const FDCAN_FilterTypeDef *filter)
{
    (void)bus;
    ++can_config_calls;
    assert(filter->IdType == FDCAN_STANDARD_ID);
    assert(filter->FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
    assert(filter->FilterID1 == 0U && filter->FilterID2 == 0U);
    return can_config_result;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *bus,
                                              uint32_t a, uint32_t b,
                                              uint32_t c, uint32_t d)
{
    (void)bus;
    assert(a == FDCAN_REJECT && b == FDCAN_REJECT);
    assert(c == FDCAN_REJECT_REMOTE && d == FDCAN_REJECT_REMOTE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *bus,
                                                uint32_t flags, uint32_t buffers)
{
    const uint32_t expected = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
        FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
        FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF;
    assert(flags == expected && buffers == 0U);
    bus->Instance->IE |= flags;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *bus,
                                                  uint32_t flags)
{
    bus->Instance->IE &= ~flags;
    ++can_deactivate_calls;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *bus)
{
    ++can_start_calls;
    if (can_start_result == HAL_OK) {
        bus->State = HAL_FDCAN_STATE_BUSY;
        bus->Instance->CCCR &= ~(FDCAN_CCCR_INIT | FDCAN_CCCR_CCE);
        if (bus->Instance->TXBRP == 0U) {
            can_io[fake_can_index(bus)].free_slots = bus->Init.TxFifoQueueElmtsNbr;
        }
        if (can_start_hook != NULL) { can_start_hook(bus); }
    }
    return can_start_result;
}
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *bus)
{
    ++can_stop_calls;
    assert(bus->Instance->CCCR & FDCAN_CCCR_INIT);
    assert(!(bus->Instance->CCCR & (FDCAN_CCCR_CSA | FDCAN_CCCR_CSR)));
    if (can_stop_result != HAL_OK) { return can_stop_result; }
    bus->Instance->CCCR |= FDCAN_CCCR_CCE;
    bus->State = HAL_FDCAN_STATE_READY;
    if (can_stop_flush) {
        bus->Instance->TXBRP = 0U;
        can_io[fake_can_index(bus)].rx_pending = 0U;
        /* GetTxFifoFreeLevel models TXFQS reset/idle while this bus has CCE. */
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *bus,
                                             FDCAN_ProtocolStatusTypeDef *status)
{
    const unsigned index = fake_can_index(bus);
    assert(test_primask == 1U); /* Poll/IRQ readers must not race. */
    ++can_protocol_reads;
    if (can_protocol_result != HAL_OK) { return can_protocol_result; }
    *status = can_protocol[index];
    /* Actual PSR reads consume LEC/DLEC; later reads must not invent a cause. */
    can_protocol[index].LastErrorCode = 7U;
    can_protocol[index].DataLastErrorCode = 7U;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *bus,
                                             FDCAN_ErrorCountersTypeDef *out)
{
    const unsigned index = fake_can_index(bus);
    assert(test_primask == 1U);
    ++can_counter_reads;
    if (can_counter_result != HAL_OK) { return can_counter_result; }
    *out = can_counters[index];
    can_counters[index].ErrorLogging = 0U; /* ECR read consumes CEL. */
    return HAL_OK;
}
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *bus)
{
    ++can_irq_calls;
    const uint32_t flags = bus->Instance->IR & bus->Instance->IE;
    bus->Instance->IR &= ~flags; /* HAL clears error IR before its callbacks. */
    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        HAL_FDCAN_RxFifo0Callback(bus, flags);
    }
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *bus)
{
    return (bus->Instance->CCCR & FDCAN_CCCR_CCE) != 0U
        ? 0U : can_io[fake_can_index(bus)].free_slots;
}
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus,
                                               const FDCAN_TxHeaderTypeDef *header,
                                               const uint8_t *data)
{
    FakeCAN_IO *io = &can_io[fake_can_index(bus)];
    ++can_tx_calls;
    can_tx_header = *header;
    static const uint8_t lengths[16] = {
        0U,1U,2U,3U,4U,5U,6U,7U,8U,12U,16U,20U,24U,32U,48U,64U};
    assert(header->DataLength < 16U);
    const unsigned length = lengths[header->DataLength];
    assert(length <= 8U && header->FDFormat == FDCAN_CLASSIC_CAN &&
           header->BitRateSwitch == FDCAN_BRS_OFF);
    memcpy(can_tx_data, data, length);
    ++io->tx_calls;
    io->tx_header = *header;
    memcpy(io->tx_data, data, length);
    if (header->Identifier == can_fail_mit_id && header->DataLength == 8U) {
        bool special = true;
        for (unsigned i = 0U; i < 7U; ++i) {
            if (data[i] != 0xFFU) { special = false; }
        }
        if (!special) { return HAL_ERROR; }
    }
    if (can_tx_result == HAL_OK && header->DataLength == 8U) {
        if (header->Identifier == 0x7FFU && data[0] < 16U && data[2] == 0xCCU) {
            ++motor_query_calls[data[0]];
        } else if (header->Identifier > 0U && header->Identifier < 16U) {
            bool special = true;
            for (unsigned i = 0U; i < 7U; ++i) {
                if (data[i] != 0xFFU) { special = false; }
            }
            const unsigned id = header->Identifier;
            if (special && data[7] == 0xFCU) {
                ++motor_enable_calls[id];
            } else if (special && data[7] == 0xFDU) {
                ++motor_disable_calls[id];
            } else {
                ++motor_mit_calls[id];
                memcpy(motor_last_mit[id], data, 8U);
            }
        }
    }
    return can_tx_result;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *bus, uint32_t fifo)
{
    assert(fifo == FDCAN_RX_FIFO0);
    return can_io[fake_can_index(bus)].rx_pending;
}
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *bus, uint32_t fifo,
                                        FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    FakeCAN_IO *io = &can_io[fake_can_index(bus)];
    (void)fifo;
    ++can_rx_calls;
    if (can_rx_result != HAL_OK) { return can_rx_result; }
    assert(io->rx_pending != 0U);
    --io->rx_pending;
    ++io->rx_calls;
    *header = io->rx_header;
    if (io->custom_rx && io->rx_header.DataLength <= 8U) {
        memcpy(data, io->rx_data, 8U);
    } else {
        memset(data, 0x5A, io->rx_header.DataLength > 8U ? 64U : 8U);
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *uart,
                                             uint8_t *data, uint16_t len)
{
    ++uart_rx_calls;
    assert(len == BSP_UART_RX_CAPACITY);
    if (uart_rx_result == HAL_OK) {
        uart_rx_data = data;
        uart->RxState = HAL_UART_STATE_BUSY_RX;
    }
    return uart_rx_result;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,
                                      const uint8_t *data, uint16_t len)
{
    ++uart_tx_calls;
    if (uart_tx_result == HAL_OK) {
        uart_tx_data = data;
        uart_tx_len = len;
        uart->gState = HAL_UART_STATE_BUSY_TX;
    }
    return uart_tx_result;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                              uint8_t *data, uint16_t len)
{
    ++uart_rx_calls;
    assert(len == BSP_UART_DMA_RX_CAPACITY && uart->hdmarx != NULL);
    if (uart_rx_result == HAL_OK) {
        uart_rx_data = data;
        uart->hdmarx->remaining = len;
        uart->RxState = HAL_UART_STATE_BUSY_RX;
        uart->Instance->CR3 |= USART_CR3_DMAR;
    }
    return uart_rx_result;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                       const uint8_t *data, uint16_t len)
{
    assert(uart->hdmatx != NULL);
    return HAL_UART_Transmit_IT(uart, data, len);
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    ++uart_abort_calls;
    if (uart_abort_result == HAL_OK) { uart->RxState = HAL_UART_STATE_READY; }
    return uart_abort_result;
}

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t peripheral)
{
    if (peripheral == RCC_PERIPHCLK_SPI2) { return 80000000U; }
    assert(peripheral == RCC_PERIPHCLK_SPI6);
    return spi_clock;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, const uint8_t *data,
                                   uint16_t len, uint32_t timeout)
{
    assert(spi == &hspi6 && len == sizeof(spi_wire) && timeout == 5U);
    ++spi_calls;
    memcpy(spi_wire, data, len);
    return spi_result;
}

static void can_receive(FDCAN_HandleTypeDef *bus, const BSP_CAN_Frame_t *frame,
                         void *context)
{
    assert(bus == &hfdcan1 && context == &can_callbacks);
    assert(frame->id == 0x13U && frame->len == 8U && frame->data[0] == 0x5AU);
    ++can_callbacks;
}

static void test_can(void)
{
    BSP_CAN_Frame_t frame = {.id = 3U, .len = 8U, .data = {1, 2, 3}};
    assert(BSP_CAN_Init(NULL) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_Send(NULL, &frame) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_Send(&hfdcan1, NULL) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, can_receive, NULL)
           == BSP_CAN_NOT_READY);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(can_tx_calls == 0U && gpio_high_writes == 0U);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_BUSY);
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, NULL, NULL)
           == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, can_receive, &can_callbacks)
           == BSP_CAN_OK);
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, can_receive, NULL)
           == BSP_CAN_BUSY);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(can_tx_header.Identifier == 3U && can_tx_header.DataLength == 8U);
    assert(can_tx_header.FDFormat == FDCAN_CLASSIC_CAN && can_tx_data[1] == 2U);
    frame.len = 0U;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(can_tx_header.DataLength == 0U);
    assert(can_protocol_reads == 0U && can_counter_reads == 0U);
    frame.len = 9U;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_INVALID_ARG);
    frame.len = 8U;
    frame.id = 0x800U;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_INVALID_ARG);
    frame.id = 3U;
    can_free = 0U;
    const unsigned sent = can_tx_calls;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_BUSY);
    assert(can_tx_calls == sent);
    can_free = 8U;
    /* Hardware INIT can be set even while HAL still says BUSY, without BO. */
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(can_tx_calls == sent);
    assert(hfdcan1.Instance->CCCR == FDCAN_CCCR_INIT && can_start_calls == 1U);
    can_protocol[0].BusOff = 1U;
    hfdcan1.Instance->IR = FDCAN_IT_BUS_OFF;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(can_tx_calls == sent);
    /* Fixture reset, not a BSP recovery operation. */
    hfdcan1.Instance->CCCR = hfdcan1.Instance->IR = 0U;
    can_protocol[0].BusOff = 0U;
    can_tx_result = HAL_ERROR;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_ERROR);
    assert(can_tx_calls == sent + 1U); /* 不软件重试 */
    can_tx_result = HAL_OK;

    can_rx_header = (FDCAN_RxHeaderTypeDef){.Identifier = 0x13U, .DataLength = 8U};
    can_rx_pending = 20U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_callbacks == 16U && can_rx_pending == 4U); /* ISR 工作量有界 */
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_callbacks == 20U);
    can_rx_header.IdType = FDCAN_EXTENDED_ID;
    can_rx_pending = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    can_rx_header.IdType = FDCAN_STANDARD_ID;
    can_rx_header.DataLength = 15U; /* 畸形 DLC，不写穿临时缓冲 */
    can_rx_pending = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_callbacks == 20U);
    can_rx_header.DataLength = 8U;
    can_rx_header.Identifier = 0x77U;
    can_rx_pending = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_callbacks == 20U); /* 未注册 ID 不交付 */
    can_rx_result = HAL_ERROR;
    can_rx_pending = 1U;
    const unsigned received = can_rx_calls;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_rx_calls == received + 1U);
    can_rx_result = HAL_OK;
    for (uint32_t id = 0x100U; id < 0x10FU; ++id) {
        assert(BSP_CAN_RegisterRxCallback(&hfdcan1, id, can_receive, NULL) == BSP_CAN_OK);
    }
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x200U, can_receive, NULL)
           == BSP_CAN_NO_RESOURCE);
    test_primask = 1U;
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, can_receive, NULL)
           == BSP_CAN_BUSY);
    assert(test_primask == 1U);
    test_primask = 0U;
    can_config_result = HAL_ERROR;
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_ERROR);
    can_config_result = HAL_OK;
    const unsigned configured = can_config_calls;
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_ERROR);
    assert(can_config_calls == configured);
    test_ipsr = 1U;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
}

static void test_key(void)
{
    bool pressed = false;
    assert(BSP_KEY_GetPressed(&pressed) == HAL_ERROR);
    assert(BSP_KEY_Init(0U) == HAL_ERROR);
    assert(BSP_KEY_Init(1001U) == HAL_ERROR);
    assert(BSP_KEY_Init(20U) == HAL_OK);
    assert(BSP_KEY_Init(20U) == HAL_BUSY);
    assert(BSP_KEY_Scan(1U) == BSP_KEY_EVENT_NONE);
    test_gpioa.IDR = 0U;
    assert(BSP_KEY_Scan(10U) == BSP_KEY_EVENT_NONE);
    test_gpioa.IDR = KEY__INPUT_Pin;
    assert(BSP_KEY_Scan(15U) == BSP_KEY_EVENT_NONE);
    test_gpioa.IDR = 0U;
    assert(BSP_KEY_Scan(20U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_Scan(39U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_Scan(40U) == BSP_KEY_EVENT_PRESSED);
    assert(BSP_KEY_Scan(100U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_GetPressed(&pressed) == HAL_OK && pressed);
    test_gpioa.IDR = KEY__INPUT_Pin;
    assert(BSP_KEY_Scan(110U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_Scan(130U) == BSP_KEY_EVENT_RELEASED);
    test_gpioa.IDR = 0U;
    assert(BSP_KEY_Scan(UINT32_MAX - 9U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_Scan(9U) == BSP_KEY_EVENT_NONE);
    assert(BSP_KEY_Scan(10U) == BSP_KEY_EVENT_PRESSED); /* Tick 回绕 */
    assert(BSP_KEY_GetPressed(NULL) == HAL_ERROR);
}

static void test_power(void)
{
    bool enabled = false;
    const uint32_t motor_pins = DC24_0__OUTPUT_Pin | DC24_1__OUTPUT_Pin;
    assert(BSP_POWER_Set(BSP_POWER_OUT1, true) == HAL_ERROR);
    assert(BSP_POWER_SetMotorPower(true) == HAL_ERROR);
    assert(BSP_POWER_SetMotorPower(false) == HAL_ERROR);
    assert(gpio_writes == 0U);
    test_gpioc.ODR = 0xFFFFU;
    assert(BSP_POWER_Init() == HAL_OK);
    assert(gpio_high_writes == 0U);
    assert((test_gpioc.ODR & 0xE000U) == 0U);
    assert((test_gpioc.ODR & 1U) != 0U); /* 不碰其他 IO */
    assert(BSP_POWER_Set((BSP_POWER_Branch_t)-1, true) == HAL_ERROR);
    assert(BSP_POWER_Set(BSP_POWER_OUT1, true) == HAL_OK);
    assert((test_gpioc.ODR & (1U << 14)) != 0U);
    assert((test_gpioc.ODR & (1U << 13)) == 0U);
    assert(BSP_POWER_GetEnabled(BSP_POWER_OUT1, &enabled) == HAL_OK && enabled);
    assert(BSP_POWER_Init() == HAL_BUSY);
    assert(BSP_POWER_GetEnabled(BSP_POWER_OUT1, &enabled) == HAL_OK && enabled);
    assert(BSP_POWER_Set(BSP_POWER_OUT2, true) == HAL_OK);
    assert(BSP_POWER_Set(BSP_POWER_EXT_5V, true) == HAL_OK);
    assert((test_gpioc.ODR & 0xE000U) == 0xE000U);
    unsigned writes = gpio_writes;
    assert(BSP_POWER_SetMotorPower(false) == HAL_OK);
    assert(gpio_writes == writes + 1U); /* 两路 EN 同一次写入 */
    assert((test_gpioc.ODR & motor_pins) == 0U);
    assert((test_gpioc.ODR & DC5__OUTPUT_Pin) != 0U);
    writes = gpio_writes;
    assert(BSP_POWER_SetMotorPower(true) == HAL_OK);
    assert(gpio_writes == writes + 1U);
    assert((test_gpioc.ODR & 0xE000U) == 0xE000U);
    assert(BSP_POWER_Set(BSP_POWER_EXT_5V, false) == HAL_OK);
    assert(BSP_POWER_SetMotorPower(false) == HAL_OK);
    assert((test_gpioc.ODR & 0xE000U) == 0U);
    assert(BSP_POWER_SetMotorPower(true) == HAL_OK);
    assert(BSP_POWER_SetMotorPower(true) == HAL_OK); /* 重复开启不产生复位脉冲 */
    assert((test_gpioc.ODR & 0xE000U) == motor_pins);
    assert((test_gpioc.ODR & 1U) != 0U);
    assert(BSP_POWER_GetEnabled(BSP_POWER_OUT1, &enabled) == HAL_OK && enabled);
    assert(BSP_POWER_GetEnabled(BSP_POWER_OUT2, &enabled) == HAL_OK && enabled);
    const uint32_t odr_before_isr = test_gpioc.ODR;
    writes = gpio_writes;
    test_ipsr = 1U;
    assert(BSP_POWER_Set(BSP_POWER_OUT1, true) == HAL_ERROR);
    assert(BSP_POWER_SetMotorPower(false) == HAL_ERROR);
    assert(BSP_POWER_SetMotorPower(true) == HAL_ERROR);
    assert(gpio_writes == writes && test_gpioc.ODR == odr_before_isr);
    test_ipsr = 0U;
    assert(BSP_POWER_AllOff() == HAL_OK);
    assert((test_gpioc.ODR & 0xE000U) == 0U);
    assert(BSP_POWER_GetEnabled(BSP_POWER_OUT1, NULL) == HAL_ERROR);
}

static void uart_receive(UART_HandleTypeDef *uart, const uint8_t *data,
                          uint16_t len, void *context)
{
    assert(uart == &huart1 && context == &uart_callbacks);
    assert(len == 3U && data[0] == 0x12U && data[2] == 0x56U);
    if (uart_rx_result == HAL_OK) {
        assert(uart_rx_data != data); /* 续接成功时已切到另一个缓冲 */
    }
    ++uart_callbacks;
}

static void test_uart(void)
{
    uint8_t data[3] = {0x12U, 0x34U, 0x56U};
    BSP_UART_Status_t status;
    assert(BSP_UART_Init(NULL, NULL, NULL) == HAL_ERROR);
    assert(BSP_UART_Init(&huart4, NULL, NULL) == HAL_ERROR);
    assert(BSP_UART_Send(&huart1, data, 3U) == HAL_ERROR);
    assert(BSP_UART_Init(&huart1, uart_receive, &uart_callbacks) == HAL_OK);
    assert(BSP_UART_Init(&huart1, uart_receive, NULL) == HAL_BUSY);
    assert(BSP_UART_Send(&huart1, NULL, 3U) == HAL_ERROR);
    assert(BSP_UART_Send(&huart1, data, BSP_UART_TX_CAPACITY + 1U) == HAL_ERROR);
    assert(BSP_UART_Send(&huart1, data, 0U) == HAL_ERROR);
    assert(BSP_UART_Send(&huart1, data, 3U) == HAL_OK);
    assert(uart_tx_data != data && uart_tx_len == 3U);
    data[0] = 0xFFU;
    assert(uart_tx_data[0] == 0x12U);
    assert(BSP_UART_Send(&huart1, data, 3U) == HAL_BUSY);
    assert(uart_tx_data[0] == 0x12U && uart_tx_calls == 1U);
    memcpy(uart_rx_data, (uint8_t[]){0x12, 0x34, 0x56}, 3U);
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart1, 3U);
    assert(uart_callbacks == 1U && uart_rx_calls == 2U);
    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    huart1.RxState = HAL_UART_STATE_READY;
    const unsigned rx_before_error = uart_rx_calls;
    HAL_UART_ErrorCallback(&huart1);
    assert(uart_rx_calls == rx_before_error); /* 故障不自动重启 */
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK);
    assert(!status.rx_running && status.tx_busy && status.rx_error == HAL_UART_ERROR_ORE);
    assert(BSP_UART_Send(&huart1, data, 3U) == HAL_BUSY); /* RX 错误不释放 TX */
    assert(BSP_UART_RestartRx(&huart1) == HAL_OK);
    assert(uart_abort_calls == 1U && huart1.gState == HAL_UART_STATE_BUSY_TX);
    huart1.gState = HAL_UART_STATE_READY;
    HAL_UART_TxCpltCallback(&huart1);
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK && !status.tx_busy);
    assert(status.rx_running && status.rx_error == HAL_UART_ERROR_NONE);
    uart_tx_result = HAL_ERROR;
    assert(BSP_UART_Send(&huart1, data, 3U) == HAL_ERROR);
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK && !status.tx_busy);
    uart_tx_result = HAL_OK;
    uart_rx_result = HAL_BUSY;
    memcpy(uart_rx_data, (uint8_t[]){0x12, 0x34, 0x56}, 3U);
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart1, 3U);
    assert(uart_callbacks == 2U);
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK);
    assert(!status.rx_running && status.rx_rearm == HAL_BUSY);
    uart_rx_result = HAL_OK;
    assert(BSP_UART_RestartRx(&huart1) == HAL_OK);
    const unsigned old_abort = uart_abort_calls;
    huart1.Instance->CR3 = USART_CR3_DMAR;
    assert(BSP_UART_RestartRx(&huart1) == HAL_BUSY);
    assert(uart_abort_calls == old_abort);
    huart1.Instance->CR3 = 0U;
    uart_abort_result = HAL_ERROR;
    assert(BSP_UART_RestartRx(&huart1) == HAL_ERROR);
    uart_abort_result = HAL_OK;
    assert(BSP_UART_RestartRx(&huart1) == HAL_OK);
    huart1.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_RxEventCallback(&huart1, BSP_UART_RX_CAPACITY + 1U);
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK && !status.rx_running);
    test_primask = 1U;
    assert(BSP_UART_GetStatus(&huart1, &status) == HAL_OK && test_primask == 1U);
    test_primask = 0U;
    assert(BSP_UART_GetStatus(&huart1, NULL) == HAL_ERROR);
    huart7.Init.WordLength = UART_WORDLENGTH_9B;
    assert(BSP_UART_Init(&huart7, NULL, NULL) == HAL_ERROR);
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    uart_rx_result = HAL_ERROR;
    assert(BSP_UART_Init(&huart7, uart_receive, NULL) == HAL_ERROR);
    uart_rx_result = HAL_OK;
    assert(BSP_UART_RestartRx(&huart7) == HAL_OK); /* 首次启动失败可显式恢复 */
    huart5.Init.Mode = UART_MODE_RX;
    assert(BSP_UART_Init(&huart5, NULL, NULL) == HAL_OK);
    assert(BSP_UART_Send(&huart5, data, 3U) == HAL_ERROR);
}

static void test_rgb(void)
{
    assert(BSP_RGB_Set(1U, 2U, 3U) == HAL_ERROR);
    spi_clock = 64000000U;
    assert(BSP_RGB_Init() == HAL_ERROR && spi_calls == 0U);
    spi_clock = 50000000U;
    assert(BSP_RGB_Init() == HAL_OK && spi_calls == 1U);
    assert(BSP_RGB_Init() == HAL_BUSY);
    for (unsigned i = 0; i < sizeof(spi_wire); ++i) {
        assert(spi_wire[i] == ((i >= 250U && i < 274U) ? 0x60U : 0U));
    }
    assert(BSP_RGB_Set(0x80U, 0x01U, 0xFFU) == HAL_OK);
    assert(spi_wire[250U] == 0x60U && spi_wire[257U] == 0x78U); /* G */
    assert(spi_wire[258U] == 0x78U && spi_wire[259U] == 0x60U); /* R */
    for (unsigned i = 266U; i < 274U; ++i) { assert(spi_wire[i] == 0x78U); }
    hspi6.State = HAL_SPI_STATE_BUSY_TX;
    const unsigned previous = spi_calls;
    assert(BSP_RGB_Set(0U, 0U, 0U) == HAL_BUSY && spi_calls == previous);
    hspi6.State = HAL_SPI_STATE_READY;
    spi_result = HAL_TIMEOUT;
    assert(BSP_RGB_Set(0U, 0U, 0U) == HAL_TIMEOUT && spi_calls == previous + 1U);
    test_ipsr = 1U;
    assert(BSP_RGB_Set(0U, 0U, 0U) == HAL_ERROR);
    test_ipsr = 0U;
    test_primask = 1U;
    assert(BSP_RGB_Set(0U, 0U, 0U) == HAL_ERROR);
    test_primask = 0U;
}

static void initialize_fake_hardware(void)
{
    const FDCAN_HandleTypeDef can = {
        .Init = {.FrameFormat = FDCAN_FRAME_CLASSIC, .Mode = FDCAN_MODE_NORMAL,
                 .StdFiltersNbr = 1U, .RxFifo0ElmtsNbr = 16U,
                 .TxFifoQueueElmtsNbr = 8U},
        .State = HAL_FDCAN_STATE_READY
    };
    hfdcan1 = can;
    hfdcan2 = can;
    hfdcan1.Instance = &can_instances[0];
    hfdcan2.Instance = &can_instances[1];
    for (unsigned i = 0U; i < 2U; ++i) { can_io[i].free_slots = 8U; }
    UART_HandleTypeDef *const uarts[] = {
        &huart1, &huart2, &huart3, &huart4, &huart5,
        &huart6, &huart7, &huart8, &huart9, &huart10
    };
    for (unsigned i = 0; i < 10U; ++i) {
        *uarts[i] = (UART_HandleTypeDef){
            .Instance = &uart_instances[i],
            .Init = {UART_WORDLENGTH_8B, UART_PARITY_NONE, UART_MODE_TX_RX, 115200U, UART_STOPBITS_1},
            .gState = HAL_UART_STATE_READY, .RxState = HAL_UART_STATE_READY
        };
    }
    hspi6 = (SPI_HandleTypeDef){
        .Instance = SPI6,
        .Init = {SPI_MODE_MASTER, SPI_DIRECTION_2LINES_TXONLY, SPI_DATASIZE_8BIT,
                 SPI_FIRSTBIT_MSB, SPI_POLARITY_LOW, SPI_PHASE_1EDGE,
                 SPI_BAUDRATEPRESCALER_8},
        .State = HAL_SPI_STATE_READY
    };
    test_gpioa.IDR = KEY__INPUT_Pin;
}

static void queue_motor_feedback(FDCAN_HandleTypeDef *bus, uint32_t master_id,
                                  const uint8_t data[8], uint32_t len)
{
    FakeCAN_IO *io = &can_io[fake_can_index(bus)];
    io->custom_rx = true;
    memcpy(io->rx_data, data, 8U);
    io->rx_header = (FDCAN_RxHeaderTypeDef){
        .Identifier = master_id, .IdType = FDCAN_STANDARD_ID,
        .RxFrameType = FDCAN_DATA_FRAME, .DataLength = len,
        .FDFormat = FDCAN_CLASSIC_CAN
    };
    io->rx_pending = 1U;
}

static void inject_motor_feedback(FDCAN_HandleTypeDef *bus, uint32_t master_id,
                                   const uint8_t data[8], uint32_t len)
{
    queue_motor_feedback(bus, master_id, data, len);
    test_ipsr = 16U;
    HAL_FDCAN_RxFifo0Callback(bus, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    test_ipsr = 0U;
}

static void test_dm_motor(void)
{
    DM_Motor_Config_t config = {0};
    DM_Motor_t *m4310 = NULL, *m4340 = NULL, *other = NULL;
    DM_Motor_Feedback_t feedback = {.received_at_ms = 123U};
    assert(DM_Motor_DefaultConfig((DM_Motor_Model_t)0, &config) ==
           BSP_CAN_INVALID_ARG && config.pmax == 0.0f);
    assert(DM_Motor_DefaultConfig(DM_MOTOR_DM4310, NULL) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_DefaultConfig(DM_MOTOR_DM4310, &config) == BSP_CAN_OK);
    assert(config.pmax == 12.5f && config.vmax == 30.0f && config.tmax == 10.0f);
    assert(config.bus == NULL && config.can_id == 0U && config.master_id == 0U);
    assert(DM_Motor_Init(&config, &m4310) == BSP_CAN_INVALID_ARG && m4310 == NULL);
    config.bus = &hfdcan1;
    config.can_id = 3U;
    config.master_id = 0x13U;
    assert(DM_Motor_Init(&config, &m4310) == BSP_CAN_NOT_READY && m4310 == NULL);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    assert(DM_Motor_Init(NULL, &m4310) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_Init(&config, NULL) == BSP_CAN_INVALID_ARG);

    DM_Motor_Config_t invalid = config;
    invalid.can_id = 16U;
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    invalid = config;
    invalid.protocol = (DM_Motor_Protocol_t)2;
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    invalid = config;
    invalid.master_id = 0U;
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    invalid.master_id = 0x7FFU;
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    invalid.master_id = invalid.can_id;
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    const float invalid_maxima[] = {0.0f, -1.0f, NAN, INFINITY, FLT_TRUE_MIN};
    for (unsigned i = 0U; i < sizeof(invalid_maxima) / sizeof(invalid_maxima[0]); ++i) {
        invalid = config;
        invalid.pmax = invalid_maxima[i];
        assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
        invalid = config;
        invalid.vmax = invalid_maxima[i];
        assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
        invalid = config;
        invalid.tmax = invalid_maxima[i];
        assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_INVALID_ARG);
    }
    test_ipsr = 1U;
    assert(DM_Motor_Init(&config, &m4310) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    assert(DM_Motor_Init(&config, &m4310) == BSP_CAN_OK && m4310 != NULL);
    assert(can_tx_calls == 0U && gpio_high_writes == 0U);
    assert(DM_Motor_Init(&config, &other) == BSP_CAN_BUSY && other == NULL);
    invalid = config;
    invalid.can_id = 4U; /* 相同 Master ID 被拒绝。 */
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_BUSY);
    invalid.master_id = 3U; /* RX ID 与另一个电机的 TX ID 冲突。 */
    assert(DM_Motor_Init(&invalid, &other) == BSP_CAN_BUSY);
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_NOT_READY);
    assert(feedback.received_at_ms == 123U);
    assert(DM_Motor_Enable(NULL) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_Disable(NULL) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_GetFeedback(NULL, &feedback) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_GetFeedback(m4310, NULL) == BSP_CAN_INVALID_ARG);

    assert(DM_Motor_DefaultConfig(DM_MOTOR_DM4340, &config) == BSP_CAN_OK);
    assert(config.pmax == 12.5f && config.vmax == 10.0f && config.tmax == 28.0f);
    config.bus = &hfdcan1;
    config.can_id = 1U;
    config.master_id = 0x11U;
    assert(DM_Motor_Init(&config, &m4340) == BSP_CAN_OK);
    config.tmax = 1.0f; /* Init 拷贝配置，后续修改不影响设备。 */
    assert(DM_Motor_Enable(m4310) == BSP_CAN_OK);
    assert(can_tx_header.Identifier == 3U && can_tx_header.DataLength == 8U);
    const uint8_t enable[8] = {255U, 255U, 255U, 255U, 255U, 255U, 255U, 252U};
    assert(memcmp(can_tx_data, enable, 8U) == 0);
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_NOT_READY);
    assert(DM_Motor_Disable(m4310) == BSP_CAN_OK && can_tx_data[7] == 0xFDU);
    assert(DM_Motor_ProbeReadOnly(&hfdcan1, 0U, false) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_ProbeReadOnly(&hfdcan1, 0x7FFU, true) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_ProbeReadOnly(&hfdcan1, 0x123U, true) == BSP_CAN_OK);
    const uint8_t probe_wire[8] = {0x23U, 1U, 0x33U, 7U, 0U, 0U, 0U, 0U};
    assert(can_tx_header.Identifier == 0x7FFU);
    assert(memcmp(can_tx_data, probe_wire, 8U) == 0);
    const DM_Motor_MIT_Command_t zero = {0};
    const uint8_t zero_wire[8] = {0x7FU, 0xFFU, 0x7FU, 0xF0U,
                                0U, 0U, 0x07U, 0xFFU};
    assert(DM_Motor_SendMIT(m4310, &zero) == BSP_CAN_OK);
    assert(memcmp(can_tx_data, zero_wire, 8U) == 0);
    assert(DM_Motor_SendTorque(m4310, 0.0f) == BSP_CAN_OK);
    assert(memcmp(can_tx_data, zero_wire, 8U) == 0);
    assert(DM_Motor_SendTorque(m4310, 5.0f) == BSP_CAN_OK);
    assert((((can_tx_data[6] & 0x0FU) << 8) | can_tx_data[7]) == 0xBFFU);
    assert(DM_Motor_SendTorque(m4340, 14.0f) == BSP_CAN_OK);
    assert(can_tx_header.Identifier == 1U);
    assert((((can_tx_data[6] & 0x0FU) << 8) | can_tx_data[7]) == 0xBFFU);

    const DM_Motor_MIT_Command_t intermediate = {
        .position_rad = -6.25f, .velocity_rad_s = 15.0f,
        .kp = 250.0f, .kd = 2.5f, .torque_ff_nm = 5.0f
    };
    const uint8_t intermediate_wire[8] = {
        0x3FU, 0xFFU, 0xBFU, 0xF7U, 0xFFU, 0x7FU, 0xFBU, 0xFFU
    };
    assert(DM_Motor_SendMIT(m4310, &intermediate) == BSP_CAN_OK);
    assert(memcmp(can_tx_data, intermediate_wire, 8U) == 0);

    DM_Motor_MIT_Command_t command = {
        .position_rad = -12.5f, .velocity_rad_s = -30.0f, .torque_ff_nm = -10.0f
    };
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_OK);
    const uint8_t minimum_wire[8] = {0};
    assert(memcmp(can_tx_data, minimum_wire, 8U) == 0);
    command = (DM_Motor_MIT_Command_t){
        .position_rad = 12.5f, .velocity_rad_s = 30.0f,
        .kp = 500.0f, .kd = 5.0f, .torque_ff_nm = 10.0f
    };
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_OK);
    for (unsigned i = 0U; i < 8U; ++i) { assert(can_tx_data[i] == 0xFFU); }
    for (unsigned reserved = 0xFFBU; reserved <= 0xFFEU; ++reserved) {
        /* 量化格中点构造，确保击中清故障/使能/失能/设零保留码。 */
        command.torque_ff_nm = (((float)reserved + 0.5f) / 4095.0f * 2.0f - 1.0f) *
                               10.0f;
        const unsigned before = can_tx_calls;
        assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
        assert(can_tx_calls == before);
    }
    const unsigned before_invalid = can_tx_calls;
    const float invalid_inputs[] = {NAN, INFINITY, -INFINITY, FLT_MAX, -FLT_MAX};
    for (unsigned i = 0U; i < sizeof(invalid_inputs) / sizeof(invalid_inputs[0]); ++i) {
        for (unsigned field = 0U; field < 5U; ++field) {
            command = zero;
            switch (field) {
            case 0U: command.position_rad = invalid_inputs[i]; break;
            case 1U: command.velocity_rad_s = invalid_inputs[i]; break;
            case 2U: command.kp = invalid_inputs[i]; break;
            case 3U: command.kd = invalid_inputs[i]; break;
            default: command.torque_ff_nm = invalid_inputs[i]; break;
            }
            assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
        }
    }
    command = zero;
    command.kp = -0.01f;
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
    command.kp = 500.1f;
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
    command = zero;
    command.kd = -0.01f;
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
    command.kd = 5.01f;
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
    command = zero;
    command.position_rad = 12.51f;
    assert(DM_Motor_SendMIT(m4310, &command) == BSP_CAN_INVALID_ARG);
    command = zero;
    command.velocity_rad_s = 10.01f;
    assert(DM_Motor_SendMIT(m4340, &command) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_SendTorque(m4310, 10.01f) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_SendTorque(m4340, 28.01f) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_SendMIT(m4310, NULL) == BSP_CAN_INVALID_ARG);
    assert(can_tx_calls == before_invalid);

    can_free = 0U;
    assert(DM_Motor_SendTorque(m4310, 0.1f) == BSP_CAN_BUSY);
    assert(DM_Motor_Disable(m4310) == BSP_CAN_BUSY);
    assert(can_tx_calls == before_invalid);
    can_free = 8U;
    can_protocol[0].BusOff = 1U;
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    assert(DM_Motor_Enable(m4310) == BSP_CAN_NOT_READY);
    assert(can_tx_calls == before_invalid);
    can_protocol[0].BusOff = 0U;
    hfdcan1.Instance->CCCR = 0U;
    can_tx_result = HAL_ERROR;
    assert(DM_Motor_SendTorque(m4310, 0.1f) == BSP_CAN_ERROR);
    can_tx_result = HAL_BUSY;
    assert(DM_Motor_Enable(m4310) == BSP_CAN_BUSY);
    can_tx_result = HAL_OK;

    const uint8_t reply4310[8] = {0x13U, 0U, 0U, 0xFFU, 0xF0U, 0U, 31U, 42U};
    inject_motor_feedback(&hfdcan2, 0x13U, reply4310, 8U); /* 错误总线。 */
    inject_motor_feedback(&hfdcan1, 0x12U, reply4310, 8U); /* 错误 Master ID。 */
    inject_motor_feedback(&hfdcan1, 0x11U, reply4310, 8U); /* 错误载荷 motor ID。 */
    inject_motor_feedback(&hfdcan1, 0x13U, reply4310, 7U); /* 截短帧。 */
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_NOT_READY);
    assert(DM_Motor_GetFeedback(m4340, &feedback) == BSP_CAN_NOT_READY);
    test_tick = UINT32_MAX - 2U;
    inject_motor_feedback(&hfdcan1, 0x13U, reply4310, 8U);
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_OK);
    assert(feedback.position_rad == -12.5f && feedback.velocity_rad_s == 30.0f);
    assert(feedback.torque_nm == -10.0f && feedback.state == 1U);
    assert(feedback.mos_temperature_c == 31U && feedback.rotor_temperature_c == 42U);
    assert(feedback.received_at_ms == UINT32_MAX - 2U && feedback.received_count == 1U);
    for (uint8_t rid = 21U; rid <= 23U; ++rid) {
        const uint8_t range_reply[8] = {3U, 0U, 0x33U, rid, 0U, 0U, 0x48U, 0x41U};
        inject_motor_feedback(&hfdcan1, 0x13U, range_reply, 8U);
        assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_OK);
        assert(feedback.position_rad == -12.5f && feedback.state == 1U);
        assert(feedback.received_count == 1U &&
               feedback.received_at_ms == UINT32_MAX - 2U);
    }
    const uint8_t master_reply[8] = {3U, 0U, 0x33U, 7U, 0x13U, 0U, 0U, 0U};
    inject_motor_feedback(&hfdcan1, 0x13U, master_reply, 8U);
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_OK);
    assert(feedback.received_count == 1U);
    const uint8_t reply4340[8] = {0xA1U, 0xFFU, 0xFFU, 0U, 0x0FU, 0xFFU, 55U, 66U};
    test_tick = 2U;
    inject_motor_feedback(&hfdcan1, 0x11U, reply4340, 8U);
    assert(DM_Motor_GetFeedback(m4340, &feedback) == BSP_CAN_OK);
    assert(feedback.position_rad == 12.5f && feedback.velocity_rad_s == -10.0f);
    assert(feedback.torque_nm == 28.0f && feedback.state == 0xAU);
    assert(feedback.received_at_ms == 2U && feedback.received_count == 1U);
    assert(DM_Motor_Disable(m4340) == BSP_CAN_OK);
    assert(DM_Motor_GetFeedback(m4340, &feedback) == BSP_CAN_OK && feedback.state == 0xAU);
    inject_motor_feedback(&hfdcan1, 0x13U, reply4310, 8U);
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_OK);
    assert(feedback.received_count == 2U && feedback.received_at_ms == 2U);
    test_primask = 1U;
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_OK && test_primask == 1U);
    test_primask = 0U;
    test_ipsr = 1U;
    const unsigned before_isr = can_tx_calls;
    assert(DM_Motor_GetFeedback(m4310, &feedback) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_SendTorque(m4310, 0.1f) == BSP_CAN_INVALID_ARG);
    assert(DM_Motor_Enable(m4310) == BSP_CAN_INVALID_ARG && can_tx_calls == before_isr);
    test_ipsr = 0U;

    /* 不同总线相同 ID 可独立存在；自定义读回量程按实例生效。 */
    assert(DM_Motor_DefaultConfig(DM_MOTOR_DM4310, &config) == BSP_CAN_OK);
    config.bus = &hfdcan2;
    config.can_id = 3U;
    config.master_id = 0x13U;
    config.pmax = 6.0f;
    config.vmax = 8.0f;
    config.tmax = 2.0f;
    assert(DM_Motor_Init(&config, &other) == BSP_CAN_OK);
    assert(DM_Motor_SendTorque(other, 1.0f) == BSP_CAN_OK);
    assert((((can_tx_data[6] & 0x0FU) << 8) | can_tx_data[7]) == 0xBFFU);
    inject_motor_feedback(&hfdcan2, 0x13U, reply4310, 8U);
    assert(DM_Motor_GetFeedback(other, &feedback) == BSP_CAN_OK);
    assert(feedback.position_rad == -6.0f && feedback.velocity_rad_s == 8.0f);
    assert(feedback.torque_nm == -2.0f);

    /* Historical screenshot: 02 64 F6 7F F8 00 1E 1B. Remap only motor ID;
     * codec raw decoding must not depend on a subsequently changed zero. */
    const uint8_t observed[8] = {0x01U, 0x64U, 0xF6U, 0x7FU, 0xF8U, 0x00U, 0x1EU, 0x1BU};
    inject_motor_feedback(&hfdcan1, 0x11U, observed, 8U);
    assert(DM_Motor_GetFeedback(m4340, &feedback) == BSP_CAN_OK);
    assert(fabsf(feedback.position_rad - (-2.640383005142212f)) < 1e-6f);
    assert(fabsf(feedback.velocity_rad_s + .00244200244f) < 1e-6f);
    assert(fabsf(feedback.torque_nm - .00683760684f) < 2e-6f);
    assert(feedback.state == 0U && feedback.mos_temperature_c == 30U &&
        feedback.rotor_temperature_c == 27U && feedback.received_count == 2U);

    /* 先耗尽共享 BSP 回调，再验证设备注册失败不占用实例/覆盖输出。 */
    for (unsigned id = 0x100U; id < 0x10DU; ++id) {
        assert(BSP_CAN_RegisterRxCallback(&hfdcan1, id, can_receive, &can_callbacks) ==
               BSP_CAN_OK);
    }
    config.can_id = 4U;
    config.master_id = 0x14U;
    DM_Motor_t *unchanged = other;
    assert(DM_Motor_Init(&config, &other) == BSP_CAN_NO_RESOURCE && other == unchanged);
    assert(DM_Motor_Init(&config, &other) == BSP_CAN_NO_RESOURCE && other == unchanged);
    assert(gpio_high_writes == 0U);
}

static void test_dm_motor_capacity(void)
{
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    DM_Motor_Config_t config;
    assert(DM_Motor_DefaultConfig(DM_MOTOR_DM4310, &config) == BSP_CAN_OK);
    DM_Motor_t *motor = NULL;
    for (unsigned i = 0U; i < 16U; ++i) {
        config.bus = i < 15U ? &hfdcan1 : &hfdcan2;
        config.can_id = (uint16_t)(i % 15U + 1U);
        config.master_id = (uint16_t)(config.can_id + 0x10U);
        assert(DM_Motor_Init(&config, &motor) == BSP_CAN_OK);
    }
    DM_Motor_t *unchanged = motor;
    config.can_id = 2U;
    config.master_id = 0x12U;
    assert(DM_Motor_Init(&config, &motor) == BSP_CAN_NO_RESOURCE && motor == unchanged);
    assert(DM_Motor_SendTorque(motor, 0.1f) == BSP_CAN_OK);
    assert(can_tx_calls == 1U && can_tx_header.Identifier == 1U);
}



static void receive_during_can_start(FDCAN_HandleTypeDef *bus)
{
    const uint8_t data[8] = {0};
    queue_motor_feedback(bus, 0x77U, data, 8U);
    bus->Instance->IR |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    test_ipsr = 16U;
    BSP_CAN_IRQHandler(bus);
    test_ipsr = 0U;
}

static void test_can_startup(void)
{
    /* NVIC entry before this instance has been initialized must not touch HAL. */
    FDCAN_GlobalTypeDef *const instance = hfdcan1.Instance;
    hfdcan1.Instance = NULL;
    BSP_CAN_IRQHandler(&hfdcan1);
    assert(can_irq_calls == 0U);
    hfdcan1.Instance = instance;

    can_start_hook = receive_during_can_start;
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    can_start_hook = NULL;
    for (unsigned i = 0U; i < 2U; ++i) {
        /* A frame received inside HAL_Start is drained even before registration.
         * Unknown IDs may be discarded; leaving a cleared IRQ with queued RX
         * could otherwise strand the FIFO until another message arrives. */
        assert(can_io[i].rx_pending == 0U && can_io[i].rx_calls == 1U);
    }
    assert(can_irq_calls == 2U && can_tx_calls == 0U && gpio_writes == 0U);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_BUSY);
    FDCAN_HandleTypeDef unknown = {0};
    const BSP_CAN_Frame_t frame = {.id = 1U, .len = 8U};
    assert(BSP_CAN_Init(&unknown) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_Send(&unknown, &frame) == BSP_CAN_INVALID_ARG);
    BSP_CAN_Diagnostics_t diag;
    assert(BSP_CAN_GetDiagnostics(&unknown, &diag) == BSP_CAN_INVALID_ARG);
    BSP_CAN_IRQHandler(&unknown);
    assert(can_irq_calls == 2U);
}

static void test_can_start_failure(void)
{
    const BSP_CAN_Frame_t frame = {.id = 1U, .len = 8U};
    can_start_result = HAL_ERROR;
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_ERROR);
    assert(hfdcan1.Instance->IE == 0U && can_deactivate_calls == 1U);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    BSP_CAN_Diagnostics_t diag;
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_NOT_READY);
    const unsigned starts = can_start_calls;
    can_start_result = HAL_OK;
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_ERROR && can_start_calls == starts);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    assert(BSP_CAN_Send(&hfdcan2, &frame) == BSP_CAN_OK);
}

static void test_can_motor_config(void)
{
    DM_Motor_t *motors[MOTOR_CAN_JOINT_COUNT] = {0};
    DM_Motor_Feedback_t feedback;
    assert(MOTOR_CAN_JOINT_COUNT == 5U && FIVE_AXES == 5);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        const DM_Motor_Config_t *cfg = &g_motor_can_config[i];
        const bool large = i < 2U;
        assert(cfg->can_id == i + 1U && cfg->master_id == 0x11U + i);
        assert(cfg->bus == (large ? &hfdcan2 : &hfdcan1));
        assert(cfg->model == (large ? DM_MOTOR_DM4340 : DM_MOTOR_DM4310));
        assert(cfg->protocol == (i == 2U ? DM_MOTOR_PROTOCOL_LEGACY_MIT :
                                          DM_MOTOR_PROTOCOL_V13));
        assert(cfg->pmax == 12.5f && cfg->vmax == (large ? 10.0f : 30.0f));
        assert(cfg->tmax == (large ? 28.0f : 10.0f));
        assert(DM_Motor_Init(cfg, &motors[i]) == BSP_CAN_OK);
    }
    assert(can_tx_calls == 0U && gpio_writes == 0U);
    assert(FiveControl_Init(motors, test_tick));
    assert(can_tx_calls == 0U);
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        const DM_Motor_Config_t *cfg = &g_motor_can_config[i];
        FakeCAN_IO *io = &can_io[fake_can_index(cfg->bus)];
        const unsigned sent = io->tx_calls;
        if (i == 2U) {
            assert(DM_Motor_RequestFeedback(motors[i]) == BSP_CAN_NOT_READY);
            assert(DM_Motor_ProbeReadOnly(cfg->bus, cfg->can_id, false) ==
                   BSP_CAN_NOT_READY);
            assert(DM_Motor_ProbeReadOnly(cfg->bus, cfg->can_id, true) ==
                   BSP_CAN_NOT_READY);
            assert(io->tx_calls == sent); /* No implicit FD/MIT/FC fallback. */
        } else {
            assert(DM_Motor_RequestFeedback(motors[i]) == BSP_CAN_OK);
            assert(io->tx_calls == sent + 1U && io->tx_header.Identifier == 0x7FFU);
            assert(io->tx_data[0] == cfg->can_id && io->tx_data[2] == 0xCCU);
        }
        uint8_t reply[8] = {(uint8_t)cfg->can_id, 0xFFU, 0xFFU,
                            0xFFU, 0xFFU, 0xFFU, (uint8_t)(30U + i), 25U};
        inject_motor_feedback(cfg->bus == &hfdcan1 ? &hfdcan2 : &hfdcan1,
                              cfg->master_id, reply, 8U);
        assert(DM_Motor_GetFeedback(motors[i], &feedback) == BSP_CAN_NOT_READY);
        reply[0] = (uint8_t)(cfg->can_id + 1U); /* Same RX ID, wrong motor nibble. */
        inject_motor_feedback(cfg->bus, cfg->master_id, reply, 8U);
        assert(DM_Motor_GetFeedback(motors[i], &feedback) == BSP_CAN_NOT_READY);
        reply[0] = (uint8_t)cfg->can_id;
        test_tick = 1010U + i;
        inject_motor_feedback(cfg->bus, cfg->master_id, reply, 8U);
        assert(DM_Motor_GetFeedback(motors[i], &feedback) == BSP_CAN_OK);
        assert(feedback.position_rad == cfg->pmax && feedback.velocity_rad_s == cfg->vmax);
        assert(feedback.torque_nm == cfg->tmax && feedback.mos_temperature_c == 30U + i);
        assert(feedback.received_count == 1U && feedback.received_at_ms == test_tick);
        assert(motor_query_calls[i + 1U] == (i == 2U ? 0U : 1U));
        assert(motor_enable_calls[i + 1U] == 0U && motor_disable_calls[i + 1U] == 0U &&
               motor_mit_calls[i + 1U] == 0U);
    }
    assert(can_io[0].tx_calls == 2U && can_io[1].tx_calls == 2U);
    /* Actual legacy J3 reply captured on 2026-11-14, not a CC response. */
    const uint8_t legacy_reply[8] = {3U, 0x84U, 0xC6U, 0x7FU,
                                    0xF8U, 1U, 27U, 26U};
    inject_motor_feedback(&hfdcan1, 0x13U, legacy_reply, 8U);
    assert(DM_Motor_GetFeedback(motors[2], &feedback) == BSP_CAN_OK);
    assert(feedback.state == 0U && feedback.received_count == 2U);
    assert(fabsf(feedback.position_rad -
        ((float)0x84C6U / 65535.0f * 2.0f - 1.0f) * 12.5f) < 1e-6f);
    assert(feedback.mos_temperature_c == 27U && feedback.rotor_temperature_c == 26U);
    /* A valid legacy position can resemble a V13 parameter reply. */
    const uint8_t legacy_collision[8] = {3U, 0U, 0x33U, 21U, 0U, 0U, 27U, 26U};
    inject_motor_feedback(&hfdcan1, 0x13U, legacy_collision, 8U);
    assert(DM_Motor_GetFeedback(motors[2], &feedback) == BSP_CAN_OK);
    assert(feedback.received_count == 3U);
    const DM_Motor_MIT_Command_t zero = {0};
    const uint8_t zero_wire[8] = {0x7FU, 0xFFU, 0x7FU, 0xF0U, 0U, 0U, 7U, 0xFFU};
    assert(DM_Motor_SendMIT(motors[2], &zero) == BSP_CAN_OK);
    assert(can_tx_header.Identifier == 3U && memcmp(can_tx_data, zero_wire, 8U) == 0);
    assert(DM_Motor_Enable(motors[2]) == BSP_CAN_OK && can_tx_data[7] == 0xFCU);
    assert(DM_Motor_Disable(motors[2]) == BSP_CAN_OK && can_tx_data[7] == 0xFDU);
    for (unsigned i = 0U; i < 7U; ++i) { assert(can_tx_data[i] == 0xFFU); }
    assert(DM_Motor_GetFeedback(motors[2], &feedback) == BSP_CAN_OK &&
           feedback.received_count == 3U); /* Submission cannot fabricate feedback. */
    can_io[0].free_slots = 0U;
    assert(DM_Motor_SendMIT(motors[2], &zero) == BSP_CAN_BUSY);
    assert(DM_Motor_Disable(motors[2]) == BSP_CAN_BUSY);
    can_io[0].free_slots = 8U;
    const unsigned before_other_bus = can_io[1].tx_calls;
    assert(DM_Motor_ProbeReadOnly(&hfdcan2, 3U, false) == BSP_CAN_OK);
    assert(can_io[1].tx_calls == before_other_bus + 1U); /* Protocol is per bus/device. */
    assert(gpio_writes == 0U && test_primask == 0U);
}

static void test_can_five_axis(void)
{
    DM_Motor_t *motors[5] = {0};
    DM_Motor_Config_t configs[5];
    DM_Motor_Feedback_t feedback;
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    /* Studio grouping; IDs below are test fixtures, NOT physical assignments.
     * J1/J2 on CAN2 and J3/J4/J5 on CAN1 deliberately reuse IDs across buses. */
    for (unsigned i = 0U; i < 5U; ++i) {
        const bool large = i < 2U;
        assert(DM_Motor_DefaultConfig(large ? DM_MOTOR_DM4340 : DM_MOTOR_DM4310,
                                     &configs[i]) == BSP_CAN_OK);
        configs[i].bus = large ? &hfdcan2 : &hfdcan1;
        configs[i].can_id = (uint16_t)(large ? i + 1U : i - 1U);
        configs[i].master_id = (uint16_t)(configs[i].can_id + 0x10U);
        assert(DM_Motor_Init(&configs[i], &motors[i]) == BSP_CAN_OK);
        assert(DM_Motor_GetFeedback(motors[i], &feedback) == BSP_CAN_NOT_READY);
    }
    assert(can_tx_calls == 0U && gpio_writes == 0U);
    DM_Motor_t *unchanged = motors[4];
    assert(DM_Motor_Init(&configs[0], &unchanged) == BSP_CAN_BUSY);
    assert(unchanged == motors[4]);

    for (unsigned i = 0U; i < 5U; ++i) {
        FakeCAN_IO *io = &can_io[fake_can_index(configs[i].bus)];
        const unsigned sent = io->tx_calls;
        assert(DM_Motor_RequestFeedback(motors[i]) == BSP_CAN_OK);
        assert(io->tx_calls == sent + 1U && io->tx_header.Identifier == 0x7FFU);
        assert(io->tx_data[0] == configs[i].can_id && io->tx_data[2] == 0xCCU);
        assert(DM_Motor_SendTorque(motors[i], 0.5f) == BSP_CAN_OK);
        assert(io->tx_calls == sent + 2U &&
               io->tx_header.Identifier == configs[i].can_id);
        const uint16_t torque = (uint16_t)(((io->tx_data[6] & 0x0FU) << 8) |
                                          io->tx_data[7]);
        const uint16_t expected = (uint16_t)((0.5f / configs[i].tmax + 1.0f) *
                                            0.5f * 4095.0f);
        assert(torque == expected); /* Each model keeps its own protocol scale. */
    }
    assert(can_io[0].tx_calls == 6U && can_io[1].tx_calls == 4U);

    const uint8_t first[8] = {1U, 0U, 0U, 0xFFU, 0xF0U, 0U, 31U, 41U};
    queue_motor_feedback(&hfdcan1, 0x11U, first, 8U);
    queue_motor_feedback(&hfdcan2, 0x11U, first, 8U);
    hfdcan1.Instance->IR |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    test_tick = 100U;
    test_ipsr = 16U;
    BSP_CAN_IRQHandler(&hfdcan1);
    test_ipsr = 0U;
    assert(can_io[0].rx_pending == 0U && can_io[1].rx_pending == 1U);
    assert(DM_Motor_GetFeedback(motors[2], &feedback) == BSP_CAN_OK);
    assert(feedback.velocity_rad_s == 30.0f && feedback.torque_nm == -10.0f);
    assert(feedback.received_at_ms == 100U && feedback.received_count == 1U);
    assert(DM_Motor_GetFeedback(motors[0], &feedback) == BSP_CAN_NOT_READY);
    hfdcan2.Instance->IR |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    test_tick = 101U;
    test_ipsr = 16U;
    BSP_CAN_IRQHandler(&hfdcan2);
    test_ipsr = 0U;
    assert(DM_Motor_GetFeedback(motors[0], &feedback) == BSP_CAN_OK);
    assert(feedback.velocity_rad_s == 10.0f && feedback.torque_nm == -28.0f);
    assert(feedback.received_at_ms == 101U && feedback.received_count == 1U);
    for (unsigned i = 0U; i < 5U; ++i) {
        uint8_t reply[8] = {0};
        memcpy(reply, first, 8U);
        reply[0] = (uint8_t)configs[i].can_id;
        reply[6] = (uint8_t)(40U + i);
        test_tick = 200U + i;
        inject_motor_feedback(configs[i].bus, configs[i].master_id, reply, 8U);
        assert(DM_Motor_GetFeedback(motors[i], &feedback) == BSP_CAN_OK);
        assert(feedback.mos_temperature_c == 40U + i &&
               feedback.received_at_ms == 200U + i);
    }

    /* Hardware FIFO pressure on either bus must not block its neighbour. */
    can_io[0].free_slots = 0U;
    unsigned sent = can_io[0].tx_calls;
    assert(DM_Motor_RequestFeedback(motors[2]) == BSP_CAN_BUSY);
    assert(can_io[0].tx_calls == sent);
    assert(DM_Motor_RequestFeedback(motors[0]) == BSP_CAN_OK);
    can_io[0].free_slots = 8U;
    can_io[1].free_slots = 0U;
    assert(DM_Motor_RequestFeedback(motors[0]) == BSP_CAN_BUSY);
    assert(DM_Motor_RequestFeedback(motors[2]) == BSP_CAN_OK);
    can_io[1].free_slots = 8U;

    /* Recover CAN2 while CAN1 still has an unread feedback frame. */
    queue_motor_feedback(&hfdcan1, 0x11U, first, 8U);
    queue_motor_feedback(&hfdcan2, 0x11U, first, 8U);
    can_protocol[1].BusOff = 1U;
    can_protocol[1].LastErrorCode = 5U;
    hfdcan2.Instance->CCCR = FDCAN_CCCR_INIT;
    hfdcan2.Instance->TXBRP = 3U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan2) == BSP_CAN_BUSY);
    assert(can_io[1].rx_pending == 0U && can_io[0].rx_pending == 1U);
    assert(DM_Motor_RequestFeedback(motors[0]) == BSP_CAN_NOT_READY);
    assert(DM_Motor_RequestFeedback(motors[2]) == BSP_CAN_OK);
    hfdcan1.Instance->IR |= FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    test_ipsr = 16U;
    BSP_CAN_IRQHandler(&hfdcan1);
    test_ipsr = 0U;
    assert(can_io[0].rx_pending == 0U);
    can_io[0].free_slots = 0U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan2) == BSP_CAN_BUSY);
    assert(BSP_CAN_RecoverBusOff(&hfdcan2) == BSP_CAN_BUSY);
    can_protocol[1].BusOff = 0U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan2) == BSP_CAN_OK);
    assert(can_io[0].free_slots == 0U);
    assert(DM_Motor_RequestFeedback(motors[2]) == BSP_CAN_BUSY);
    assert(DM_Motor_RequestFeedback(motors[0]) == BSP_CAN_OK);
    BSP_CAN_Diagnostics_t diag;
    assert(BSP_CAN_GetDiagnostics(&hfdcan2, &diag) == BSP_CAN_OK);
    assert(diag.bus_off_valid && diag.first.lec == 5U);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(!diag.first_valid && !diag.bus_off_valid);
    for (unsigned i = 0U; i < 16U; ++i) {
        assert(motor_enable_calls[i] == 0U && motor_disable_calls[i] == 0U);
    }
    assert(gpio_writes == 0U && test_primask == 0U);
}

static void test_can_diagnostics(void)
{
    BSP_CAN_Diagnostics_t diag, saved;
    const BSP_CAN_Frame_t frame = {.id = 1U, .len = 8U};
    assert(BSP_CAN_GetDiagnostics(NULL, &diag) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, NULL) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(!diag.first_valid && !diag.bus_off_valid && !diag.error_observations);
    BSP_CAN_IRQHandler(NULL);
    assert(can_irq_calls == 0U);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(can_protocol_reads == 0U && can_counter_reads == 0U);

    test_tick = UINT32_MAX - 5U;
    hfdcan1.Instance->IR = FDCAN_IT_ARB_PROTOCOL_ERROR;
    hfdcan1.Instance->TXBRP = 3U;
    can_protocol[0].LastErrorCode = 3U; /* ACK error, before Bus-Off. */
    can_counters[0] = (FDCAN_ErrorCountersTypeDef){.TxErrorCnt = 8U, .ErrorLogging = 1U};
    const unsigned sent = can_tx_calls;
    test_ipsr = 16U;
    BSP_CAN_IRQHandler(&hfdcan1);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    assert(test_primask == 0U && hfdcan1.Instance->IR == 0U);
    assert(can_irq_calls == 1U && can_tx_calls == sent);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(diag.first_valid && !diag.bus_off_valid && diag.error_observations == 1U);
    assert(diag.first.timestamp_ms == UINT32_MAX - 5U);
    assert(diag.first.lec == 3U && diag.first.dlec == 0U); /* No earlier PSR read. */
    assert(diag.first.tx_errors == 8U && diag.first.logged_errors == 1U);
    assert(diag.first.counters_valid && diag.first.tx_pending_mask == 3U);
    assert(diag.first.interrupts == FDCAN_IT_ARB_PROTOCOL_ERROR);
    saved = diag;
    const unsigned reads = can_protocol_reads, counters = can_counter_reads;
    test_primask = 1U;
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(test_primask == 1U);
    test_primask = 0U;
    assert(can_protocol_reads == reads && can_counter_reads == counters);

    test_tick = 3U; /* Tick rollover must not invalidate an immutable record. */
    can_protocol[0].LastErrorCode = 5U;
    can_counters[0].TxErrorCnt = 16U;
    hfdcan1.Instance->IR = FDCAN_IT_ARB_PROTOCOL_ERROR;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK); /* Same send policy. */
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(memcmp(&diag.first, &saved.first, sizeof(diag.first)) == 0);
    assert(diag.latest.lec == 5U && diag.latest.timestamp_ms == 3U);
    assert(diag.latest.tx_errors == 16U &&
           diag.latest.interrupts == FDCAN_IT_ARB_PROTOCOL_ERROR);
    BSP_CAN_IRQHandler(&hfdcan1); /* Let HAL acknowledge the pending interrupt. */

    test_tick = 8U;
    can_protocol[0].BusOff = 1U;
    can_protocol[0].ErrorPassive = can_protocol[0].Warning = 1U;
    can_protocol[0].LastErrorCode = 5U;
    can_counters[0].TxErrorCnt = 248U;
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    hfdcan1.Instance->IR = FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE;
    hfdcan1.Instance->TXBRP = 0xC3U;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(diag.bus_off_valid && diag.bus_off.timestamp_ms == 8U);
    assert(diag.bus_off.lec == 5U && diag.bus_off.tx_errors == 248U);
    assert(diag.bus_off.error_passive && diag.bus_off.warning);
    assert(diag.bus_off.tx_pending_mask == 0xC3U);
    assert(memcmp(&diag.first, &saved.first, sizeof(diag.first)) == 0);
    saved = diag;
    BSP_CAN_IRQHandler(&hfdcan1); /* LEC now 7; don't overwrite the BO snapshot. */
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(memcmp(&diag.bus_off, &saved.bus_off, sizeof(diag.bus_off)) == 0);
    const uint32_t observations = diag.error_observations;
    test_tick = 500U;
    for (unsigned i = 0U; i < 10U; ++i) {
        assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    }
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(diag.error_observations == observations);
    assert(can_tx_calls == sent + 1U && can_start_calls == 1U);
    assert(hfdcan1.Instance->TXBRP == 0xC3U); /* No cancel, recovery or old replay. */
    assert(hfdcan1.Instance->CCCR == FDCAN_CCCR_INIT);

    /* Reset the simulated bus only; diagnostic evidence must stay immutable. */
    hfdcan1.Instance->CCCR = 0U;
    can_protocol[0].BusOff = 0U;
    can_protocol[0].LastErrorCode = 4U;
    hfdcan1.Instance->IR = FDCAN_IT_ARB_PROTOCOL_ERROR;
    can_counter_result = HAL_ERROR;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(diag.latest.lec == 4U && !diag.latest.counters_valid);
    can_counter_result = HAL_OK;
    can_protocol_result = HAL_ERROR;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_ERROR);
    can_protocol_result = HAL_OK;
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(memcmp(&diag.first, &saved.first, sizeof(diag.first)) == 0);
    assert(memcmp(&diag.bus_off, &saved.bus_off, sizeof(diag.bus_off)) == 0);

    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    assert(BSP_CAN_GetDiagnostics(&hfdcan2, &diag) == BSP_CAN_OK);
    assert(!diag.first_valid && !diag.bus_off_valid);
    can_protocol[1].LastErrorCode = 6U;
    hfdcan2.Instance->IR = FDCAN_IT_ARB_PROTOCOL_ERROR;
    BSP_CAN_IRQHandler(&hfdcan2);
    assert(BSP_CAN_GetDiagnostics(&hfdcan2, &diag) == BSP_CAN_OK);
    assert(diag.first_valid && diag.first.lec == 6U);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &diag) == BSP_CAN_OK);
    assert(diag.first.lec == 3U && diag.bus_off_valid);
    assert(test_primask == 0U && motor_enable_calls[1] == 0U);
}

static void test_can_recovery(void)
{
    const BSP_CAN_Frame_t frame = {.id = 1U, .len = 8U};
    BSP_CAN_Diagnostics_t before, after;
    assert(BSP_CAN_RecoverBusOff(NULL) == BSP_CAN_INVALID_ARG);
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_Init(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Init(&hfdcan2) == BSP_CAN_OK);
    assert(BSP_CAN_RegisterRxCallback(&hfdcan1, 0x13U, can_receive, &can_callbacks)
           == BSP_CAN_OK);
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_NOT_READY);
    assert(can_protocol_reads == 0U && can_stop_calls == 0U);
    test_ipsr = 1U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_INVALID_ARG);
    test_ipsr = 0U;
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_NOT_READY);
    assert(can_stop_calls == 0U); /* INIT alone is not a recoverable Bus-Off. */
    can_protocol[0].BusOff = 1U;
    hfdcan1.Instance->CCCR |= FDCAN_CCCR_CSA;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_ERROR);
    assert(can_stop_calls == 0U); /* Never enter HAL's sleep wait. */
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    hfdcan1.Instance->TXBRP = 0x81U;
    can_rx_pending = 3U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_BUSY);
    assert(can_stop_calls == 1U && can_start_calls == 2U);
    assert(hfdcan1.Instance->TXBRP == 0U && can_rx_pending == 0U);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &before) == BSP_CAN_OK);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_Send(&hfdcan2, &frame) == BSP_CAN_OK);
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_BUSY);
    assert(can_start_calls == 3U);
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_BUSY);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    can_protocol[0].BusOff = 0U;
    can_free = 0U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_BUSY);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    can_free = 8U;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_OK);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_OK);
    assert(BSP_CAN_GetDiagnostics(&hfdcan1, &after) == BSP_CAN_OK);
    assert(memcmp(&before.first, &after.first, sizeof(before.first)) == 0);
    assert(memcmp(&before.bus_off, &after.bus_off, sizeof(before.bus_off)) == 0);
    can_rx_header = (FDCAN_RxHeaderTypeDef){.Identifier = 0x13U, .DataLength = 8U};
    can_rx_pending = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(can_callbacks == 1U); /* Recovery did not unregister Device RX. */
    can_protocol[0].BusOff = 1U;
    hfdcan1.Instance->CCCR = FDCAN_CCCR_INIT;
    can_stop_result = HAL_ERROR;
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_ERROR);
    assert(BSP_CAN_Send(&hfdcan1, &frame) == BSP_CAN_NOT_READY);
    assert(BSP_CAN_RecoverBusOff(&hfdcan1) == BSP_CAN_ERROR);
    assert(can_start_calls == 3U && motor_enable_calls[1] == 0U);
    assert(test_primask == 0U && hfdcan2.Instance->CCCR == 0U);
}

#include "five_mit_checks.inc"
#include "five_drag_checks.inc"

int main(int argc, char **argv)
{
    initialize_fake_hardware();
    if (argc==2 && strcmp(argv[1],"five-mit")==0) { test_five_mit(); return 0; }
    if (argc==2 && strcmp(argv[1],"five-control")==0) { test_control(); return 0; }
    if (argc==2 && strcmp(argv[1],"five-key")==0) { test_key_modes(); return 0; }
    if (argc==2 && strcmp(argv[1],"five-drag")==0) { test_drag_control(); return 0; }
#ifdef YLAMP_J3_REVERSE_TEST
    if (argc==2 && strcmp(argv[1],"five-j3-reverse")==0) { test_j3_reverse(); return 0; }
#endif
    if (argc==2 && strcmp(argv[1],"five-motion-admission")==0) { test_motion_admission(); return 0; }
    if (argc==2 && strcmp(argv[1],"five-smooth-control")==0) { test_smooth_control(); return 0; }
    if (argc==2 && strcmp(argv[1],"five-cold")==0) { test_cold(false); return 0; }
    if (argc==2 && strcmp(argv[1],"five-cold-silent")==0) { test_cold(true); return 0; }
    if (argc==2 && strcmp(argv[1],"five-j3-read")==0) { test_j3_read("ok"); return 0; }
    if (argc==2 && strcmp(argv[1],"five-j3-read-timeout")==0) { test_j3_read("timeout"); return 0; }
    if (argc==2 && strcmp(argv[1],"five-j3-read-cancel")==0) { test_j3_read("cancel"); return 0; }
    if (argc==2 && strcmp(argv[1],"five-j3-read-tx")==0) { test_j3_read("tx"); return 0; }
    if (argc==2 && strcmp(argv[1],"five-wire")==0) { return five_wire_stream(); }
    if (argc==2 && strcmp(argv[1],"can-start-failure")==0) { test_can_start_failure(); return 0; }
    if (argc == 2 && strcmp(argv[1], "can-motor-config") == 0) {
        test_can_motor_config();
        puts("PASS: production J1..J5 CAN bus/ID/ranges, feedback isolation and no enable");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "can-startup") == 0) {
        test_can_startup();
        puts("PASS: early IRQ, receive during HAL Start and unknown-bus rejection");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "can-five-axis") == 0) {
        test_can_five_axis();
        puts("PASS: five mixed motors, dual-bus IDs, FIFO pressure and recovery isolation");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "can-recovery") == 0) {
        test_can_recovery();
        puts("PASS: explicit bounded CAN recovery gates, FIFO flush and immutable evidence");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "can-diagnostics") == 0) {
        test_can_diagnostics();
        puts("PASS: immutable first CAN error / Bus-Off evidence, IRQ and send sampling");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "dm-motor-full") == 0) {
        test_dm_motor_capacity();
        puts("PASS: DM motor static capacity and existing instance preservation");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "dm-motor") == 0) {
        test_dm_motor();
        puts("PASS: DM4310 / DM4340 MIT, feedback and transport failures");
        return 0;
    }
    if (argc == 2 && strcmp(argv[1], "key-boot") == 0) {
        bool pressed = false;
        test_gpioa.IDR = 0U;
        assert(BSP_KEY_Init(20U) == HAL_OK);
        assert(BSP_KEY_GetPressed(&pressed) == HAL_OK && pressed);
        assert(BSP_KEY_Scan(100U) == BSP_KEY_EVENT_NONE);
        puts("PASS: key held at boot does not synthesize an edge");
        return 0;
    }
    if (argc != 1) { return 2; }
    test_can();
    test_key();
    test_power();
    test_uart();
    test_rgb();
    puts("PASS: CAN / UART / RGB / KEY / POWER behavioral checks");
    return 0;
}
