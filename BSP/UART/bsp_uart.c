#include "bsp_uart.h"

#include <stddef.h>
#include <string.h>

typedef struct {
    bool initialized;
    bool dma;
    uint8_t active_rx;
    uint16_t dma_rx_at;
    BSP_UART_RxCallback_t callback;
    void *context;
    volatile BSP_UART_Status_t status;
    uint8_t rx[2][BSP_UART_RX_CAPACITY];
    uint8_t tx[BSP_UART_TX_CAPACITY];
} UART_Port;

/* NOLOAD: RX only reads bytes already written by DMA; TX is copied before use.
 * The linker constrains this whole section to the MPU non-cacheable region. */
static struct {
    uint8_t rx[BSP_UART_DMA_RX_CAPACITY];
    uint8_t tx[BSP_UART_TX_CAPACITY];
} dma_buffer __attribute__((section(".dma_buffer"), aligned(32)));

static UART_Port ports[7];
static UART_HandleTypeDef *const handles[7] = {
    &huart1, &huart2, &huart3, &huart5, &huart6, &huart7, &huart10
};

static UART_Port *find_port(const UART_HandleTypeDef *uart)
{
    for (unsigned i = 0; i < 7U; ++i) {
        if (handles[i] == uart) { return &ports[i]; }
    }
    return NULL;
}

static HAL_StatusTypeDef start_rx(UART_HandleTypeDef *uart, UART_Port *port)
{
    /* 发布状态在启动之前，避免立即到来的错误中断被返回路径覆盖。 */
    port->status.rx_running = true;
    port->status.rx_rearm = HAL_OK;
    const HAL_StatusTypeDef result = port->dma ?
        HAL_UARTEx_ReceiveToIdle_DMA(uart, dma_buffer.rx, BSP_UART_DMA_RX_CAPACITY) :
        HAL_UARTEx_ReceiveToIdle_IT(uart, port->rx[port->active_rx], BSP_UART_RX_CAPACITY);
    if (result != HAL_OK) {
        port->status.rx_running = false;
        port->status.rx_rearm = result;
    }
    return result;
}

static HAL_StatusTypeDef initialize(UART_HandleTypeDef *uart,
    BSP_UART_RxCallback_t callback, void *context, bool dma)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || __get_IPSR() != 0U ||
        (!(uart->Init.WordLength == UART_WORDLENGTH_8B &&
           uart->Init.Parity == UART_PARITY_NONE) &&
         !(uart->Init.WordLength == UART_WORDLENGTH_9B &&
           uart->Init.Parity != UART_PARITY_NONE)) ||
        (callback != NULL && (uart->Init.Mode & UART_MODE_RX) == 0U)) {
        return HAL_ERROR;
    }
    if (port->initialized) { return HAL_BUSY; }
    if (uart->gState != HAL_UART_STATE_READY ||
        uart->RxState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    port->callback = callback;
    port->context = context;
    port->dma = dma;
    port->status.rx_rearm = HAL_OK;
    port->initialized = true;
    if (callback != NULL) {
        /* 失败后保留已绑定对象，后续必须显式 RestartRx，不重复注册覆盖。 */
        return start_rx(uart, port);
    }
    return HAL_OK;
}

HAL_StatusTypeDef BSP_UART_Init(UART_HandleTypeDef *uart,
                                BSP_UART_RxCallback_t callback, void *context)
{
    return initialize(uart, callback, context, false);
}

HAL_StatusTypeDef BSP_UART_InitDMA(UART_HandleTypeDef *uart,
                                   BSP_UART_RxCallback_t callback, void *context)
{
    if (uart != &huart1 || callback == NULL || uart->hdmarx == NULL ||
        uart->hdmatx == NULL || uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        uart->hdmatx->Init.Mode != DMA_NORMAL ||
        uart->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY ||
        uart->hdmatx->Init.Direction != DMA_MEMORY_TO_PERIPH) { return HAL_ERROR; }
    return initialize(uart, callback, context, true);
}

HAL_StatusTypeDef BSP_UART_Send(UART_HandleTypeDef *uart,
                                const uint8_t *data, uint16_t len)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || !port->initialized || data == NULL || len == 0U ||
        len > BSP_UART_TX_CAPACITY || __get_IPSR() != 0U ||
        (uart->Init.Mode & UART_MODE_TX) == 0U) {
        return HAL_ERROR;
    }
    if (port->status.tx_busy || uart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    uint8_t *const buffer = port->dma ? dma_buffer.tx : port->tx;
    memcpy(buffer, data, len);
    __DMB(); /* Publish the copied payload before DMA can fetch it. */
    port->status.tx_busy = true;
    const HAL_StatusTypeDef result = port->dma ?
        HAL_UART_Transmit_DMA(uart, buffer, len) :
        HAL_UART_Transmit_IT(uart, buffer, len);
    if (result != HAL_OK) { port->status.tx_busy = false; }
    return result;
}

HAL_StatusTypeDef BSP_UART_RestartRx(UART_HandleTypeDef *uart)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || !port->initialized || port->callback == NULL ||
        __get_IPSR() != 0U) {
        return HAL_ERROR;
    }
    /* DMA 故障后保持缓冲所有权，不在控制任务中等待 DMA 停止。 */
    if (port->dma || (uart->Instance->CR3 & USART_CR3_DMAR) != 0U) { return HAL_BUSY; }
    port->status.rx_running = false;
    const HAL_StatusTypeDef result = HAL_UART_AbortReceive(uart);
    port->status.rx_rearm = result;
    if (result != HAL_OK) { return result; }
    port->active_rx = 0U;
    port->status.rx_error = HAL_UART_ERROR_NONE;
    return start_rx(uart, port);
}

HAL_StatusTypeDef BSP_UART_GetStatus(UART_HandleTypeDef *uart,
                                     BSP_UART_Status_t *status)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || !port->initialized || status == NULL ||
        __get_IPSR() != 0U) {
        return HAL_ERROR;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *status = port->status;
    __DMB();
    __set_PRIMASK(mask);
    return HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || !port->initialized || port->callback == NULL ||
        !port->status.rx_running) {
        return;
    }
    if (port->dma) {
        /* Sample NDTR, not the callback's historical HT/TC position: an IDLE IRQ
         * may have delivered more data before a pending DMA IRQ is serviced.
         * Same-priority UART/DMA IRQs serialize these cursor updates. */
        const uint32_t remaining = __HAL_DMA_GET_COUNTER(uart->hdmarx);
        if (remaining > BSP_UART_DMA_RX_CAPACITY) {
            port->status.rx_rearm = HAL_ERROR;
            return;
        }
        const uint16_t at = (uint16_t)(
            (BSP_UART_DMA_RX_CAPACITY - remaining) % BSP_UART_DMA_RX_CAPACITY);
        const uint16_t previous = port->dma_rx_at;
        port->dma_rx_at = at;
        __DMB();
        if (at < previous) {
            port->callback(uart, &dma_buffer.rx[previous],
                (uint16_t)(BSP_UART_DMA_RX_CAPACITY - previous), port->context);
        }
        const uint16_t from = at < previous ? 0U : previous;
        if (at > from) {
            port->callback(uart, &dma_buffer.rx[from], (uint16_t)(at - from), port->context);
        }
        return;
    }
    const uint8_t completed = port->active_rx;
    port->status.rx_running = false;
    if (size == 0U || size > BSP_UART_RX_CAPACITY) {
        port->status.rx_rearm = HAL_ERROR;
        return;
    }
    port->active_rx ^= 1U;
    /* 先续接另一个缓冲再交付，回调返回前不会覆盖已交付的块。 */
    (void)start_rx(uart, port);
    port->callback(uart, port->rx[completed], size, port->context);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    UART_Port *port = find_port(uart);
    if (port != NULL && port->initialized) { port->status.tx_busy = false; }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    UART_Port *port = find_port(uart);
    if (port == NULL || !port->initialized) { return; }
    port->status.rx_error |= uart->ErrorCode;
    if (uart->RxState == HAL_UART_STATE_READY) {
        port->status.rx_running = false;
        port->status.rx_rearm = HAL_ERROR;
    }
    /* RX 错误不能清 tx_busy：TX 可能仍持有发送缓冲，须等 TC 回调。 */
}
