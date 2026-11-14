/* Board ownership: USART1 uses DMA2 streams 5/6, not SPI2's DMA1 streams 0/1.
 * Stream 6 is reserved here; the SDK's unsupported UART9 must not be enabled.
 * Keep USART1 and both DMA IRQs at priority 5; none calls FreeRTOS APIs. */
#include "bsp_uart.h"

static DMA_HandleTypeDef receive_dma, transmit_dma;

HAL_StatusTypeDef YlampSerialDMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    receive_dma.Instance = DMA2_Stream5;
    receive_dma.Init.Request = DMA_REQUEST_USART1_RX;
    receive_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    receive_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    receive_dma.Init.MemInc = DMA_MINC_ENABLE;
    receive_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    receive_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    receive_dma.Init.Mode = DMA_CIRCULAR;
    receive_dma.Init.Priority = DMA_PRIORITY_HIGH;
    receive_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&receive_dma) != HAL_OK) { return HAL_ERROR; }
    transmit_dma.Instance = DMA2_Stream6;
    transmit_dma.Init.Request = DMA_REQUEST_USART1_TX;
    transmit_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    transmit_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    transmit_dma.Init.MemInc = DMA_MINC_ENABLE;
    transmit_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    transmit_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    transmit_dma.Init.Mode = DMA_NORMAL;
    transmit_dma.Init.Priority = DMA_PRIORITY_MEDIUM;
    transmit_dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&transmit_dma) != HAL_OK) {
        (void)HAL_DMA_DeInit(&receive_dma);
        return HAL_ERROR;
    }
    __HAL_LINKDMA(&huart1, hdmarx, receive_dma);
    __HAL_LINKDMA(&huart1, hdmatx, transmit_dma);
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5U, 0U);
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    return HAL_OK;
}

void DMA2_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(&receive_dma); }
void DMA2_Stream6_IRQHandler(void) { HAL_DMA_IRQHandler(&transmit_dma); }
