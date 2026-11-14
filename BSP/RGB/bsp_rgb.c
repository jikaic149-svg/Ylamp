#include "bsp_rgb.h"

#include <stdbool.h>

#define RGB_SPI_CLOCK_HZ 6250000U
#define RGB_RESET_BYTES 250U
#define RGB_FRAME_BYTES (RGB_RESET_BYTES + 24U + RGB_RESET_BYTES)

static bool initialized;
/* 静态缓冲避免占用控制任务栈；非 DMA，不依赖 Cache/链接段设置。 */
_Alignas(4) static uint8_t wire_data[RGB_FRAME_BYTES];

HAL_StatusTypeDef BSP_RGB_Init(void)
{
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U) { return HAL_ERROR; }
    if (initialized) { return HAL_BUSY; }
    if (hspi6.Instance != SPI6 || hspi6.State != HAL_SPI_STATE_READY ||
        hspi6.Init.Mode != SPI_MODE_MASTER ||
        hspi6.Init.Direction != SPI_DIRECTION_2LINES_TXONLY ||
        hspi6.Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi6.Init.FirstBit != SPI_FIRSTBIT_MSB ||
        hspi6.Init.CLKPolarity != SPI_POLARITY_LOW ||
        hspi6.Init.CLKPhase != SPI_PHASE_1EDGE ||
        hspi6.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_8 ||
        HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6) != RGB_SPI_CLOCK_HZ * 8U) {
        return HAL_ERROR;
    }
    initialized = true;
    const HAL_StatusTypeDef result = BSP_RGB_Set(0U, 0U, 0U);
    if (result != HAL_OK) { initialized = false; }
    return result;
}

HAL_StatusTypeDef BSP_RGB_Set(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!initialized || __get_IPSR() != 0U || __get_PRIMASK() != 0U) {
        return HAL_ERROR;
    }
    if (hspi6.State != HAL_SPI_STATE_READY) { return HAL_BUSY; }
    const uint8_t grb[3] = {green, red, blue};
    for (unsigned channel = 0; channel < 3U; ++channel) {
        for (unsigned bit = 0; bit < 8U; ++bit) {
            /* 6.25MHz: T0H=0.32us，T1H=0.64us，每个颜色位 1.28us。 */
            wire_data[RGB_RESET_BYTES + channel * 8U + bit] =
                (grb[channel] & (0x80U >> bit)) != 0U ? 0x78U : 0x60U;
        }
    }
    /* 前后各 320us 连续低电平，首帧和相邻帧都有明确的复位间隔。 */
    return HAL_SPI_Transmit(&hspi6, wire_data, sizeof(wire_data), 5U);
}
