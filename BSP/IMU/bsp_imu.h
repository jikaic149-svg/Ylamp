#ifndef YLAMP_BSP_IMU_H
#define YLAMP_BSP_IMU_H

#include "spi.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { BSP_IMU_ACCEL = 0, BSP_IMU_GYRO = 1 } BSP_IMU_Chip_t;

/* DM-MC02 SPI2, PC0/PC3 CS, PE10/PE12 DRDY.
 * MX_GPIO_Init/MX_SPI2_Init must have completed. One main-loop owner.
 * No DMA, no allocation, no automatic reset/recovery. */
HAL_StatusTypeDef BSP_IMU_Init(void);
/* Boot configuration or explicit sensor power transition; HAL timeout 2ms.
 * The caller supplies the device command/dummy bytes. */
HAL_StatusTypeDef BSP_IMU_Exchange(BSP_IMU_Chip_t chip, const uint8_t *tx,
                                  uint8_t *rx, uint16_t length);
/* Nonblocking runtime transfer. Buffers must remain alive and untouched until
 * TakeResult is no longer BUSY. No retry/timeout abort/reuse of a live buffer. */
HAL_StatusTypeDef BSP_IMU_Start(BSP_IMU_Chip_t chip, const uint8_t *tx,
                               uint8_t *rx, uint16_t length);
HAL_StatusTypeDef BSP_IMU_TakeResult(void);
void BSP_IMU_GetReadyCounts(uint32_t counts[2]);
/* GPIO EXTI callback forwards here; counts only, no SPI/math in ISR. */
void BSP_IMU_NotifyReady(uint16_t pin);

#ifdef __cplusplus
}
#endif
#endif
