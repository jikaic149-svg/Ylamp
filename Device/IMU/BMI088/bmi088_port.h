#ifndef YLAMP_BMI088_PORT_H
#define YLAMP_BMI088_PORT_H

#include "spi.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { BMI088_PORT_ACCEL = 0, BMI088_PORT_GYRO = 1 } BMI088_PortChip_t;

/* DM-MC02 SPI2, PC0/PC3 CS, PE10/PE12 DRDY.
 * MX_GPIO_Init/MX_SPI2_Init must have completed. One main-loop owner.
 * No DMA, no allocation, no automatic reset/recovery. */
HAL_StatusTypeDef BMI088_Port_Init(void);
/* Boot configuration or explicit sensor power transition; HAL timeout 2ms.
 * The caller supplies the device command/dummy bytes. */
HAL_StatusTypeDef BMI088_Port_Exchange(BMI088_PortChip_t chip, const uint8_t *tx,
                                  uint8_t *rx, uint16_t length);
/* Nonblocking runtime transfer. Buffers must remain alive and untouched until
 * TakeResult is no longer BUSY. No retry/timeout abort/reuse of a live buffer. */
HAL_StatusTypeDef BMI088_Port_Start(BMI088_PortChip_t chip, const uint8_t *tx,
                               uint8_t *rx, uint16_t length);
HAL_StatusTypeDef BMI088_Port_TakeResult(void);
void BMI088_Port_GetReadyCounts(uint32_t counts[2]);
/* GPIO EXTI callback forwards here; counts only, no SPI/math in ISR. */
void BMI088_Port_NotifyReady(uint16_t pin);

#ifdef __cplusplus
}
#endif
#endif
