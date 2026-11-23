#ifndef YLAMP_BSP_OSPI_H
#define YLAMP_BSP_OSPI_H
#include <stdbool.h>
#include <stdint.h>

/* Board02 OSPI2, standard single-data-line transactions. Task context only.
 * No memory mapping, DMA, interrupts, reset or automatic recovery. */
bool BSP_OSPI_Init(void);
bool BSP_OSPI_Command(uint8_t instruction, uint32_t address, bool addressed,
                      const uint8_t *tx, uint8_t *rx, uint16_t length);
#endif
