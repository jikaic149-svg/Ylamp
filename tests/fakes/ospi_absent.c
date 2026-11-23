#include "bsp_ospi.h"
/* Existing controller tests have no external chip. A missing journal must
 * not change cold/legacy/G/trajectory contracts or invent saved positions. */
bool BSP_OSPI_Init(void) { return false; }
bool BSP_OSPI_Command(uint8_t instruction,uint32_t address,bool addressed,
                      const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    (void)instruction;(void)address;(void)addressed;(void)tx;(void)rx;(void)length;
    return false;
}
