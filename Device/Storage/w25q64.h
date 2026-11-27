#ifndef YLAMP_W25Q64_H
#define YLAMP_W25Q64_H
#include <stdbool.h>
#include <stdint.h>
enum { W25Q64_SIZE=0x800000U, W25Q64_SECTOR=4096U, W25Q64_PAGE=256U };
typedef enum { W25Q64_HISTORY=1, W25Q64_LOOKAROUND=2 } W25Q64_Owner;
/* Same application thread only. Hold across WIP and suspended erases, not
 * just the short SPI transfer. An unavailable owner is never preempted. */
bool W25Q64_TryLock(W25Q64_Owner owner);
void W25Q64_Unlock(W25Q64_Owner owner);
#ifdef YLAMP_HOST_FIXTURES
void W25Q64_ResetLockForTest(void);
#endif
/* One owner; no chip erase, QE/parameter writes, resets or busy-wait loops.
 * The caller must check WIP/SUS and own the addressed storage partition. */
bool W25Q64_Init(void);
bool W25Q64_Status(uint8_t *sr1,uint8_t *sr2);
bool W25Q64_Read(uint32_t address,uint8_t *data,uint16_t length);
bool W25Q64_Program(uint32_t address,const uint8_t *data,uint16_t length);
bool W25Q64_Erase(uint32_t address);
bool W25Q64_Suspend(void);
bool W25Q64_Resume(void);
#endif
