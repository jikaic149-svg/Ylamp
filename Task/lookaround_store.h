#ifndef YLAMP_LOOKAROUND_STORE_H
#define YLAMP_LOOKAROUND_STORE_H
#include "five_trajectory.h"

enum {
    LOOKAROUND_READY=1U, LOOKAROUND_LOADED=2U, LOOKAROUND_UPLOADING=4U,
    LOOKAROUND_SAVING=8U, LOOKAROUND_IO_ERROR=16U, LOOKAROUND_FOREIGN=32U,
    LOOKAROUND_NO_CHIP=64U
};
typedef struct {
    uint32_t flags,sequence,checksum,staged_checksum,errors;
    unsigned count,expected,staged_count;
} LookaroundStore_Status;
/* Sole application-thread owner. Boot scan is read-only, before motor power.
 * Saves advance one <=256B transfer per call, never wait for WIP, never move.
 * Unknown nonempty slots are not erased. No internal-Flash point fallback. */
void LookaroundStore_Init(void);
void LookaroundStore_Run(uint32_t now_ms);
const LookaroundStore_Status *LookaroundStore_StatusNow(void);
const FiveTrajectory_Point *LookaroundStore_Points(unsigned *count);
bool LookaroundStore_Begin(unsigned count);
bool LookaroundStore_Append(unsigned index,const uint8_t bytes[64]);
bool LookaroundStore_Save(uint32_t checksum);
/* Read-only backup of this partition only; false while another chip owner is
 * busy. No arbitrary flash address access, formatting or motor operation. */
bool LookaroundStore_Read(unsigned offset,uint8_t *bytes,unsigned length);
void LookaroundStore_CancelUpload(void); /* Never cancels an accepted flash commit. */
#ifdef YLAMP_HOST_FIXTURES
void LookaroundStore_ResetForTest(void);
#endif
#endif
