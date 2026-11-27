#ifndef YLAMP_FIVE_HISTORY_H
#define YLAMP_FIVE_HISTORY_H
#include "motor_config.h"

enum {
    FIVE_HISTORY_READY=1U,FIVE_HISTORY_LOADED=2U,FIVE_HISTORY_WRITING=4U,
    FIVE_HISTORY_ERASING=8U,FIVE_HISTORY_IO_ERROR=16U,
    FIVE_HISTORY_FOREIGN=32U,FIVE_HISTORY_NO_CHIP=64U
};
typedef struct {
    uint32_t flags,sequence,writes,errors,committed_at_ms;
    uint8_t saved_mask,restored_mask;
    float saved_rad[5],step_deg[5]; /* Supplemental step; zero-crossing saves independently. */
    uint32_t corrections[5];
} FiveHistory_Status;
/* Same task/sole owner as FiveControl. Init reads only, no erase/program.
 * Failure leaves original coordinate fallback usable and visible in flags. */
void FiveHistory_Init(void);
float FiveHistory_Reference(unsigned axis,const DM_Motor_Feedback_t *feedback);
/* First fresh acquisition, turn-N changes and supplemental displacement
 * checkpoints use the same journal. At most one short journal step;
 * flash WIP is polled on later calls, never waited on in the motor owner.
 * Does not enable/disable, change modes or write motor parameters. */
void FiveHistory_Run(uint32_t now);
const FiveHistory_Status *FiveHistory_StatusNow(void);
#ifdef YLAMP_HOST_FIXTURES
void FiveHistory_ResetForTest(void); /* Simulate lost SRAM, retain fake NOR. */
#endif
#endif
