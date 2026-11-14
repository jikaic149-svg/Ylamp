#ifndef YLAMP_FAULT_H
#define YLAMP_FAULT_H

#include <stdint.h>

enum {
    YLAMP_FAULT_ERROR = 1U,
    YLAMP_FAULT_ASSERT,
    YLAMP_FAULT_STACK,
    YLAMP_FAULT_EXCEPTION
};

enum {
    YLAMP_STAGE_BOOT,
    YLAMP_STAGE_CLOCK,
    YLAMP_STAGE_IMU_INIT,
    YLAMP_STAGE_CAN_INIT,
    YLAMP_STAGE_MOTOR_INIT,
    YLAMP_STAGE_SCHEDULER,
    YLAMP_STAGE_MOTOR,
    YLAMP_STAGE_IMU,
    YLAMP_STAGE_CALIBRATION,
    YLAMP_STAGE_TUNING,
    YLAMP_STAGE_SERIAL,
    YLAMP_STAGE_STATUS,
    YLAMP_STAGE_WAIT
};

#define YLAMP_FAULT_MAGIC 0x59464C54U

/* Board-specific debugger evidence, not a transport protocol or retained log.
 * Read only after halting the CPU; all fields are lost on reset. */
typedef struct {
    uint32_t magic, version, reason, stage;
    uint32_t hal_tick, control_iterations, stack_free_words, reset_flags;
    uint32_t ipsr, exc_return, msp, psp, control, primask, basepri;
    uint32_t icsr, cfsr, hfsr, dfsr, afsr, abfsr, mmfar, bfar;
    uint32_t shcsr, ccr, vtor;
    uint32_t frame_valid, frame[8]; /* r0,r1,r2,r3,r12,lr,pc,xpsr */
    uint32_t caller, file, line;    /* Flash addresses; decode with matching ELF. */
} YlampFault_Record_t;

extern volatile YlampFault_Record_t g_fault_record;
extern volatile uint32_t g_fault_boot_reset_flags;
extern volatile uint32_t g_fault_stage;

/* Called once before HAL/clock setup; saves RSR then acknowledges reset flags. */
void YlampFault_Boot(void);
__attribute__((noreturn))
void YlampFault_Stop(uint32_t reason, const char *file, uint32_t line);

/* Only this H723 application's DTCM stacks are trusted. Cortex-M7's basic
 * r0..xPSR frame starts at SP for BOTH basic and extended FP frames; the FP
 * extension follows it, not before it. Do not dereference a failed stack. */
static inline uint32_t YlampFault_FrameAddress(uint32_t exc_return,
    uint32_t msp, uint32_t psp, uint32_t cfsr)
{
    switch (exc_return) {
    case 0xFFFFFFF1U: case 0xFFFFFFF9U: case 0xFFFFFFFDU:
    case 0xFFFFFFE1U: case 0xFFFFFFE9U: case 0xFFFFFFEDU:
        break;
    default:
        return 0U;
    }
    /* MUNSTKERR/MSTKERR/MLSPERR and UNSTKERR/STKERR/LSPERR. */
    if ((cfsr & 0x00003838U) != 0U) { return 0U; }
    const uint32_t sp = (exc_return & 4U) != 0U ? psp : msp;
    const uint32_t bytes = (exc_return & 16U) != 0U ? 32U : 104U;
    if ((sp & 7U) != 0U || sp < 0x20000000U ||
        sp > 0x20020000U - bytes) {
        return 0U;
    }
    return sp;
}

#endif
