/* H723 startup/exception adaptation. No HAL, RTOS, UART or motor calls from
 * fatal paths; keep the existing latched stop, with no reset or power action. */
#include "fault.h"
#include "main.h"

volatile YlampFault_Record_t g_fault_record;
volatile uint32_t g_fault_boot_reset_flags;
volatile uint32_t g_fault_stage;
static uint32_t fault_stack[128] __attribute__((used, aligned(8)));

extern volatile uint32_t g_rtos_control_iterations;
extern volatile uint32_t g_rtos_control_stack_free_words;
extern __IO uint32_t uwTick;

void YlampFault_Boot(void)
{
    g_fault_boot_reset_flags = RCC->RSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    g_fault_stage = YLAMP_STAGE_BOOT;
}

static void record_common(uint32_t reason, uint32_t msp, uint32_t psp,
                          uint32_t control, uint32_t primask, uint32_t basepri)
{
    g_fault_record.magic = 0U; /* Commit only after all requested fields exist. */
    g_fault_record.version = 1U;
    g_fault_record.reason = reason;
    g_fault_record.stage = g_fault_stage;
    g_fault_record.hal_tick = uwTick;
    g_fault_record.control_iterations = g_rtos_control_iterations;
    g_fault_record.stack_free_words = g_rtos_control_stack_free_words;
    g_fault_record.reset_flags = g_fault_boot_reset_flags;
    g_fault_record.ipsr = __get_IPSR();
    g_fault_record.msp = msp;
    g_fault_record.psp = psp;
    g_fault_record.control = control;
    g_fault_record.primask = primask;
    g_fault_record.basepri = basepri;
    g_fault_record.icsr = SCB->ICSR;
    g_fault_record.cfsr = SCB->CFSR;
    g_fault_record.hfsr = SCB->HFSR;
    g_fault_record.dfsr = SCB->DFSR;
    g_fault_record.afsr = SCB->AFSR;
    g_fault_record.abfsr = SCB->ABFSR;
    g_fault_record.mmfar = SCB->MMFAR; /* Valid only when CFSR.MMARVALID is set. */
    g_fault_record.bfar = SCB->BFAR;   /* Valid only when CFSR.BFARVALID is set. */
    g_fault_record.shcsr = SCB->SHCSR;
    g_fault_record.ccr = SCB->CCR;
    g_fault_record.vtor = SCB->VTOR;
}

__attribute__((noreturn))
static void commit_and_stop(void)
{
    __DMB();
    g_fault_record.magic = YLAMP_FAULT_MAGIC;
    __DSB();
    for (;;) { __NOP(); }
}

/* Both naked entries switch to a private 512-byte DTCM stack before any C
 * prologue. No FP instructions are permitted in this translation unit, so
 * taking evidence cannot itself trigger lazy FP stacking onto the old stack. */
__attribute__((used, noreturn))
void YlampFault_StopCapture(uint32_t reason, const char *file, uint32_t line,
    uint32_t caller, uint32_t msp, uint32_t psp, uint32_t control,
    uint32_t primask, uint32_t basepri)
{
    record_common(reason, msp, psp, control, primask, basepri);
    g_fault_record.exc_return = 0U;
    g_fault_record.frame_valid = 0U;
    g_fault_record.caller = caller;
    g_fault_record.file = (uint32_t)(uintptr_t)file;
    g_fault_record.line = line;
    commit_and_stop();
}

__attribute__((naked, noreturn))
void YlampFault_Stop(uint32_t reason __attribute__((unused)),
    const char *file __attribute__((unused)), uint32_t line __attribute__((unused)))
{
    __asm volatile(
        "mov r3, lr\n"
        "mrs r4, msp\n"
        "mrs r5, psp\n"
        "mrs r6, control\n"
        "mrs r7, primask\n"
        "mrs r8, basepri\n"
        "cpsid i\n"
        "ldr r12, =fault_stack + 512\n"
        "msr msp, r12\n"
        "bic r12, r6, #2\n" /* Thread mode must use the new MSP, not old PSP. */
        "msr control, r12\n"
        "isb\n"
        "push {r4-r8, lr}\n" /* Five stack arguments plus 8-byte alignment. */
        "b YlampFault_StopCapture\n");
}

__attribute__((used, noreturn))
void YlampFault_ExceptionCapture(uint32_t exc_return, uint32_t msp,
    uint32_t psp, uint32_t control, uint32_t primask, uint32_t basepri)
{
    record_common(YLAMP_FAULT_EXCEPTION, msp, psp, control, primask, basepri);
    g_fault_record.exc_return = exc_return;
    g_fault_record.frame_valid = 0U;
    g_fault_record.caller = 0U;
    g_fault_record.file = 0U;
    g_fault_record.line = 0U;
    const uint32_t address = YlampFault_FrameAddress(exc_return, msp, psp,
                                                    g_fault_record.cfsr);
    if (address != 0U) {
        const volatile uint32_t *frame = (const volatile uint32_t *)(uintptr_t)address;
        if ((frame[7] & (1U << 24U)) != 0U) { /* Stacked xPSR must be Thumb. */
            for (unsigned i = 0U; i < 8U; ++i) {
                g_fault_record.frame[i] = frame[i];
            }
            g_fault_record.frame_valid = 1U;
        }
    }
    commit_and_stop();
}

__attribute__((naked, noreturn))
void YlampFault_Exception(void)
{
    __asm volatile(
        "mov r0, lr\n"
        "mrs r1, msp\n"
        "mrs r2, psp\n"
        "mrs r3, control\n"
        "mrs r4, primask\n"
        "mrs r5, basepri\n"
        "cpsid i\n"
        "ldr r12, =fault_stack + 512\n"
        "msr msp, r12\n"
        "push {r4, r5}\n"
        "b YlampFault_ExceptionCapture\n");
}

void NMI_Handler(void) __attribute__((alias("YlampFault_Exception"), noreturn));
void HardFault_Handler(void) __attribute__((alias("YlampFault_Exception"), noreturn));
void MemManage_Handler(void) __attribute__((alias("YlampFault_Exception"), noreturn));
void BusFault_Handler(void) __attribute__((alias("YlampFault_Exception"), noreturn));
void UsageFault_Handler(void) __attribute__((alias("YlampFault_Exception"), noreturn));

_Static_assert(YLAMP_FAULT_ERROR == 1U, "Error_Handler assembly reason");
__attribute__((naked, noreturn))
void Error_Handler(void)
{
    /* Tail branch preserves the failing caller's LR, including SDK callers. */
    __asm volatile(
        "movs r0, #1\n"
        "movs r1, #0\n"
        "movs r2, #0\n"
        "b YlampFault_Stop\n");
}
