/* Tests production App/rtos.c wiring, not the ARM kernel/context switch itself. */
#include "FreeRTOS.h"
#include "task.h"
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static jmp_buf returned;
static TaskFunction_t entry;
static uint32_t steps, now = 1234U, delays, fatal_calls;
static bool fail_create;
static uint32_t last_reason;
extern volatile uint32_t g_rtos_control_iterations;
extern volatile uint32_t g_rtos_control_stack_free_words;
void Ylamp_StartScheduler(void);
void vApplicationGetIdleTaskMemory(StaticTask_t **, StackType_t **, uint32_t *);
void vApplicationStackOverflowHook(TaskHandle_t, char *);

uint32_t HAL_GetTick(void) { return now; }
void Error_Handler(void) { ++fatal_calls; longjmp(returned, 2); }
void YlampFault_Stop(uint32_t reason, const char *file, uint32_t line)
{
    if (reason == YLAMP_FAULT_ASSERT) { assert(file && line != 0U); }
    else { assert(reason == YLAMP_FAULT_STACK && !file && line == 0U); }
    last_reason = reason;
    ++fatal_calls;
    longjmp(returned, 2);
}
TaskHandle_t xTaskCreateStatic(TaskFunction_t fn, const char *name, uint32_t depth,
    void *arg, UBaseType_t priority, StackType_t *stack, StaticTask_t *tcb)
{
    assert(fn && !arg && stack && tcb && !strcmp(name, "YlampControl"));
    assert(depth == 2048U && sizeof(*stack) == 4U && priority == 2U);
    entry = fn;
    return fail_create ? NULL : tcb;
}
TickType_t xTaskGetTickCount(void) { return 40U; }
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t task)
{
    assert(task == NULL);
    return 123U;
}
void vTaskDelayUntil(TickType_t *wake, TickType_t period)
{
    assert(period == 1U && *wake == 40U + delays);
    *wake += period;
    ++delays; ++now;
    if (delays == 1000U) { longjmp(returned, 1); }
}
void Ylamp_ControlStep(uint32_t ms) { assert(ms == now); ++steps; }
void vTaskStartScheduler(void)
{
    StaticTask_t *tcb;
    StackType_t *stack;
    uint32_t depth;
    vApplicationGetIdleTaskMemory(&tcb, &stack, &depth);
    assert(tcb && stack && depth == 256U);
    entry(NULL);
}

int main(void)
{
    assert(configSUPPORT_DYNAMIC_ALLOCATION == 0 && configUSE_TIMERS == 0);
    if (!setjmp(returned)) { Ylamp_StartScheduler(); assert(0); }
    assert(steps == 1000U && delays == 1000U && fatal_calls == 0U);
    assert(g_rtos_control_iterations == 1000U && g_rtos_control_stack_free_words == 123U);
    fail_create = true;
    if (!setjmp(returned)) { Ylamp_StartScheduler(); assert(0); }
    assert(fatal_calls == 1U && last_reason == YLAMP_FAULT_ASSERT);
    if (!setjmp(returned)) { vApplicationStackOverflowHook(NULL, NULL); assert(0); }
    assert(fatal_calls == 2U && last_reason == YLAMP_FAULT_STACK);
    puts("PASS: static tasks/stacks, 1ms absolute-period wiring, HAL clock separation,"
         " watermark collection, creation failure and fatal overflow path.");
}
