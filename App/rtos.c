#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

static StaticTask_t control_tcb, idle_tcb;
static StackType_t control_stack[YLAMP_CONTROL_STACK_WORDS];
static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

/* Debugger-readable evidence; watermarks require target measurements. */
volatile uint32_t g_rtos_control_iterations;
volatile uint32_t g_rtos_control_stack_free_words;

void Ylamp_ControlStep(uint32_t now_ms);

static void control_task(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        Ylamp_ControlStep(HAL_GetTick());
        ++g_rtos_control_iterations;
        if ((g_rtos_control_iterations % 1000U) == 0U) {
            g_rtos_control_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(YLAMP_CONTROL_PERIOD_MS));
    }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack,
                                   uint32_t *depth)
{
    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    YlampFault_Stop(YLAMP_FAULT_STACK, NULL, 0U); /* Same latched fatal stop. */
}

void Ylamp_StartScheduler(void)
{
    configASSERT(pdMS_TO_TICKS(YLAMP_CONTROL_PERIOD_MS) > 0U);
    configASSERT(xTaskCreateStatic(control_task, "YlampControl",
        YLAMP_CONTROL_STACK_WORDS, NULL, YLAMP_CONTROL_PRIORITY,
        control_stack, &control_tcb) != NULL);
    vTaskStartScheduler();
    Error_Handler(); /* Returning from scheduler start is not normal operation. */
}
