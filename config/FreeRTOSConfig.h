#ifndef YLAMP_FREERTOS_CONFIG_H
#define YLAMP_FREERTOS_CONFIG_H

#include <stdint.h>
#include "fault.h"
extern uint32_t SystemCoreClock;
void Error_Handler(void);

/* Native FreeRTOS V10.3.1 from BSP_CUBE_ROOT, no CMSIS wrapper or heap. */
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ 1000U
#define configMAX_PRIORITIES 4U
#define configMINIMAL_STACK_SIZE 256U
#define configMAX_TASK_NAME_LEN 16U
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 0
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_TIMERS 0
#define configUSE_MUTEXES 0
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_CO_ROUTINES 0
#define configUSE_TASK_NOTIFICATIONS 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_TRACE_FACILITY 0
#define configGENERATE_RUN_TIME_STATS 0
#define configPRIO_BITS 4U
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15U
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5U
#define configKERNEL_INTERRUPT_PRIORITY (15U << 4U)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (5U << 4U)
#define configASSERT(condition) do { if (!(condition)) { \
    YlampFault_Stop(YLAMP_FAULT_ASSERT, __FILE__, __LINE__); } } while (0)
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetSchedulerState 1
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* FreeRTOS native stack depths are uint32_t words: 2048 words = 8192 bytes. */
#define YLAMP_CONTROL_STACK_WORDS 2048U
#define YLAMP_CONTROL_PRIORITY 2U
#define YLAMP_CONTROL_PERIOD_MS 1U

#endif
