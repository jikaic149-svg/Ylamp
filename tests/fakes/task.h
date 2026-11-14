#ifndef YLAMP_TEST_TASK_H
#define YLAMP_TEST_TASK_H
#include "FreeRTOS.h"
TaskHandle_t xTaskCreateStatic(TaskFunction_t, const char *, uint32_t, void *,
                               UBaseType_t, StackType_t *, StaticTask_t *);
TickType_t xTaskGetTickCount(void);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t);
void vTaskDelayUntil(TickType_t *, TickType_t);
void vTaskStartScheduler(void);
#endif
