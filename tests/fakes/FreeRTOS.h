#ifndef YLAMP_TEST_FREERTOS_H
#define YLAMP_TEST_FREERTOS_H
#include "FreeRTOSConfig.h"
#include <stddef.h>
typedef uint32_t StackType_t;
typedef uint32_t TickType_t;
typedef uint32_t UBaseType_t;
typedef struct {uint32_t reserved[32];} StaticTask_t;
typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms) * configTICK_RATE_HZ / 1000U))
#endif
