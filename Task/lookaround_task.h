#ifndef YLAMP_LOOKAROUND_TASK_H
#define YLAMP_LOOKAROUND_TASK_H
#include "bsp_can.h"
#include <stdbool.h>

/* Same app thread as FiveControl, after its tick; never submits CAN itself. */
BSP_CAN_Status_t LookaroundTask_Run(bool key_pressed);
#endif
