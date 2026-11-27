#include "lookaround_task.h"
#include "five_control.h"
#include "lookaround_store.h"
#include <stddef.h>

BSP_CAN_Status_t LookaroundTask_Run(bool key_pressed)
{
    static uint32_t last_tick;
    const FiveControl_Status *s=FiveControl_StatusNow();
    if (!s) { return BSP_CAN_NOT_READY; }
    if (key_pressed) {
        if (s->phase==FIVE_DRAG && !FiveControl_LocalTrajectoryOwned()) {
            unsigned count=0;
            if (!LookaroundStore_Points(&count)) { return BSP_CAN_NOT_READY; }
            return FiveControl_LocalTrajectoryBegin(count);
        }
        return FiveControl_KeyPress();
    }
    if (s->ticks!=last_tick) {
        last_tick=s->ticks;
        LookaroundStore_Run(HAL_GetTick());
        FiveControl_LocalTrajectoryStep(LookaroundStore_Points(NULL));
    }
    return BSP_CAN_OK;
}
