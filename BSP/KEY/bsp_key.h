#ifndef BSP_KEY_H
#define BSP_KEY_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_KEY_EVENT_NONE = 0,
    BSP_KEY_EVENT_PRESSED,
    BSP_KEY_EVENT_RELEASED
} BSP_KEY_Event_t;

/* MX_GPIO_Init 后调用；消抖时间 1..1000 ms，建议 20 ms。
 * 上电已按下只记录状态，不生成虚构的按下事件；重复 Init 返回 HAL_BUSY。 */
HAL_StatusTypeDef BSP_KEY_Init(uint32_t debounce_ms);

/* 主循环/单一任务每 1..5 ms 调用，now_ms 来自 HAL_GetTick()。
 * 返回经消抖的单次边沿，未初始化返回 NONE；不包含长按/连击业务。
 * 时间差使用无符号回绕；相邻调用间隔须小于 2^31 ms。 */
BSP_KEY_Event_t BSP_KEY_Scan(uint32_t now_ms);

/* pressed 不得为 NULL；未初始化返回 HAL_ERROR，不改写输出。 */
HAL_StatusTypeDef BSP_KEY_GetPressed(bool *pressed);

#ifdef __cplusplus
}
#endif
#endif
