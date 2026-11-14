#ifndef BSP_POWER_H
#define BSP_POWER_H

#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* V1.1 手册：OUT1/CAN1=PC14（丝印 VCC_OUT0），
 * OUT2/CAN2=PC13（丝印 VCC_OUT1），对外 5V=PC15。
 * OUT1/2 跟随输入电压，并非固定 24V 稳压输出。使能信号均高有效。 */
typedef enum {
    BSP_POWER_OUT1 = 0,
    BSP_POWER_OUT2,
    BSP_POWER_EXT_5V
} BSP_POWER_Branch_t;

/* MX_GPIO_Init 后、连接负载控制前调用，三路保持关闭。
 * 重复调用返回 HAL_BUSY，不在运行中隐式关闭已有支路。 */
HAL_StatusTypeDef BSP_POWER_Init(void);

/* 仅主循环/单一任务调用。开启必须由应用显式请求，不与其他代码写这些 IO 并发。
 * HAL_ERROR 表示未初始化/非法支路/ISR；无自动重开、故障断电或延时策略。 */
HAL_StatusTypeDef BSP_POWER_Set(BSP_POWER_Branch_t branch, bool enabled);
/* 联动 OUT1/CAN1 与 OUT2/CAN2，一次 GPIO 写入两路 EN；不改变对外 5V。
 * 调用上下文与 Set 相同。仅控制供电，不发送电机使能/失能命令。
 * 不保证负载端电压同时建立；关闭会失去电机支撑，由应用显式决定。 */
HAL_StatusTypeDef BSP_POWER_SetMotorPower(bool enabled);
HAL_StatusTypeDef BSP_POWER_AllOff(void);

/* enabled 不得为 NULL。读取 GPIO 输出锁存值，不代表负载端电压或电流已被测量。 */
HAL_StatusTypeDef BSP_POWER_GetEnabled(BSP_POWER_Branch_t branch, bool *enabled);

#ifdef __cplusplus
}
#endif
#endif
