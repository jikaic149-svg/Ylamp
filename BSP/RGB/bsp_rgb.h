#ifndef BSP_RGB_H
#define BSP_RGB_H

#include <stdint.h>
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MX_SPI6_Init 后调用；SPI6 专用于 PA7 上单颗 WS2812，初始化写入黑色。
 * 要求 8bit/MSB/模式0/TXONLY，内核时钟 50MHz、分频8，实际 SCK=6.25MHz。
 * 不满足配置返回 HAL_ERROR，不擅自改全局时钟；重复初始化返回 HAL_BUSY。 */
HAL_StatusTypeDef BSP_RGB_Init(void);

/* RGB 各通道 0..255，在线上编码为 GRB；主循环/普通任务使用，禁止 ISR。
 * 同步发送约 0.68ms，HAL 超时 5ms，须保持中断和 HAL 毫秒时基运行。
 * 调用者串行化所有 RGB 操作；无 DMA、软件队列、自动重试或自动闪烁。
 * 非 OK 时颜色可能部分更新；本接口不谎报灯珠已确认颜色。
 * 不得用于高频电机控制路径。 */
HAL_StatusTypeDef BSP_RGB_Set(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif
#endif
