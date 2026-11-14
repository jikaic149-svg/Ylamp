#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_UART_RX_CAPACITY 128U
#define BSP_UART_TX_CAPACITY 512U
#define BSP_UART_DMA_RX_CAPACITY 512U

/* ISR 回调，data 在本次回调返回前有效；需保存则复制。context 可为 NULL。
 * IDLE/满缓冲仅是接收分块，不代表完整业务帧。禁止阻塞/打印/重入本模块。 */
typedef void (*BSP_UART_RxCallback_t)(UART_HandleTypeDef *uart,
                                     const uint8_t *data, uint16_t len,
                                     void *context);

typedef struct {
    bool rx_running;
    bool tx_busy;
    uint32_t rx_error;             /* 累积 HAL UART 错误位，RestartRx 成功清零 */
    HAL_StatusTypeDef rx_rearm;    /* 最近接收启动/续接结果 */
} BSP_UART_Status_t;

/* 当前生成工程有 IRQ 的 USART1/2/3/6/10、UART5/7。
 * 三个独立 TTL 插座是 USART1、UART7、USART10；UART5 是 SBUS 接收，不可普通发送。
 * UART4/8/9 未接入生成的中断入口，明确返回 HAL_ERROR，不伪装可用。
 * MX_USARTx_UART_Init/MX_UARTx_Init 后调用，8位有效数据（允许9bit+校验）。
 * callback 可为 NULL（不启动接收）；context 和回调需在整个接收期间有效。
 * 模块独占 HAL UART 回调和该实例收发，不与原 BSP/DMA 驱动并用。
 * 同一实例由一个主循环/任务持有；普通接口禁止 ISR，调用者串行化。 */
HAL_StatusTypeDef BSP_UART_Init(UART_HandleTypeDef *uart,
                                BSP_UART_RxCallback_t callback, void *context);

/* USART1 专用 DMA 后端：调用方先绑定 circular RX / normal TX DMA。
 * 使用 .dma_buffer 内独立静态缓冲，链接到 MPU 不可缓存的 AXI SRAM 首64KiB；
 * 不能放在 DMA1/2 不可达的 DTCM。HT/TC/IDLE 只报告新字节，不停止循环RX。
 * USART1 与 RX DMA IRQ 必须同抢占优先级，最大服务间隔小于半缓冲时间。
 * 其他端口仍使用上面的 IT 入口；不隐式回退，不自动恢复硬件错误。 */
HAL_StatusTypeDef BSP_UART_InitDMA(UART_HandleTypeDef *uart,
                                   BSP_UART_RxCallback_t callback, void *context);

/* 拷贝 1..BSP_UART_TX_CAPACITY 字节，非阻塞 IT 或 DMA 发送；data 不得为 NULL，返回后可复用。
 * OK 只是已启动；GetStatus.tx_busy 清零表示 HAL 的 TC 完成回调已发生。
 * BUSY 不覆盖在途数据，不排队、不重试；ERROR 表示参数/配置/启动错误。 */
HAL_StatusTypeDef BSP_UART_Send(UART_HandleTypeDef *uart,
                                const uint8_t *data, uint16_t len);

/* 仅显式调用时重启 RX，会丢弃未交付的部分接收，不中断 TX、不复位总线。
 * 正常 IDLE/满缓冲会续接下一缓冲；故障或续接失败不自动恢复。
 * 再次启动前确认旧接收停止。DMA 后端故障锁存，本接口返回 HAL_BUSY；
 * 不在控制任务中同步等待 DMA 中止或悄悄重置共享外设。 */
HAL_StatusTypeDef BSP_UART_RestartRx(UART_HandleTypeDef *uart);

/* status 不得为 NULL；短临界区取得一致状态，不代表远端已收到发送数据。 */
HAL_StatusTypeDef BSP_UART_GetStatus(UART_HandleTypeDef *uart,
                                     BSP_UART_Status_t *status);

#ifdef __cplusplus
}
#endif
#endif
