#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_CAN_MAX_STD_ID      0x7FFU
#define BSP_CAN_MAX_DATA_LENGTH 8U

typedef enum {
    BSP_CAN_OK = 0,
    BSP_CAN_INVALID_ARG,
    BSP_CAN_NOT_READY,
    BSP_CAN_BUSY,
    BSP_CAN_NO_RESOURCE,
    BSP_CAN_ERROR
} BSP_CAN_Status_t;

/* 电机接口保持标准 ID、Classic CAN 数据帧，不支持远程帧。 */
typedef struct {
    uint32_t id; /* 0 .. BSP_CAN_MAX_STD_ID */
    uint8_t data[BSP_CAN_MAX_DATA_LENGTH];
    uint8_t len; /* 有效字节数：0 .. BSP_CAN_MAX_DATA_LENGTH */
} BSP_CAN_Frame_t;

/* 诊断时间为 HAL tick ms。LEC/DLEC 0=无错误，1..6=错误类型，7=无新事件。
 * 一次寄存器采样，不承诺这些硬件字段严格同时锁存。
 * counters_valid=0 时错误计数未知，不能将结构中的零当作实测零。 */
typedef struct {
    uint32_t timestamp_ms;
    uint32_t interrupts;       /* 采样时已启用的错误中断位；轮询发现可为 0 */
    uint32_t tx_pending_mask;  /* TXBRP；待发送不等于对端已收到 */
    uint8_t lec, dlec;
    uint8_t tx_errors, rx_errors, logged_errors;
    uint8_t bus_off, error_passive, warning, rx_passive, counters_valid;
} BSP_CAN_ErrorSnapshot_t;

typedef struct {
    uint32_t error_observations; /* 软件异常采样次数，不是总线错误帧数 */
    uint8_t first_valid, bus_off_valid;
    BSP_CAN_ErrorSnapshot_t first;   /* 上电后最早观察到的异常，不被后续覆盖 */
    BSP_CAN_ErrorSnapshot_t latest;
    BSP_CAN_ErrorSnapshot_t bus_off; /* 首次观察到 Bus-Off 时的独立快照 */
} BSP_CAN_Diagnostics_t;

/**
 * 接收回调在 FDCAN 接收中断内执行，不允许阻塞、打印或调用本模块其他接口。
 * frame 及其 data 仅在本次回调返回前有效，需要保存时由调用者复制。
 * context 原样传回，可为 NULL；非空对象必须在整个接收期间保持有效。
 */
typedef void (*BSP_CAN_RxCallback_t)(FDCAN_HandleTypeDef *hfdcan,
                                    const BSP_CAN_Frame_t *frame,
                                    void *context);

/**
 * 初始化一个 FDCAN 实例的 BSP 接收路径并启动控制器。
 *
 * 前置条件：已完成 HAL 初始化，使用 Classic CAN 标准帧配置，
 * 已配置消息 RAM、接收 FIFO0（1..16 项）、发送 FIFO/Queue 和对应 NVIC 中断。
 * IT0/IT1 入口须转发 BSP_CAN_IRQHandler，以便在 HAL 清中断前保存错误证据。
 * 仅接管生成的 hfdcan1/2；全模块静态回调容量 16 项。
 * hfdcan 不得为 NULL，必须指向板级生成的有效句柄，生命周期覆盖整个运行期。
 * 本模块独占该实例的接收过滤配置、FIFO0、协议状态读取和错误通知配置；
 * 不在其他代码直接读取 PSR/ECR，否则会消费 LEC/错误日志计数。
 * 不与参考工程旧版 bsp_can.c 同时使用。
 *
 * 仅在启动阶段、注册回调和发送之前调用；不在 ISR 中调用。
 * 成功返回 OK；参数错误返回 INVALID_ARG；配置/启动失败返回 ERROR。
 * 已由本模块初始化的实例返回 BUSY，不重新启动或清除已有状态。
 * 失败时实例不可用，且不保证 HAL 已完全回滚；调用者不得直接发送。
 * 已尝试硬件启动但失败的实例不隐式重试 Init，须由上层诊断处理。
 * 不修改引脚、时钟、位速率或电源支路，不发送电机使能/控制命令。
 */
BSP_CAN_Status_t BSP_CAN_Init(FDCAN_HandleTypeDef *hfdcan);

/**
 * 非阻塞提交一帧到硬件发送 FIFO/Queue，无软件队列或软件自动重试。
 *
 * hfdcan、frame 不得为 NULL，id 和 len 必须满足 BSP_CAN_Frame_t 的范围。
 * 调用期间 frame 不得被修改；返回后即可复用，模块不保留该指针。
 * 仅在主循环/任务中调用；同一实例的多个发送调用由调用者串行化，
 * 且不得与其他代码对该实例的直接 HAL 发送并发执行。
 *
 * OK 仅表示硬件已接受，不表示已上总线、收到 ACK 或电机已执行。
 * INVALID_ARG：参数非法；NOT_READY：未初始化或控制器不可发送；
 * BUSY：发送 FIFO/Queue 已满或 HAL 忙；ERROR：其他 HAL 提交错误。
 * 非 OK 时本次未成功提交，是否再试由上层决定，不在此函数内等待。
 * 控制器硬件自动重发行为沿用 CubeMX 配置，不由本接口改变。
 * 正常提交不读 PSR/ECR；错误中断待处理或 INIT 时才补充采样，
 * 硬件仍在 INIT 时拒绝提交，不自动清 INIT/恢复总线。
 */
BSP_CAN_Status_t BSP_CAN_Send(FDCAN_HandleTypeDef *hfdcan,
                              const BSP_CAN_Frame_t *frame);

/* 显式、非阻塞、分步恢复；仅资源所有者普通任务串行调用，非 Send/ISR 隐式重试。
 * 只接受实际 INIT+Bus-Off；正常或非 Bus-Off 的 INIT 返回 NOT_READY，不修改硬件。
 * 首次 BUSY：已阻断发送，Stop/CCE 撤销旧 TX 并清 RX FIFO 状态；后续调用核对
 * 硬件队列确已清空才 Start，再等待硬件 Bus-Off 同步结束才返回 OK/开放发送。
 * BUSY 表示仍在推进；ERROR 不代表回滚成功，发送可仍被阻断，不能强清状态。
 * 超时/重试次数归调用者；不重放旧帧、不发电机命令、不清诊断/电机故障，
 * 不复位共享 RCC/引脚/位时序，不注销接收回调。恢复时所有旧 TX/RX 均丢弃。 */
BSP_CAN_Status_t BSP_CAN_RecoverBusOff(FDCAN_HandleTypeDef *hfdcan);

/* 仅从该实例 IT0/IT1 中断入口调用，先有界采样，再转发 HAL IRQHandler。
 * BSP 尚未尝试初始化的实例忽略，不访问未准备的 HAL 句柄。
 * 不打印、不重试、不恢复 CAN、不提交执行器命令。 */
void BSP_CAN_IRQHandler(FDCAN_HandleTypeDef *hfdcan);

/* 主循环/任务中复制一致的 RAM 快照；不读/清硬件错误寄存器，不发送 CAN。
 * 参数非空；成功初始化后可读，即使 Bus-Off。无错误时 valid 为 0。
 * 证据保持到 MCU 重启，不随 Task CLEAR、正常发送或本 Getter 清除。
 * 首次“观察到”不保证是物理层首个错误；中断合并/硬件字段更新可能丢细节。 */
BSP_CAN_Status_t BSP_CAN_GetDiagnostics(FDCAN_HandleTypeDef *hfdcan,
                                        BSP_CAN_Diagnostics_t *diagnostics);

/**
 * 为指定实例和标准 ID 注册一个精确匹配的接收回调。
 *
 * hfdcan、callback 不得为 NULL；id 范围为 0 .. BSP_CAN_MAX_STD_ID。
 * context 可为 NULL。回调函数及 context 对象需保持有效，直到系统停止接收。
 * 仅在 BSP_CAN_Init() 成功后、上层开始控制前的主循环/任务中注册；
 * 注册期间不允许同一实例的注册/发送并发调用，不在 ISR 中调用。
 * 实现负责使注册表更新相对于接收 ISR 安全；不提供运行期撤销接口。
 * 只向已注册 ID 分发回调；未知 ID 不分发，注册不承诺补收此前的帧。
 *
 * OK：注册成功；INVALID_ARG：参数非法；NOT_READY：实例未初始化；
 * BUSY：同一实例和 ID 已注册，不覆盖旧回调；
 * NO_RESOURCE：静态注册表已满，既有注册保持不变。
 */
BSP_CAN_Status_t BSP_CAN_RegisterRxCallback(FDCAN_HandleTypeDef *hfdcan,
                                            uint32_t id,
                                            BSP_CAN_RxCallback_t callback,
                                            void *context);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */
