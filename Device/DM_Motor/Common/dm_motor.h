#ifndef DEVICE_DM_MOTOR_H
#define DEVICE_DM_MOTOR_H

/* Shared API for model/version profiles under Device/DM_Motor. */

#include "bsp_can.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DM_MOTOR_DM4310 = 1,
    DM_MOTOR_DM4340
} DM_Motor_Model_t;

typedef enum {
    DM_MOTOR_PROTOCOL_V13 = 0, /* 支持独立0xCC查询和0x33读参；保持默认兼容。 */
    DM_MOTOR_PROTOCOL_LEGACY_MIT /* 命令应答取反馈，不支持上述独立查询。 */
} DM_Motor_Protocol_t;

/* 模块持有的静态对象，不向调用者暴露可变状态。 */
typedef struct DM_Motor DM_Motor_t;

typedef struct {
    DM_Motor_Model_t model;
    FDCAN_HandleTypeDef *bus;
    uint16_t can_id;    /* 本版支持 1..15；与电机 CAN ID 相同。 */
    uint16_t master_id; /* 1..0x7FE，每个电机在同一总线上必须唯一。 */
    /* 三项必须为正的有限正规浮点数；与每台电机真实配置一致。 */
    float pmax;        /* rad，协议映射范围 ±pmax，不是机械限位。 */
    float vmax;        /* rad/s，协议映射范围 ±vmax，不是运行限速。 */
    float tmax;        /* N*m，协议映射范围 ±tmax，不是安全力矩上限。 */
    DM_Motor_Protocol_t protocol; /* 按已知固件配置，不在运行中自动探测/升级。 */
} DM_Motor_Config_t;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float kp;           /* N*m/rad，0..500。 */
    float kd;           /* N*m*s/rad，0..5。 */
    float torque_ff_nm;
} DM_Motor_MIT_Command_t;

typedef struct {
    float position_rad;   /* 原始协议位置，不自动归零/展开多圈。 */
    float velocity_rad_s;
    float torque_nm;      /* 驱动估算反馈，不是外置传感器实测。 */
    uint8_t state;        /* D0 高 4 位原样保留：0 失能、1 使能，其余查固件故障表。 */
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
    uint32_t received_at_ms; /* HAL_GetTick() 接收时间，可回绕。 */
    uint32_t received_count; /* 有效反馈计数，可回绕；不是 CAN 序号。 */
} DM_Motor_Feedback_t;

/* 填入 24V 型号的 SDK 默认量程；bus=NULL、两个 ID=0，不能直接 Init。
 * DM4310: 12.5/30/10；DM4340: 12.5/10/28。
 * 必须与上位机读回的真实 PMAX/VMAX/TMAX 核对；只填软件配置，不写电机。
 * config 非空；无效型号/参数返回 INVALID_ARG，输出保持不变。 */
BSP_CAN_Status_t DM_Motor_DefaultConfig(DM_Motor_Model_t model,
                                       DM_Motor_Config_t *config);

/* 在 BSP_CAN_Init 成功后、开始业务发送前调用；不发送任何帧，不切模式/电源。
 * 电机必须已在上位机设为 MIT；按当前 5.0.1.1/5.1.1.9 使用的 MIT 布局实现，
 * 未逐版本上板验证，也不通过软件识别或验证实际固件、量程与控制模式。
 * config 在调用时复制，motor 为非空输出；失败不改写输出。
 * 最多 16 个静态实例，和其他 CAN 使用者共享 BSP 的 16 个回调名额。
 * 同总线 TX/RX ID 不能交叉冲突；已占用返回 BUSY，满表返回 NO_RESOURCE。
 * 不提供反初始化，成功返回的句柄在整个运行期有效，不可自行释放。
 * 所有公开接口只在主循环/普通任务调用；同总线调用由上层串行化。
 * 非法句柄/数值/ISR 调用返回 INVALID_ARG；其他错误沿用 BSP_CAN_Status_t。
 * 不创建调度/队列/超时/自动恢复，不承诺任何周期或最大允许力矩。 */
BSP_CAN_Status_t DM_Motor_Init(const DM_Motor_Config_t *config,
                              DM_Motor_t **motor);

/* 显式发送 FF FF FF FF FF FF FF FC / FD。
 * OK 仅为硬件接受帧，不代表电机已使能/失能；以新反馈 state 核对。
 * 不清故障、不改变本地反馈，也不自动重发。失能不是机械制动或断电。 */
BSP_CAN_Status_t DM_Motor_Enable(DM_Motor_t *motor);
BSP_CAN_Status_t DM_Motor_Disable(DM_Motor_t *motor);

/* V13只读状态查询：标准 ID 0x7FF，载荷 {CAN_ID,0,0xCC,0,0,0,0,0}。
 * 不使能、不设零、不写参数；OK 只代表已提交，以新反馈计数确认应答。
 * LEGACY_MIT返回NOT_READY且不发送；只从显式MIT/使能/失能命令应答取反馈。
 * 绝不把只读查询替换成失能或零力矩命令。不内置周期、超时或重试。 */
BSP_CAN_Status_t DM_Motor_RequestFeedback(DM_Motor_t *motor);

/* Diagnostic only, no instance/feedback mutation: 1..0x7FE address on an
 * initialized bus, false=0xCC feedback query, true=0x33 read Master ID/RID7.
 * Registered LEGACY_MIT addresses return NOT_READY without sending.
 * No writes, enable or mode change. OK is submission only; inspect raw RX.
 * Ordinary task only; caller owns bus and response attribution. */
BSP_CAN_Status_t DM_Motor_ProbeReadOnly(FDCAN_HandleTypeDef *bus,
                                      uint16_t can_id, bool read_master_id);

/* MIT: Tref = kp*(p_des-p) + kd*(v_des-v) + torque_ff。
 * 位置/速度/前馈力矩须在对应 ±MAX 内，增益范围见上；必须全部为有限数。
 * 超范围直接拒绝且不发帧，不静默截断；拒绝与特殊命令碰撞的编码。
 * OK 只表示提交硬件 FIFO。调用后 command 可立即复用。
 * 未检查反馈时效或使能状态；使能时机、运行边界和故障退出由上层决定。 */
BSP_CAN_Status_t DM_Motor_SendMIT(DM_Motor_t *motor,
                                 const DM_Motor_MIT_Command_t *command);
/* Same encoder used by SendMIT. Pure preview, no HAL, bus, or enable. */
BSP_CAN_Status_t DM_Motor_EncodeMIT(const DM_Motor_Config_t *config,
    const DM_Motor_MIT_Command_t *command, BSP_CAN_Frame_t *out);
/* No transmit. Validate a caller-supplied handle before multi-axis submission. */
bool DM_Motor_MatchesConfig(const DM_Motor_t *motor,const DM_Motor_Config_t *config);

/* SendMIT 的纯力矩便捷入口：kp=kd=0，p_des=v_des=0。
 * 无阻尼/限速，空载仍可持续加速；不自动使能，也不是校准后的力传感闭环。
 * 12bit 对称量化不能精确表示零力矩，0 指令仍有最多一个量化步长误差。 */
BSP_CAN_Status_t DM_Motor_SendTorque(DM_Motor_t *motor, float torque_nm);

/* 通过短临界区取得 ISR 更新的一致快照；不请求电机反馈。
 * 未收到有效反馈返回 NOT_READY，且不改写 feedback；参数错误同样不改写。
 * 成功不代表当前在线，调用者用 received_at_ms 判断时效，不内置离线阈值。
 * 此实例的 Master ID 接收通道用于运动反馈；不与并发参数读写应答混用。 */
BSP_CAN_Status_t DM_Motor_GetFeedback(const DM_Motor_t *motor,
                                     DM_Motor_Feedback_t *feedback);

#ifdef __cplusplus
}
#endif
#endif /* DEVICE_DM_MOTOR_H */
