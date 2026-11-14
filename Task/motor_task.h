#ifndef YLAMP_MOTOR_TASK_H
#define YLAMP_MOTOR_TASK_H

#include "motor_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_TASK_IDLE = 0, MOTOR_TASK_ARMING, MOTOR_TASK_RUNNING,
    MOTOR_TASK_STOPPING, MOTOR_TASK_HOLDING, MOTOR_TASK_FAULT,
    MOTOR_TASK_RECOVERING
} MotorTask_State_t;

typedef enum {
    MOTOR_REASON_NONE = 0, MOTOR_REASON_POLICY, MOTOR_REASON_NO_FEEDBACK,
    MOTOR_REASON_STALE, MOTOR_REASON_ANGLE, MOTOR_REASON_STATE,
    MOTOR_REASON_TEMPERATURE, MOTOR_REASON_NOT_STILL, MOTOR_REASON_TRACKING,
    MOTOR_REASON_TIMING, MOTOR_REASON_TRANSPORT, MOTOR_REASON_ENABLE_TIMEOUT,
    MOTOR_REASON_MANUAL_DISABLE
} MotorTask_Reason_t;

typedef struct {
    MotorTask_State_t state;
    MotorTask_Reason_t reason;
    bool policy_ready;
    uint8_t feedback_valid_mask; /* 曾收到有效反馈，不代表当前新鲜。 */
    uint8_t disable_submitted_mask; /* 提交成功不是对端已失能。 */
    BSP_CAN_Status_t last_transport;
    BSP_CAN_Status_t disable_status[MOTOR_TASK_JOINT_COUNT];
    DM_Motor_Feedback_t feedback[MOTOR_TASK_JOINT_COUNT]; /* Unmodified wire/raw frame. */
    float target_rad[MOTOR_TASK_JOINT_COUNT]; /* Actual MIT wire/raw target. */
    float target_velocity_rad_s[MOTOR_TASK_JOINT_COUNT];
    float target_torque_ff_nm[MOTOR_TASK_JOINT_COUNT];
    float target_kp[MOTOR_TASK_JOINT_COUNT];
    float target_kd[MOTOR_TASK_JOINT_COUNT];
    DragMode_Status_t drag; /* 名义模型，不是力传感器测量。 */
    MotionMode_Status_t motion;
    uint32_t mode_generation; /* Draft invalidation across reset/stop/fault. */
    MotorMode_Id_t mode;
    MotorMode_Phase_t mode_phase; /* 公共生命周期，Task不解释DragMode私有阶段。 */
    float reference_rad[MOTOR_TASK_JOINT_COUNT]; /* Raw + integer-turn offset. */
    float reference_offset_rad[MOTOR_TASK_JOINT_COUNT];
    float target_reference_rad[MOTOR_TASK_JOINT_COUNT];
} MotorTask_Status_t;

/* 纯区间工具，留一位置量化步长；无效输入不改输出，不裁剪原始反馈。
 * 不用于发送端逐轴修改轨迹；Motion应在接受/规划时整体验证可执行性。 */
BSP_CAN_Status_t MotorTask_ClampTarget(const MotorTask_JointConfig_t *config,
                                      float requested, float *target);
/* 单实例，复制配置。所有接口仅同一普通任务调用，不允许 ISR。
 * Init 不发送、不使能、不操作电源，不支持重复初始化。 */
BSP_CAN_Status_t MotorTask_Init(const MotorTask_Config_t *config,
                               DM_Motor_t *const motors[MOTOR_TASK_JOINT_COUNT],
                               uint32_t now_ms);
/* 1..5ms 调用；内部5ms双轴快照->计算->顺序提交，无积压补发。
 * 活动阶段以 MIT 应答更新反馈；IDLE/FAULT/RECOVERING 按 query_period_ms 查询，
 * 恢复期间还须等待 BSP 开放发送，不积压补发查询。
 * KEY: 开始拖动 / 停止转保持 / 继续拖动；恢复期间KEY取消自动出力。
 * 返回值：FAULT/RECOVERING 均为 ERROR，即使本次尚未到控制周期。
 * 实际Bus-Off按config限次恢复、清旧帧、核对失能近静止后自动重入故障前行为；
 * 上电空闲/人工失能不自动启动，非通信故障仍锁定。STOP/F6也取消自动重入。
 * 正常停止保留重力支撑，F6/故障失能失去支撑；不是独立硬件急停。 */
BSP_CAN_Status_t MotorTask_Run(uint32_t now_ms);
BSP_CAN_Status_t MotorTask_GetStatus(MotorTask_Status_t *status);
BSP_CAN_Status_t MotorTask_GetConfig(MotorTask_Config_t *config);
/* Inactive-only selection; no enable/output or implicit target. */
BSP_CAN_Status_t MotorTask_SelectMode(MotorMode_Id_t mode);
/* Explicit one-button action: capture fresh reference angles once and enter
 * Motion impedance. Enables through the existing ACK handshake if idle; a
 * live Mode handoff does not disable/re-enable. No v/a/jerk or move implied.
 * Manual-disable may be cleared by this explicit action, not other faults. */
BSP_CAN_Status_t MotorTask_CaptureHold(uint32_t now_ms);
/* Single dual-axis job, only Motion HOLDING accepts it. Infeasible jobs leave
 * the existing hold intact, without a fault. Reference encoder rad, not model
 * zero. No implicit enable, repetition or replay after communication loss. */
BSP_CAN_Status_t MotorTask_Move(const MotionMode_Request_t *request, uint32_t now_ms);
/* 整组校验，仅 IDLE/FAULT、两轴新鲜失能近静止时允许；不使能/清故障。
 * Device身份/量程、调度及退出策略不可由调参包修改。 */
BSP_CAN_Status_t MotorTask_ApplyConfig(const MotorTask_Config_t *config,
                                      uint32_t now_ms);
typedef enum {
    MOTOR_COMMAND_START = 1, MOTOR_COMMAND_STOP,
    MOTOR_COMMAND_DISABLE, MOTOR_COMMAND_CLEAR
} MotorTask_Command_t;
BSP_CAN_Status_t MotorTask_Command(MotorTask_Command_t command, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
