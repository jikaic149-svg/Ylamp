#ifndef YLAMP_MOTOR_CONFIG_H
#define YLAMP_MOTOR_CONFIG_H

#include <stdbool.h>
#include "dm_motor.h"
#include "robot_model.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 通信登记按 J1..J5 排列；控制/模型尚为 J1/J2，不能把这两个数量等同。 */
#define MOTOR_CAN_JOINT_COUNT 5U
#define MOTOR_TASK_JOINT_COUNT 2U
/* Explicit communication-check request 6 only; never starts at boot. */
#define MOTOR_CAN_CHECK_STREAM_MS 2000U
#if MOTOR_TASK_JOINT_COUNT != ROBOT_MODEL_JOINT_COUNT
#error "Provide a matching physical model before enabling more axes"
#endif
/* 自动恢复仅针对实际 CAN Bus-Off；同一故障簇最多三次，健康运行10s后复位预算。
 * 恢复后须两轴新鲜失能近静止持续100ms，再按当前姿态自动重入原拖动/保持。
 * KEY/STOP/F6 可取消自动出力；不改变上电仅查询或其他故障的人工解除规则。
 * 固定部署策略，不属于上位机38项RAM调参，毫秒时基可回绕。 */
#define MOTOR_CAN_RECOVERY_RETRY_MS 100U
#define MOTOR_CAN_RECOVERY_TIMEOUT_MS 1500U
#define MOTOR_CAN_RECOVERY_STABLE_MS 100U
#define MOTOR_CAN_RECOVERY_HEALTHY_MS 10000U
#define MOTOR_CAN_RECOVERY_ATTEMPTS 3U
/* Bit 0 (former runtime SPEED protection) is retired; keep other wire bits. */
#define MOTOR_PROTECT_TRACKING 2U
#define MOTOR_PROTECT_TEMP     4U
#define MOTOR_PROTECT_ALL      (MOTOR_PROTECT_TRACKING | MOTOR_PROTECT_TEMP)

typedef enum {
    MOTOR_STOP_UNCONFIRMED = 0,
    MOTOR_STOP_GRAVITY_HOLD
} MotorTask_StopMode_t;

typedef struct {
    DM_Motor_Config_t device; /* 与该电机读回的协议量程、编号一致。 */
    float lower_rad;         /* 标定参考圈的编码器下限；原始反馈不改写。 */
    float upper_rad;         /* 同一参考圈上限，区间宽度必须小于 2*pi。 */
    float hold_kp;            /* 仅退出拖动后的保持，N*m/rad；拖动时 Kp=0。 */
    float hold_kd;            /* 保持阻尼，N*m*s/rad。 */
    float tracking_error_rad; /* 仅停止/保持时检查，不阻止自由拖动。 */
    float start_speed_rad_s;            /* 使能前近静止检查，不是起动速度阈值。 */
    uint8_t temperature_c;
    uint8_t protection_flags; /* 跟随/过温开关；角度/失联/器件故障始终检查。 */
    float drag_kd_low;        /* 名义点参考力臂处的低速Kd；J1横向/J2竖向。 */
    float drag_kd_high;       /* 同上高速Kd，<=5；不是硬限速/总力矩限制。 */
    float drag_speed_low_rad_s;
    float drag_speed_high_rad_s; /* 乘参考力臂后为手端速度阈值，smoothstep。 */
    float drag_ff_limit_nm;   /* 全部FF总预算；重力/原动态项优先，不限内部PD。 */
    float drag_kd_slew_per_s; /* Kd 变化率，N*m*s/rad 每秒。 */
} MotorTask_JointConfig_t;

typedef struct {
    MotorTask_JointConfig_t joint[MOTOR_TASK_JOINT_COUNT];
    uint32_t control_period_ms;
    uint32_t query_period_ms;
    uint32_t feedback_max_age_ms;
    uint32_t enable_timeout_ms;
    uint32_t max_step_ms;
    uint32_t key_debounce_ms;
    uint32_t power_settle_ms;
    uint32_t status_period_ms;
    /* 未确认的行为不自动选择；任一未放行时 KEY 不会使能电机。 */
    MotorTask_StopMode_t stop_mode;
    bool fault_disable_approved;
    float drag_j2_zero_rad;   /* 公共朝上软件零点；保留旧调参字段名/编号，不属于拖动私有状态。 */
    uint8_t drag_reference_confirmed; /* 一次确认后沿用固件配置；不是 IMU/力矩实测通过标志。 */
    float drag_gravity_scale; /* 重力模型倍率，0..1.2，待实机核对。 */
    uint32_t drag_transition_ms; /* 建立重力支撑/切换保持刚度的时间。 */
    float drag_ff_slew_nm_s;
    float drag_velocity_filter_ms;
    float drag_acceleration_filter_ms; /* 速度差分后的加速度低通；不是目标轨迹加速度。 */
} MotorTask_Config_t;

/* 旧源码检查接口兼容别名；真实模型类型、数据和计算归Model。 */
typedef RobotModel_Parameters_t MotorTask_DragModel_t;
typedef RobotModel_Point_t MotorTask_GripPoint_t;
#define g_drag_model g_robot_model
#define g_drag_grip g_robot_tool_point

/* 固定部署参数，不扩展现有38项RAM调参协议。 */
typedef struct {
    float scale;             /* 同比例补偿 M*ddq_est+c；保留部分名义惯量。 */
    float torque_limit_nm[2]; /* 动态项单独限幅；重力优先占用总前馈预算。 */
    float slew_nm_s;         /* 动态项引入/撤出变化率。 */
    uint32_t feedback_max_age_ms; /* 超时只撤动态助力，不新增失能保护。 */
} MotorTask_DragDynamics_t;

/* 命令 ID=关节编号，反馈 ID=0x10+关节编号；只配置 MCU，不改写电机。
 * J1/J2的量程沿用既有配置；J3按旧固件截图核对，J4/J5仍为SDK初值待核对。 */
extern const DM_Motor_Config_t g_motor_can_config[MOTOR_CAN_JOINT_COUNT];
extern const MotorTask_Config_t g_motor_config;
/* 离线名义刚体模型；驱动惯量未知，CAD/编码器方向未验证。
 * 用于有限比例的拖动动态补偿，不是已辨识值或完整惯量抵消。 */
extern const MotorTask_DragDynamics_t g_drag_dynamics;

/* Position impedance is a different behavior, not Drag's hold/variable damping.
 * Gains are initial bench values, not identified actuator capability.
 * v/a/jerk are supplied explicitly with each point-to-point request. */
typedef struct {
    float kp[MOTOR_TASK_JOINT_COUNT];
    float kd[MOTOR_TASK_JOINT_COUNT];
    uint32_t enter_ms;
    float friction_smooth_rad_s; /* Smooth-zero FF, not a motion limit. */
    float friction_velocity_filter_ms; /* Encoder-position derivative low-pass time constant. */
    float velocity_limit_rad_s, acceleration_limit_rad_s2; /* Planning only. */
} MotionMode_Config_t;
extern const MotionMode_Config_t g_motion_config;

/* J5台架初值，不扩大双轴模型，也不代表已测机械能力。 */
typedef struct {
    float amplitude_rad, kp, kd, travel_limit_rad;
    uint32_t entry_ms, leg_ms, settle_ms;
} J5Test_Config_t;
extern const J5Test_Config_t g_j5_test_config;

#ifdef __cplusplus
}
#endif
#endif
