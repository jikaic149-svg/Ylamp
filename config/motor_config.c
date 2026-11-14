#include "motor_config.h"

#define DEG_RAD(degrees) ((degrees) * 0.01745329251994329577f)

/* 通信表和旧双轴控制共享设备初值，防止同一个关节的总线/ID/量程漂移。 */
#define J1_DEVICE_CONFIG { \
    .model = DM_MOTOR_DM4340, .bus = &hfdcan2, \
    .can_id = 0x01U, .master_id = 0x11U, \
    .pmax = 12.5f, .vmax = 10.0f, .tmax = 28.0f }
#define J2_DEVICE_CONFIG { \
    .model = DM_MOTOR_DM4340, .bus = &hfdcan2, \
    .can_id = 0x02U, .master_id = 0x12U, \
    .pmax = 12.5f, .vmax = 10.0f, .tmax = 28.0f }

/* V1.2：CAN2 / PC13 配 J1/J2，CAN1 / PC14 配 J3/J4/J5。
 * J3的03/13和量程按5011/007截图核对，旧协议回包已实测。
 * J4/J5量程沿用SDK初值；只配置MCU，不写电机参数或表示机械能力。 */
const DM_Motor_Config_t g_motor_can_config[MOTOR_CAN_JOINT_COUNT] = {
    J1_DEVICE_CONFIG,
    J2_DEVICE_CONFIG,
    {
        .model = DM_MOTOR_DM4310, .bus = &hfdcan1,
        .can_id = 0x03U, .master_id = 0x13U,
        .pmax = 12.5f, .vmax = 30.0f, .tmax = 10.0f,
        .protocol = DM_MOTOR_PROTOCOL_LEGACY_MIT /* J3: 5011/007，旧协议已实测。 */
    },
    {
        .model = DM_MOTOR_DM4310, .bus = &hfdcan1,
        .can_id = 0x04U, .master_id = 0x14U,
        .pmax = 12.5f, .vmax = 30.0f, .tmax = 10.0f
    },
    {
        .model = DM_MOTOR_DM4310, .bus = &hfdcan1,
        .can_id = 0x05U, .master_id = 0x15U,
        .pmax = 12.5f, .vmax = 30.0f, .tmax = 10.0f
    }
};

/* 双 DM4340P，原始双编码器，24V / CAN2 / MIT。
 * 仅拖动及原位保持；已删除旧往返、方向前馈和起动助力。
 * 数值均为待实机验证的初值；运行 SPEED 超速保护已删除，速度仍用于阻尼。
 * 故障失能会失去重力支撑，须有独立支撑/急停；软件不是安全认证。 */
const MotorTask_Config_t g_motor_config = {
    .joint = {
        {
            .device = J1_DEVICE_CONFIG,
            .lower_rad = DEG_RAD(-210.0f), .upper_rad = DEG_RAD(0.0f),
            .hold_kp = 5.0f, .hold_kd = 5.0f,
            .tracking_error_rad = DEG_RAD(5.0f),
            .start_speed_rad_s = DEG_RAD(1.0f), .temperature_c = 60U,
            .protection_flags = MOTOR_PROTECT_ALL,
            /* Free drag: soft below 20 deg/s, full damping by 60 deg/s.
             * Positive Kd with v_des=0 opposes feedback velocity, never assists it. */
            .drag_kd_low = 0.005f, .drag_kd_high = 1.2f,
            .drag_speed_low_rad_s = DEG_RAD(20.0f),
            .drag_speed_high_rad_s = DEG_RAD(60.0f),
            .drag_ff_limit_nm = 0.2f, .drag_kd_slew_per_s = 30.0f
        },
        {
            .device = J2_DEVICE_CONFIG,
            /* 用户当前原始读数约 160°..300°；在标定参考圈等价 -200°..-60°。
             * 暂定软件工作区间，不是已测量的硬限位；保留原朝上软件零点。 */
            .lower_rad = DEG_RAD(-200.0f), .upper_rad = DEG_RAD(-60.0f),
            .hold_kp = 5.0f, .hold_kd = 5.0f,
            .tracking_error_rad = DEG_RAD(5.0f),
            .start_speed_rad_s = DEG_RAD(1.0f), .temperature_c = 60U,
            .protection_flags = MOTOR_PROTECT_ALL,
            .drag_kd_low = 0.01f, .drag_kd_high = 1.8f,
            .drag_speed_low_rad_s = DEG_RAD(20.0f),
            .drag_speed_high_rad_s = DEG_RAD(60.0f),
            .drag_ff_limit_nm = 0.7f, .drag_kd_slew_per_s = 30.0f
        }
    },
    .control_period_ms = 5U, .query_period_ms = 10U,
    .feedback_max_age_ms = 100U, .enable_timeout_ms = 500U,
    .max_step_ms = 20U, .key_debounce_ms = 20U,
    .power_settle_ms = 1000U, .status_period_ms = 500U,
    .stop_mode = MOTOR_STOP_GRAVITY_HOLD, .fault_disable_approved = true,
    /* 用户已确认的朝上参考 -151.2828024927°，随固件保存，不逐次开机确认。
     * 来源是人工记录；不代表 IMU 复核或实际补偿方向试验已经通过。 */
    .drag_j2_zero_rad = -2.640383005142212f,
    .drag_reference_confirmed = 1U,
    .drag_gravity_scale = 1.0f,
    .drag_transition_ms = 1000U,
    .drag_ff_slew_nm_s = 1.0f,
    .drag_velocity_filter_ms = 10.0f,
    .drag_acceleration_filter_ms = 100.0f
};

/* 手动拖动没有 ddq_des。用滤波后的实测加速度做部分惯量反馈：
 * tau_dynamic = 0.25 * (M(q2)*ddq_est + c(q2,dq))。
 * 理想无延迟/不限幅且模型准确时保留75%名义惯量；这不是稳定性保证。
 * 不含摩擦补偿，也不使用电流估算转矩做助推。 */
const MotorTask_DragDynamics_t g_drag_dynamics = {
    .scale = 0.25f,
    .torque_limit_nm = {0.10f, 0.15f},
    .slew_nm_s = 0.5f,
    .feedback_max_age_ms = 20U
};

/* MIT spring/damper: kp*(p_des-p) + kd*(v_des-v) + model FF.
 * Separate from Drag; friction uses differentiated, low-pass filtered encoder
 * position, including hand motion around a fixed target. No static integral.
 * Starting mode remains DRAG; these gains never enable a motor by themselves. */
const MotionMode_Config_t g_motion_config = {
    .kp = {5.0f, 5.0f},
    .kd = {0.5f, 0.5f},
    .enter_ms = 1000U,
    .friction_smooth_rad_s = 0.02f,
    .friction_velocity_filter_ms = 50.0f,
    /* 用户2026-11-09确认：160rpm、±2rad/s²；另受实际MIT编码量程约束。
     * 只约束新轨迹，不恢复实际速度瞬时跳闸。 */
    .velocity_limit_rad_s = 16.75516082f,
    .acceleration_limit_rad_s2 = 2.0f
};

/* 用户2026-11-14要求：Kp=10，+30°/2s，再用2s返回；
 * 两端速度/加速度为零，参考峰速约28.13°/s。
 * 33°仅为意外偏移终止阈值，不是已确认的机械行程。需外部支撑。 */
const J5Test_Config_t g_j5_test_config = {
    .amplitude_rad = DEG_RAD(30.0f), .kp = 10.0f, .kd = 0.5f,
    .travel_limit_rad = DEG_RAD(33.0f),
    .entry_ms = 1000U, .leg_ms = 2000U, .settle_ms = 1000U
};
