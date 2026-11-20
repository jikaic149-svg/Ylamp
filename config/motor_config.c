#include "motor_config.h"
#include "4310/LegacyMIT/dm4310_legacy.h"
#include "4310/V13/dm4310_v13.h"
#include "4340/dm4340.h"

#define DEG_RAD(degrees) ((degrees) * 0.01745329251994329577f)

/* Single source for the monitor and J2 control/gravity defaults. */
#define JOINT_REFERENCE(joint, zero, lower, upper, kind) \
    [(joint) - 1U] = {DEG_RAD(zero), DEG_RAD(lower), DEG_RAD(upper), \
        MOTOR_REFERENCE_##kind},
const MotorJointReference_Config_t g_joint_reference_config[MOTOR_CAN_JOINT_COUNT] = {
#include "joint_references.def"
};
#undef JOINT_REFERENCE

/* 通信表和旧双轴控制共享设备初值，防止同一个关节的总线/ID/量程漂移。 */
#define J1_DEVICE_CONFIG { \
    DM4340_CONFIG_FIELDS, .bus = &hfdcan2, \
    .can_id = 0x01U, .master_id = 0x11U }
#define J2_DEVICE_CONFIG { \
    DM4340_CONFIG_FIELDS, .bus = &hfdcan2, \
    .can_id = 0x02U, .master_id = 0x12U }

/* V1.2：CAN2 / PC13 配 J1/J2，CAN1 / PC14 配 J3/J4/J5。
 * J3的03/13和量程按5011/007截图核对，旧协议回包已实测。
 * J4/J5量程沿用SDK初值；只配置MCU，不写电机参数或表示机械能力。 */
const DM_Motor_Config_t g_motor_can_config[MOTOR_CAN_JOINT_COUNT] = {
    J1_DEVICE_CONFIG,
    J2_DEVICE_CONFIG,
    {
        DM4310_LEGACY_CONFIG_FIELDS, .bus = &hfdcan1,
        .can_id = 0x03U, .master_id = 0x13U /* J3: 5011/007，旧协议已实测。 */
    },
    {
        DM4310_V13_CONFIG_FIELDS, .bus = &hfdcan1,
        .can_id = 0x04U, .master_id = 0x14U
    },
    {
        DM4310_V13_CONFIG_FIELDS, .bus = &hfdcan1,
        .can_id = 0x05U, .master_id = 0x15U
    }
};

