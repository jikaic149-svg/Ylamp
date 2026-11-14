#include "dm_motor.h"

#include <stdbool.h>
#include <float.h>
#include <math.h>

#define DM_MOTOR_COUNT 16U
#define DM_KP_MAX 500.0f
#define DM_KD_MAX 5.0f

struct DM_Motor {
    bool initialized;
    DM_Motor_Config_t config;
    volatile bool feedback_valid;
    volatile DM_Motor_Feedback_t feedback;
};

static DM_Motor_t motors[DM_MOTOR_COUNT];

static bool valid_handle(const DM_Motor_t *motor)
{
    for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i) {
        if (motor == &motors[i]) { return motors[i].initialized; }
    }
    return false;
}

static bool valid_maximum(float value)
{
    /* 排除 M7 浮点 flush-to-zero 设置下可能被当作零除数的非正规量程。 */
    return isfinite(value) && value >= FLT_MIN;
}

static bool in_range(float value, float minimum, float maximum)
{
    return isfinite(value) && value >= minimum && value <= maximum;
}

static uint16_t encode_signed(float value, float maximum, uint16_t bits_max)
{
    /* 先除量程，避免 value+maximum 或 2*maximum 在异常量程下溢出。 */
    return (uint16_t)((value / maximum + 1.0f) * 0.5f * (float)bits_max);
}

static float decode_signed(uint16_t value, float maximum, uint16_t bits_max)
{
    return ((float)value / (float)bits_max * 2.0f - 1.0f) * maximum;
}

static void receive_feedback(FDCAN_HandleTypeDef *bus,
                              const BSP_CAN_Frame_t *frame, void *context)
{
    DM_Motor_t *motor = context;
    const DM_Motor_Config_t *config = &motor->config;
    if (bus != config->bus || frame->id != config->master_id ||
        frame->len != 8U || (frame->data[0] & 0x0FU) != config->can_id) {
        return;
    }
    const uint8_t *data = frame->data;
    /* GUI reads range registers; bench discovery also reads Master ID/RID7.
     * Their replies carry a little-endian motor ID + 0x33 + RID, not a
     * position/velocity sample. Do not publish them as fresh axis feedback.
     * Other parameter traffic is outside this feedback-only interface. */
    if (config->protocol == DM_MOTOR_PROTOCOL_V13 &&
        data[0] == config->can_id && data[1] == 0U && data[2] == 0x33U &&
        (data[3] == 7U || (data[3] >= 21U && data[3] <= 23U))) {
        return;
    }
    const uint16_t position = ((uint16_t)data[1] << 8) | data[2];
    const uint16_t velocity = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    const uint16_t torque = ((uint16_t)(data[4] & 0x0FU) << 8) | data[5];
    const DM_Motor_Feedback_t feedback = {
        .position_rad = decode_signed(position, config->pmax, UINT16_MAX),
        .velocity_rad_s = decode_signed(velocity, config->vmax, 0xFFFU),
        .torque_nm = decode_signed(torque, config->tmax, 0xFFFU),
        .state = data[0] >> 4,
        .mos_temperature_c = data[6],
        .rotor_temperature_c = data[7],
        .received_at_ms = HAL_GetTick(),
        .received_count = motor->feedback.received_count + 1U
    };
    motor->feedback = feedback;
    motor->feedback_valid = true;
}

BSP_CAN_Status_t DM_Motor_DefaultConfig(DM_Motor_Model_t model,
                                       DM_Motor_Config_t *config)
{
    if (config == NULL || __get_IPSR() != 0U ||
        (model != DM_MOTOR_DM4310 && model != DM_MOTOR_DM4340)) {
        return BSP_CAN_INVALID_ARG;
    }
    *config = (DM_Motor_Config_t){
        .model = model,
        .pmax = 12.5f,
        .vmax = model == DM_MOTOR_DM4310 ? 30.0f : 10.0f,
        .tmax = model == DM_MOTOR_DM4310 ? 10.0f : 28.0f
    };
    return BSP_CAN_OK;
}

BSP_CAN_Status_t DM_Motor_Init(const DM_Motor_Config_t *config,
                              DM_Motor_t **motor)
{
    if (config == NULL || motor == NULL || __get_IPSR() != 0U ||
        (config->model != DM_MOTOR_DM4310 && config->model != DM_MOTOR_DM4340) ||
        (config->protocol != DM_MOTOR_PROTOCOL_V13 &&
         config->protocol != DM_MOTOR_PROTOCOL_LEGACY_MIT) ||
        config->bus == NULL || config->can_id == 0U || config->can_id > 15U ||
        config->master_id == 0U || config->master_id >= BSP_CAN_MAX_STD_ID ||
        config->master_id == config->can_id ||
        !valid_maximum(config->pmax) || !valid_maximum(config->vmax) ||
        !valid_maximum(config->tmax)) {
        return BSP_CAN_INVALID_ARG;
    }
    DM_Motor_t *slot = NULL;
    for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i) {
        if (!motors[i].initialized) {
            if (slot == NULL) { slot = &motors[i]; }
        } else if (motors[i].config.bus == config->bus &&
                   (motors[i].config.can_id == config->can_id ||
                    motors[i].config.master_id == config->master_id ||
                    motors[i].config.master_id == config->can_id ||
                    motors[i].config.can_id == config->master_id)) {
            return BSP_CAN_BUSY;
        }
    }
    if (slot == NULL) { return BSP_CAN_NO_RESOURCE; }
    /* 注册之前准备好全部 ISR 上下文。失败时 BSP 不发布该回调。 */
    slot->config = *config;
    slot->feedback_valid = false;
    slot->feedback = (DM_Motor_Feedback_t){0};
    const BSP_CAN_Status_t result = BSP_CAN_RegisterRxCallback(
        config->bus, config->master_id, receive_feedback, slot);
    if (result != BSP_CAN_OK) { return result; }
    slot->initialized = true;
    *motor = slot;
    return BSP_CAN_OK;
}

static BSP_CAN_Status_t send_special(DM_Motor_t *motor, uint8_t command)
{
    if (!valid_handle(motor) || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    const BSP_CAN_Frame_t frame = {
        .id = motor->config.can_id,
        .data = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, command},
        .len = 8U
    };
    return BSP_CAN_Send(motor->config.bus, &frame);
}

BSP_CAN_Status_t DM_Motor_Enable(DM_Motor_t *motor)
{
    return send_special(motor, 0xFCU);
}

BSP_CAN_Status_t DM_Motor_Disable(DM_Motor_t *motor)
{
    return send_special(motor, 0xFDU);
}

BSP_CAN_Status_t DM_Motor_RequestFeedback(DM_Motor_t *motor)
{
    if (!valid_handle(motor) || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    return DM_Motor_ProbeReadOnly(motor->config.bus, motor->config.can_id, false);
}

BSP_CAN_Status_t DM_Motor_ProbeReadOnly(FDCAN_HandleTypeDef *bus,
                                       uint16_t can_id, bool read_master_id)
{
    if (can_id == 0U || can_id >= BSP_CAN_MAX_STD_ID || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    for (unsigned i = 0U; i < DM_MOTOR_COUNT; ++i) {
        if (motors[i].initialized && motors[i].config.bus == bus &&
            motors[i].config.can_id == can_id &&
            motors[i].config.protocol == DM_MOTOR_PROTOCOL_LEGACY_MIT) {
            return BSP_CAN_NOT_READY;
        }
    }
    const BSP_CAN_Frame_t frame = {
        .id = 0x7FFU, .len = 8U,
        .data = {(uint8_t)can_id, (uint8_t)(can_id >> 8),
                 read_master_id ? 0x33U : 0xCCU,
                 read_master_id ? 7U : 0U, 0U, 0U, 0U, 0U}
    };
    return BSP_CAN_Send(bus, &frame);
}

BSP_CAN_Status_t DM_Motor_SendMIT(DM_Motor_t *motor,
                                 const DM_Motor_MIT_Command_t *command)
{
    if (!valid_handle(motor) || command == NULL || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    const DM_Motor_Config_t *config = &motor->config;
    if (!in_range(command->position_rad, -config->pmax, config->pmax) ||
        !in_range(command->velocity_rad_s, -config->vmax, config->vmax) ||
        !in_range(command->torque_ff_nm, -config->tmax, config->tmax) ||
        !in_range(command->kp, 0.0f, DM_KP_MAX) ||
        !in_range(command->kd, 0.0f, DM_KD_MAX)) {
        return BSP_CAN_INVALID_ARG;
    }
    const uint16_t p = encode_signed(command->position_rad, config->pmax,
                                     UINT16_MAX);
    const uint16_t v = encode_signed(command->velocity_rad_s, config->vmax, 0xFFFU);
    const uint16_t t = encode_signed(command->torque_ff_nm, config->tmax, 0xFFFU);
    const uint16_t kp = (uint16_t)(command->kp / DM_KP_MAX * 4095.0f);
    const uint16_t kd = (uint16_t)(command->kd / DM_KD_MAX * 4095.0f);
    const BSP_CAN_Frame_t frame = {
        .id = config->can_id,
        .data = {(uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(v >> 4),
                 (uint8_t)((v << 4) | (kp >> 8)), (uint8_t)kp,
                 (uint8_t)(kd >> 4), (uint8_t)((kd << 4) | (t >> 8)), (uint8_t)t},
        .len = 8U
    };
    /* MIT 顶端量化值可能与清故障/使能/失能/设零命令同码，禁止误触发。 */
    bool special_prefix = true;
    for (unsigned i = 0U; i < 7U; ++i) {
        if (frame.data[i] != 0xFFU) { special_prefix = false; break; }
    }
    if (special_prefix && frame.data[7] >= 0xFBU && frame.data[7] <= 0xFEU) {
        return BSP_CAN_INVALID_ARG;
    }
    return BSP_CAN_Send(config->bus, &frame);
}

BSP_CAN_Status_t DM_Motor_SendTorque(DM_Motor_t *motor, float torque_nm)
{
    const DM_Motor_MIT_Command_t command = {.torque_ff_nm = torque_nm};
    return DM_Motor_SendMIT(motor, &command);
}

BSP_CAN_Status_t DM_Motor_GetFeedback(const DM_Motor_t *motor,
                                     DM_Motor_Feedback_t *feedback)
{
    if (!valid_handle(motor) || feedback == NULL || __get_IPSR() != 0U) {
        return BSP_CAN_INVALID_ARG;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool valid = motor->feedback_valid;
    if (valid) { *feedback = motor->feedback; }
    __set_PRIMASK(mask);
    return valid ? BSP_CAN_OK : BSP_CAN_NOT_READY;
}
