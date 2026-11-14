/* 拖拽模式：公共Model -> 拖动阻尼/运动估计 -> 生命周期 -> 双轴MIT目标。
 * 仅拖拽策略收拢在本文件；物理事实归Model，硬件提交与公共门控留在Task。
 * 不包含主动轨迹、摩擦补偿、使能/失能或故障自动恢复逻辑。 */
#include "drag_mode.h"

#include <math.h>
#include <string.h>

/* 1. 拖动专属手端阻尼，不放进公共物理模型。 */
static float scheduled_kd(const MotorTask_JointConfig_t *joint, float speed)
{
    float x = (speed - joint->drag_speed_low_rad_s) /
        (joint->drag_speed_high_rad_s - joint->drag_speed_low_rad_s);
    x = fminf(1.0f, fmaxf(0.0f, x));
    return joint->drag_kd_low +
        (joint->drag_kd_high - joint->drag_kd_low) * x*x*(3.0f - 2.0f*x);
}

bool DragGeometry_Damping(const MotorTask_Config_t *config,
                          const DragGeometry_t *g, const float v[2], float d[3])
{
    if (config == NULL || g == NULL || v == NULL || d == NULL ||
        !isfinite(v[0]) || !isfinite(v[1])) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *j = &config->joint[i];
        if (!isfinite(j->drag_kd_low) || !isfinite(j->drag_kd_high) ||
            j->drag_kd_low < 0.0f || j->drag_kd_high < j->drag_kd_low ||
            j->drag_kd_high > 5.0f || !isfinite(j->drag_speed_low_rad_s) ||
            !isfinite(j->drag_speed_high_rad_s) || j->drag_speed_low_rad_s < 0.0f ||
            j->drag_speed_high_rad_s <= j->drag_speed_low_rad_s) { return false; }
    }
    float velocity[3];
    for (unsigned k = 0U; k < 3U; ++k) {
        velocity[k] = g->jacobian[k][0]*v[0] + g->jacobian[k][1]*v[1];
    }
    const float rx = g->reference_radius_m[0], rz = g->reference_radius_m[1];
    if (!isfinite(rx) || !isfinite(rz) || rx < 0.001f || rz < 0.001f) {
        return false;
    }
    const float bxy = scheduled_kd(&config->joint[0],
        hypotf(velocity[0], velocity[1]) / rx) / (rx*rx);
    const float bz = scheduled_kd(&config->joint[1], fabsf(velocity[2]) / rz) / (rz*rz);
    float result[3] = {0};
    for (unsigned k = 0U; k < 3U; ++k) {
        const float b = k == 2U ? bz : bxy;
        result[0] += b*g->jacobian[k][0]*g->jacobian[k][0];
        result[1] += b*g->jacobian[k][0]*g->jacobian[k][1];
        result[2] += b*g->jacobian[k][1]*g->jacobian[k][1];
    }
    for (unsigned k = 0U; k < 3U; ++k) {
        if (!isfinite(result[k])) { return false; }
    }
    if (result[0] < 0.0f || result[0] > 5.0f + 1e-5f ||
        result[2] < 0.0f || result[2] > 5.0f + 1e-5f) { return false; }
    result[0] = fminf(5.0f, result[0]);
    result[2] = fminf(5.0f, result[2]);
    for (unsigned k = 0U; k < 3U; ++k) { d[k] = result[k]; }
    return true;
}

/* 2. 平滑切换、前馈预算与采样估计。 */
static float approach(float from, float to, float step)
{
    return from + fminf(step, fmaxf(-step, to - from));
}

static float smooth(float value)
{
    value = fminf(1.0f, fmaxf(0.0f, value));
    return value * value * (3.0f - 2.0f * value);
}

static float clamp(float value, float lower, float upper)
{
    return fminf(upper, fmaxf(lower, value));
}

/* MIT Kd occupies the diagonal. Only -D12*dq_other uses FF.
 * Reduce BOTH cross terms by one factor; separate per-axis clipping can turn
 * a symmetric dissipative matrix into an active one. Never displace base FF.
 * This is a sampled model property, not a passivity guarantee on real motors. */
static void add_cross_damping(const MotorTask_Config_t *config,
    const DM_Motor_Feedback_t fb[2], const float desired[3], float blend,
    DragMode_Status_t *status, DM_Motor_MIT_Command_t output[2])
{
    float wire_kd[2];
    for (unsigned i = 0U; i < 2U; ++i) {
        /* Match Device's truncation and operation order, including one-LSB edges. */
        wire_kd[i] = (float)(uint16_t)(output[i].kd / 5.0f * 4095.0f) / 4095.0f * 5.0f;
    }
    const float denominator = sqrtf(desired[0] * desired[2]);
    const float correlation = denominator > 1e-12f ?
        clamp(desired[1] / denominator, -1.0f, 1.0f) : 0.0f;
    const float cross = blend * correlation * sqrtf(wire_kd[0] * wire_kd[1]);
    const float requested[2] = {-cross * fb[1].velocity_rad_s,
                                -cross * fb[0].velocity_rad_s};
    float scale = 1.0f;
    for (unsigned i = 0U; i < 2U; ++i) {
        const float limit = config->joint[i].drag_ff_limit_nm;
        const float room = requested[i] >= 0.0f ?
            limit - output[i].torque_ff_nm : limit + output[i].torque_ff_nm;
        if (fabsf(requested[i]) > 0.0f) {
            scale = fminf(scale, fmaxf(0.0f, room) / fabsf(requested[i]));
        }
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        const float base = output[i].torque_ff_nm;
        const float limit = config->joint[i].drag_ff_limit_nm;
        /* Final clamp only catches floating-point boundary rounding. */
        output[i].torque_ff_nm = clamp(base + scale * requested[i], -limit, limit);
        status->grip_ff_nm[i] = output[i].torque_ff_nm - base;
        status->damping[i == 0U ? 0U : 2U] = wire_kd[i];
    }
    status->damping[1] = scale * cross;
}

/* 只按新反馈的采样间隔求导，不按控制周期反复差分旧速度。
 * 丢样/时间倒退/间隔过大时清掉导数，恢复后至少等一次正常差分。 */
static void update_motion_estimate(const MotorTask_Config_t *config,
    const DM_Motor_Feedback_t *fb, unsigned i, float dt_s, DragMode_State_t *next)
{
    const uint32_t elapsed_ms = fb->received_at_ms - next->sample_ms[i];
    const float max_age_s = g_drag_dynamics.feedback_max_age_ms * 0.001f;
    const bool new_sample = fb->received_count != next->sample_count[i] &&
                           elapsed_ms > 0U;
    next->sample_age_s[i] += dt_s;
    if (!next->initialized || (new_sample &&
        elapsed_ms > g_drag_dynamics.feedback_max_age_ms)) {
        next->velocity[i] = fb->velocity_rad_s;
        next->acceleration[i] = 0.0f;
        next->derivative_ready[i] = false;
        next->sample_age_s[i] = 0.0f;
        next->sample_ms[i] = fb->received_at_ms;
        next->sample_count[i] = fb->received_count;
        if (!next->initialized) { next->kd[i] = config->joint[i].drag_kd_low; }
    } else if (new_sample) {
        const float sample_s = (float)elapsed_ms * 0.001f;
        const float alpha = sample_s /
            (config->drag_velocity_filter_ms * 0.001f + sample_s);
        const float previous = next->velocity[i];
        next->velocity[i] += alpha * (fb->velocity_rad_s - previous);
        const float acceleration = (next->velocity[i] - previous) / sample_s;
        const float accel_alpha = sample_s /
            (config->drag_acceleration_filter_ms * 0.001f + sample_s);
        next->acceleration[i] += accel_alpha * (acceleration - next->acceleration[i]);
        next->derivative_ready[i] = true;
        next->sample_age_s[i] = 0.0f;
        next->sample_ms[i] = fb->received_at_ms;
        next->sample_count[i] = fb->received_count;
    }
    if (next->sample_age_s[i] > max_age_s) {
        next->sample_age_s[i] = max_age_s + dt_s;
        next->acceleration[i] = 0.0f;
        next->derivative_ready[i] = false;
    }
}

/* 3. 生命周期。Reset仅清算法状态，不代表电机已失能。 */
void DragMode_Reset(DragMode_State_t *state)
{
    if (state != NULL) { memset(state, 0, sizeof(*state)); }
}

bool DragMode_ValidConfig(const MotorTask_Config_t *config)
{
    if (config == NULL ||
        config->drag_reference_confirmed > 1U ||
        !isfinite(config->drag_j2_zero_rad) ||
        config->drag_j2_zero_rad < -config->joint[1].device.pmax ||
        config->drag_j2_zero_rad > config->joint[1].device.pmax ||
        !isfinite(config->drag_gravity_scale) ||
        config->drag_gravity_scale < 0.0f || config->drag_gravity_scale > 1.2f ||
        config->drag_transition_ms < 100U || config->drag_transition_ms > 10000U ||
        !isfinite(config->drag_ff_slew_nm_s) ||
        config->drag_ff_slew_nm_s <= 0.0f || config->drag_ff_slew_nm_s > 100.0f ||
        !isfinite(config->drag_velocity_filter_ms) ||
        config->drag_velocity_filter_ms < 0.0f ||
        config->drag_velocity_filter_ms > 1000.0f ||
        !isfinite(config->drag_acceleration_filter_ms) ||
        config->drag_acceleration_filter_ms < 1.0f ||
        config->drag_acceleration_filter_ms > 1000.0f) {
        return false;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *joint = &config->joint[i];
        if (!isfinite(joint->device.pmax) || joint->device.pmax <= 0.0f ||
            !isfinite(joint->device.vmax) || joint->device.vmax <= 0.0f ||
            !isfinite(joint->device.tmax) || joint->device.tmax <= 0.0f ||
            !isfinite(joint->lower_rad) || !isfinite(joint->upper_rad) ||
            joint->lower_rad < -joint->device.pmax ||
            joint->upper_rad > joint->device.pmax ||
            joint->upper_rad - joint->lower_rad <= 8.0f * joint->device.pmax / 65535.0f ||
            !isfinite(joint->start_speed_rad_s) || joint->start_speed_rad_s <= 0.0f ||
            !isfinite(joint->drag_kd_low) || !isfinite(joint->drag_kd_high) ||
            joint->drag_kd_low < 0.0f || joint->drag_kd_high > 5.0f ||
            joint->drag_kd_high < joint->drag_kd_low ||
            !isfinite(joint->drag_speed_low_rad_s) ||
            !isfinite(joint->drag_speed_high_rad_s) ||
            joint->drag_speed_low_rad_s < 0.0f ||
            joint->drag_speed_high_rad_s <= joint->drag_speed_low_rad_s ||
            joint->drag_speed_high_rad_s > joint->device.vmax ||
            !isfinite(joint->drag_ff_limit_nm) || joint->drag_ff_limit_nm < 0.0f ||
            joint->drag_ff_limit_nm > joint->device.tmax ||
            !isfinite(joint->drag_kd_slew_per_s) ||
            joint->drag_kd_slew_per_s <= 0.0f || joint->drag_kd_slew_per_s > 1000.0f ||
            !isfinite(joint->hold_kp) || joint->hold_kp <= 0.0f ||
            joint->hold_kp > 500.0f || !isfinite(joint->hold_kd) ||
            joint->hold_kd < 0.0f || joint->hold_kd > 5.0f) {
            return false;
        }
    }
    return true;
}

static bool valid_feedback(const MotorTask_Config_t *config,
                            const DM_Motor_Feedback_t feedback[2])
{
    if (feedback == NULL) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!isfinite(feedback[i].position_rad) ||
            fabsf(feedback[i].position_rad) > config->joint[i].device.pmax ||
            !isfinite(feedback[i].velocity_rad_s) ||
            fabsf(feedback[i].velocity_rad_s) > config->joint[i].device.vmax) {
            return false;
        }
    }
    return true;
}

bool DragMode_Enter(const MotorTask_Config_t *config,
                    const DM_Motor_Feedback_t feedback[2], DragMode_State_t *state)
{
    if (state == NULL || !DragMode_ValidConfig(config) ||
        !valid_feedback(config, feedback) || state->phase != DRAG_MODE_OFF) {
        return false;
    }
    DragMode_Reset(state);
    state->phase = DRAG_MODE_ENTERING;
    return true;
}

bool DragMode_Stop(const MotorTask_Config_t *config,
                   const DM_Motor_Feedback_t feedback[2], DragMode_State_t *state)
{
    if (state == NULL || !DragMode_ValidConfig(config) ||
        !valid_feedback(config, feedback) || state->phase == DRAG_MODE_OFF) {
        return false;
    }
    if (state->phase == DRAG_MODE_STOPPING || state->phase == DRAG_MODE_HOLDING) {
        return true;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *joint = &config->joint[i];
        const float quantum = 2.0f * joint->device.pmax / 65535.0f;
        state->hold_target_rad[i] = fminf(joint->upper_rad - quantum,
            fmaxf(joint->lower_rad + quantum, feedback[i].position_rad));
    }
    state->phase = DRAG_MODE_STOPPING;
    return true;
}

bool DragMode_Resume(DragMode_State_t *state)
{
    if (state == NULL || state->phase != DRAG_MODE_HOLDING) { return false; }
    state->phase = DRAG_MODE_RESUMING;
    return true;
}

/* 4. 本周期双轴目标：先完整计算，全部成功后一起提交给调用者。 */
bool DragMode_Update(const MotorTask_Config_t *config,
                     const DM_Motor_Feedback_t feedback[2], float dt_s,
                     DragMode_State_t *state, DragMode_Status_t *status,
                     DM_Motor_MIT_Command_t commands[2])
{
    if (!DragMode_ValidConfig(config) || feedback == NULL || state == NULL ||
        status == NULL || commands == NULL ||
        !valid_feedback(config, feedback) || state->phase > DRAG_MODE_RESUMING ||
        !isfinite(dt_s) || dt_s < 0.0f ||
        dt_s > (float)config->max_step_ms * 0.001f) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!isfinite(state->hold_target_rad[i]) ||
            fabsf(state->hold_target_rad[i]) > config->joint[i].device.pmax) { return false; }
    }
    DragMode_State_t next = *state;
    const bool enabled = next.phase != DRAG_MODE_OFF;
    const bool holding = next.phase == DRAG_MODE_STOPPING ||
                         next.phase == DRAG_MODE_HOLDING;
    DragMode_Status_t result = {0};
    DM_Motor_MIT_Command_t output[2] = {0};
    for (unsigned i = 0U; i < 2U; ++i) {
        update_motion_estimate(config, &feedback[i], i, dt_s, &next);
        if (!isfinite(next.velocity[i]) || !isfinite(next.acceleration[i])) { return false; }
        result.filtered_velocity_rad_s[i] = next.velocity[i];
        result.acceleration_rad_s2[i] = next.acceleration[i];
    }
    next.initialized = true;
    const float q = feedback[1].position_rad - config->drag_j2_zero_rad;
    DragGeometry_t geometry;
    float damping[3];
    if (!RobotModel_Geometry(&g_robot_tool_point, feedback[0].position_rad, q, &geometry) ||
        !DragGeometry_Damping(config, &geometry, next.velocity, damping)) { return false; }
    memcpy(result.grip_radius_m, geometry.radius_m, sizeof(result.grip_radius_m));
    RobotModel_Dynamics_t dynamics;
    if (!RobotModel_Dynamics(&g_robot_model, q, next.velocity,
                             next.acceleration, &dynamics)) { return false; }
    result.q2_rad = q;
    memcpy(result.inertia, dynamics.inertia, sizeof(result.inertia));
    memcpy(result.velocity_coupling_nm, dynamics.coupling_nm,
           sizeof(result.velocity_coupling_nm));
    memcpy(result.inertial_nm, dynamics.inertial_nm, sizeof(result.inertial_nm));
    result.gravity_nm[1] = config->drag_gravity_scale * dynamics.gravity_nm[1];
    const bool dynamics_ready = next.derivative_ready[0] && next.derivative_ready[1];
    const bool free_drag = next.phase == DRAG_MODE_DRAGGING;
    const float blend_step = dt_s * 1000.0f / (float)config->drag_transition_ms;
    next.support_blend = enabled ? approach(next.support_blend, 1.0f, blend_step) : 0.0f;
    next.hold_blend = enabled ? approach(next.hold_blend, holding ? 1.0f : 0.0f,
                                         blend_step) : 0.0f;
    result.hold_blend = next.hold_blend;
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *joint = &config->joint[i];
        float kd = damping[i == 0U ? 0U : 2U];
        const float hold = smooth(next.hold_blend);
        kd += hold * (joint->hold_kd - kd);
        next.kd[i] = approach(next.kd[i], kd, joint->drag_kd_slew_per_s * dt_s);
        /* 重力支撑与动态助力分开限斜率。动态项不能挤掉重力预算。 */
        const float gravity = clamp(smooth(next.support_blend) * result.gravity_nm[i],
                                    -joint->drag_ff_limit_nm, joint->drag_ff_limit_nm);
        next.gravity_ff[i] = enabled ? approach(next.gravity_ff[i], gravity,
            config->drag_ff_slew_nm_s * dt_s) : 0.0f;
        next.gravity_ff[i] = clamp(next.gravity_ff[i],
                                  -joint->drag_ff_limit_nm, joint->drag_ff_limit_nm);
        const float dynamic_limit = g_drag_dynamics.torque_limit_nm[i];
        const float desired_dynamic = free_drag && dynamics_ready ?
            clamp(g_drag_dynamics.scale *
                (result.inertial_nm[i] + result.velocity_coupling_nm[i]),
                -dynamic_limit, dynamic_limit) : 0.0f;
        next.dynamic_ff[i] = enabled && dynamics_ready ?
            approach(next.dynamic_ff[i], desired_dynamic,
                     g_drag_dynamics.slew_nm_s * dt_s) : 0.0f;
        next.dynamic_ff[i] = clamp(next.dynamic_ff[i],
            fmaxf(-dynamic_limit, -joint->drag_ff_limit_nm - next.gravity_ff[i]),
            fminf(dynamic_limit, joint->drag_ff_limit_nm - next.gravity_ff[i]));
        result.dynamic_ff_nm[i] = next.dynamic_ff[i];
        output[i] = (DM_Motor_MIT_Command_t){
            .position_rad = holding || next.phase == DRAG_MODE_RESUMING ?
                next.hold_target_rad[i] : feedback[i].position_rad,
            .velocity_rad_s = 0.0f,
            .kp = enabled ? hold * joint->hold_kp : 0.0f,
            .kd = next.kd[i],
            .torque_ff_nm = next.gravity_ff[i] + next.dynamic_ff[i]
        };
        /* 拖动/捕获保持点在Mode内生成合法目标；这不是轨迹规划器。
         * 保留原量化边距，不让Task的逐轴裁剪成为未来Motion的隐式规划。 */
        const float quantum = joint->device.pmax / 65535.0f * 2.0f;
        output[i].position_rad = clamp(output[i].position_rad,
            joint->lower_rad + quantum, joint->upper_rad - quantum);
        if (!isfinite(result.gravity_nm[i]) || !isfinite(result.velocity_coupling_nm[i]) ||
            !isfinite(result.inertial_nm[i]) || !isfinite(output[i].kp) ||
            !isfinite(output[i].kd) || !isfinite(output[i].torque_ff_nm)) { return false; }
    }
    const bool cross_fresh = next.sample_age_s[0] <=
        g_drag_dynamics.feedback_max_age_ms * 0.001f &&
        next.sample_age_s[1] <= g_drag_dynamics.feedback_max_age_ms * 0.001f;
    const float cross_blend = enabled && cross_fresh ?
        smooth(next.support_blend) * (1.0f - smooth(next.hold_blend)) : 0.0f;
    add_cross_damping(config, feedback, damping, cross_blend, &result, output);
    if (next.phase == DRAG_MODE_ENTERING && next.support_blend >= 1.0f) {
        next.phase = DRAG_MODE_DRAGGING;
    } else if (next.phase == DRAG_MODE_RESUMING && next.hold_blend <= 0.0f) {
        next.phase = DRAG_MODE_DRAGGING;
    } else if (next.phase == DRAG_MODE_STOPPING && next.hold_blend >= 1.0f &&
        fabsf(feedback[0].velocity_rad_s) <= config->joint[0].start_speed_rad_s &&
        fabsf(feedback[1].velocity_rad_s) <= config->joint[1].start_speed_rad_s) {
        next.phase = DRAG_MODE_HOLDING;
    }
    result.phase = next.phase;
    /* 保持现有Mode/Task链路，摩擦项始终为零，无独立助推控制器。 */
    result.valid = true;
    *state = next;
    *status = result;
    memcpy(commands, output, sizeof(output));
    return true;
}
