#include "motion_mode.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

_Static_assert(JOINT_TRAJECTORY_AXES == MOTOR_TASK_JOINT_COUNT,
               "Adapt model, mode, coordinates and transport together");

bool MotionMode_ValidConfig(const MotorTask_Config_t *config)
{
    if (config == NULL || config->control_period_ms == 0U ||
        !isfinite(config->drag_j2_zero_rad) || g_motion_config.enter_ms == 0U ||
        !isfinite(g_motion_config.friction_smooth_rad_s) ||
        g_motion_config.friction_smooth_rad_s <= 0.0f ||
        !isfinite(g_motion_config.friction_velocity_filter_ms) ||
        g_motion_config.friction_velocity_filter_ms <= 0.0f ||
        !isfinite(g_motion_config.velocity_limit_rad_s) ||
        g_motion_config.velocity_limit_rad_s <= 0.0f ||
        !isfinite(g_motion_config.acceleration_limit_rad_s2) ||
        g_motion_config.acceleration_limit_rad_s2 <= 0.0f) {
        return false;
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        const MotorTask_JointConfig_t *joint = &config->joint[i];
        float friction;
        if (!RobotModel_Friction(&g_robot_friction[i], 0.0f,
                g_motion_config.friction_smooth_rad_s, &friction)) { return false; }
        if (!isfinite(g_motion_config.kp[i]) || g_motion_config.kp[i] <= 0.0f ||
            g_motion_config.kp[i] > 500.0f ||
            !isfinite(g_motion_config.kd[i]) || g_motion_config.kd[i] < 0.0f ||
            g_motion_config.kd[i] > 5.0f ||
            !isfinite(joint->lower_rad) || !isfinite(joint->upper_rad) ||
            joint->lower_rad >= joint->upper_rad) { return false; }
    }
    return true;
}

void MotionMode_Reset(MotionMode_State_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
        JointTrajectory_Reset();
    }
}

static bool within(const MotorTask_JointConfig_t *joint, double position)
{
    /* One encoder quantum keeps rounding inside the existing mechanical range.
     * Validate before motion, never independently clip a moving reference. */
    const double margin = 2.0 * joint->device.pmax / 65535.0;
    return isfinite(position) && position >= joint->lower_rad + margin &&
        position <= joint->upper_rad - margin;
}

bool MotionMode_Enter(const MotorTask_Config_t *config,
                      const DM_Motor_Feedback_t feedback[2], MotionMode_State_t *state)
{
    if (!MotionMode_ValidConfig(config) || feedback == NULL || state == NULL ||
        state->phase != MOTION_MODE_OFF) { return false; }
    MotionMode_State_t next = {0};
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!within(&config->joint[i], feedback[i].position_rad) ||
            !isfinite(feedback[i].velocity_rad_s)) { return false; }
        next.reference.position[i] = feedback[i].position_rad;
        next.encoder_position_rad[i] = feedback[i].position_rad;
        next.encoder_sample_ms[i] = feedback[i].received_at_ms;
        next.encoder_sample_count[i] = feedback[i].received_count;
    }
    if (JointTrajectory_Init(config->control_period_ms * .001) < 0) { return false; }
    next.phase = MOTION_MODE_ENTERING;
    *state = next;
    return true;
}

/* Task supplies continuous reference-branch encoder positions. Differentiate
 * NEW samples using their receive timestamps, not repeated control snapshots.
 * A long/backwards sample interval only reseeds this estimate; no new fault. */
static float friction_velocity(const MotorTask_Config_t *config,
    const DM_Motor_Feedback_t *feedback, unsigned i, MotionMode_State_t *next)
{
    const float tau_s = g_motion_config.friction_velocity_filter_ms * .001f;
    const uint32_t elapsed_ms =
        feedback->received_at_ms - next->encoder_sample_ms[i];
    if (feedback->received_count != next->encoder_sample_count[i] && elapsed_ms > 0U) {
        const float delta = feedback->position_rad - next->encoder_position_rad[i];
        if (elapsed_ms <= config->feedback_max_age_ms) {
            const float sample_s = elapsed_ms * .001f;
            const float differentiated = delta / sample_s;
            const float alpha = sample_s / (tau_s + sample_s);
            next->filtered_velocity_rad_s[i] += alpha *
                (differentiated - next->filtered_velocity_rad_s[i]);
        } else {
            next->filtered_velocity_rad_s[i] = 0.0f;
        }
        next->encoder_position_rad[i] = feedback->position_rad;
        next->encoder_sample_ms[i] = feedback->received_at_ms;
        next->encoder_sample_count[i] = feedback->received_count;
    }
    /* A one-count position disturbance produces at most quantum/tau after
     * this low-pass. Remove that resolution-sized noise CONTINUOUSLY; this is
     * not a speed trip or a minimum allowed motion. Do not use raw motor v. */
    const float quantum = 2.0f * config->joint[i].device.pmax / 65535.0f;
    const float filtered = next->filtered_velocity_rad_s[i];
    return copysignf(fmaxf(0.0f, fabsf(filtered) - quantum / tau_s), filtered);
}

static bool plan_within_limits(const MotorTask_Config_t *config)
{
    double low[2], high[2];
    if (JointTrajectory_PositionBounds(low, high) < 0) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!within(&config->joint[i], low[i]) ||
            !within(&config->joint[i], high[i])) { return false; }
    }
    return true;
}

static JointTrajectory_Input_t make_input(const MotionMode_State_t *state,
                                         const MotionMode_Request_t *request)
{
    JointTrajectory_Input_t input = {0};
    for (unsigned i = 0U; i < 2U; ++i) {
        input.axis[i] = (JointTrajectory_Axis_t) {
            .position = state->reference.position[i],
            .velocity = state->reference.velocity[i],
            .acceleration = state->reference.acceleration[i],
            .target_position = request->target_rad[i],
            .max_velocity = request->velocity_rad_s[i],
            .max_acceleration = request->acceleration_rad_s2[i],
            .max_jerk = request->jerk_rad_s3[i]
        };
    }
    return input;
}

/* Conservative nominal FF encoding check, not a claimed physical torque limit.
 * Prevent an obviously unencodable job before execution; no per-axis clipping. */
static bool encodable_ff(const MotorTask_Config_t *config,
                         const MotionMode_Request_t *request)
{
    const double a = fabs(g_robot_model.a0) + g_robot_model.extra_inertia_kg_m2[0] +
        hypot(g_robot_model.a_cos2, g_robot_model.a_sin2);
    const double ap = 2.0 * hypot(g_robot_model.a_cos2, g_robot_model.a_sin2);
    const double b = hypot(g_robot_model.b_cos, g_robot_model.b_sin);
    const double v1 = request->velocity_rad_s[0], v2 = request->velocity_rad_s[1];
    const double x1 = request->acceleration_rad_s2[0], x2 = request->acceleration_rad_s2[1];
    const double friction1 = g_robot_friction[0].coulomb_nm +
        g_robot_friction[0].viscous_nm_s*v1;
    const double friction2 = g_robot_friction[1].coulomb_nm +
        g_robot_friction[1].viscous_nm_s*v2;
    return a*x1 + b*x2 + ap*v1*v2 + b*v2*v2 + friction1 <= config->joint[0].device.tmax &&
        b*x1 + (fabs(g_robot_model.d) + g_robot_model.extra_inertia_kg_m2[1])*x2 + .5*ap*v1*v1 +
            fabs(g_robot_model.gravity_sin_nm) + friction2 <= config->joint[1].device.tmax;
}

bool MotionMode_Move(const MotorTask_Config_t *config,
                     const MotionMode_Request_t *request, MotionMode_State_t *state)
{
    if (!MotionMode_ValidConfig(config) || request == NULL || state == NULL ||
        state->phase != MOTION_MODE_HOLDING) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!within(&config->joint[i], request->target_rad[i]) ||
            !isfinite(request->velocity_rad_s[i]) || request->velocity_rad_s[i] <= 0.0f ||
            request->velocity_rad_s[i] > config->joint[i].device.vmax ||
            request->velocity_rad_s[i] > g_motion_config.velocity_limit_rad_s ||
            !isfinite(request->acceleration_rad_s2[i]) || request->acceleration_rad_s2[i] <= 0.0f ||
            request->acceleration_rad_s2[i] > g_motion_config.acceleration_limit_rad_s2 ||
            !isfinite(request->jerk_rad_s3[i]) || request->jerk_rad_s3[i] <= 0.0f) {
            return false;
        }
    }
    if (!encodable_ff(config, request)) { return false; }
    const JointTrajectory_Input_t input = make_input(state, request);
    double duration;
    if (JointTrajectory_Calculate(&input, &duration) < 0 ||
        !plan_within_limits(config)) {
        JointTrajectory_Reset();
        return false; /* HOLDING does not depend on the invalidated planner. */
    }
    state->request = *request;
    state->duration_s = duration;
    state->elapsed_s = 0.0;
    state->phase = MOTION_MODE_MOVING;
    return true;
}

bool MotionMode_Stop(const MotorTask_Config_t *config, MotionMode_State_t *state)
{
    if (!MotionMode_ValidConfig(config) || state == NULL ||
        state->phase == MOTION_MODE_OFF) { return false; }
    if (state->phase != MOTION_MODE_MOVING) { return true; }
    const JointTrajectory_Input_t input = make_input(state, &state->request);
    double duration;
    if (JointTrajectory_Brake(&input, &duration) < 0 ||
        !plan_within_limits(config)) { return false; }
    state->elapsed_s = 0.0;
    state->duration_s = duration;
    state->phase = MOTION_MODE_STOPPING;
    return true;
}

bool MotionMode_Update(const MotorTask_Config_t *config,
                       const DM_Motor_Feedback_t feedback[2], float dt_s,
                       MotionMode_State_t *state, MotionMode_Status_t *status,
                       DM_Motor_MIT_Command_t commands[2])
{
    if (!MotionMode_ValidConfig(config) || feedback == NULL || state == NULL ||
        status == NULL || commands == NULL || !isfinite(dt_s) || dt_s < 0.0f ||
        dt_s > config->max_step_ms * .001f) { return false; }
    MotionMode_State_t next = *state;
    MotionMode_Status_t report = {0};
    DM_Motor_MIT_Command_t output[2] = {0};
    float velocity[2], acceleration[2] = {0};
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!isfinite(feedback[i].position_rad) ||
            !isfinite(feedback[i].velocity_rad_s)) { return false; }
        velocity[i] = feedback[i].velocity_rad_s;
        output[i].position_rad = feedback[i].position_rad;
    }
    if (next.phase == MOTION_MODE_OFF) {
        *status = report;
        memcpy(commands, output, sizeof(output));
        return true; /* Pre-enable preview has zero stiffness and FF. */
    }
    if (next.phase == MOTION_MODE_ENTERING) {
        next.entry_blend = fminf(1.0f, next.entry_blend +
            dt_s * 1000.0f / g_motion_config.enter_ms);
        if (next.entry_blend >= 1.0f) { next.phase = MOTION_MODE_HOLDING; }
    } else if (next.phase == MOTION_MODE_MOVING || next.phase == MOTION_MODE_STOPPING) {
        next.elapsed_s = fmin(next.duration_s, next.elapsed_s + dt_s);
        const JointTrajectory_Result_t result =
            JointTrajectory_Sample(next.elapsed_s, &next.reference);
        if (result < 0) { return false; }
        if (result == JOINT_TRAJECTORY_FINISHED) {
            next.phase = MOTION_MODE_HOLDING;
            /* Both job types end at rest. Remove numerical endpoint residuals. */
            memset(next.reference.velocity, 0, sizeof(next.reference.velocity));
            memset(next.reference.acceleration, 0, sizeof(next.reference.acceleration));
        }
    }
    for (unsigned i = 0U; i < 2U; ++i) {
        acceleration[i] = (float)next.reference.acceleration[i];
    }
    /* Full nominal inverse dynamics, not Drag's measured-acceleration feedback.
     * Actual q/dq evaluate the model; desired ddq is from the shared trajectory.
     * Positive coordinate direction is the existing encoder/CAD convention. */
    if (!RobotModel_Dynamics(&g_robot_model,
            feedback[1].position_rad - config->drag_j2_zero_rad,
            velocity, acceleration, &report.model)) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        /* 编码器位置差分 -> 一阶低通 -> 分辨率消噪 -> 既有摩擦模型。
         * 固定目标下的手推/回正也参与；不再额外叠加规划速度摩擦FF。
         * M*ddq_des、c(q,dq)、g(q)、MIT目标p/v和Kp/Kd仍走原逻辑。 */
        report.friction_velocity_rad_s[i] = friction_velocity(config, &feedback[i], i, &next);
        if (!RobotModel_Friction(&g_robot_friction[i],
                report.friction_velocity_rad_s[i], g_motion_config.friction_smooth_rad_s,
                &report.friction_nm[i])) { return false; }
        const float ff = report.model.inertial_nm[i] + report.model.coupling_nm[i] +
            report.model.gravity_nm[i] + report.friction_nm[i];
        output[i] = (DM_Motor_MIT_Command_t) {
            .position_rad = (float)next.reference.position[i],
            .velocity_rad_s = (float)next.reference.velocity[i],
            .kp = g_motion_config.kp[i] * next.entry_blend,
            .kd = next.entry_kd[i] * (1.0f - next.entry_blend) +
                g_motion_config.kd[i] * next.entry_blend,
            .torque_ff_nm = next.entry_ff_nm[i] * (1.0f - next.entry_blend) +
                ff * next.entry_blend
        };
        if (!within(&config->joint[i], output[i].position_rad) ||
            !isfinite(ff) || fabsf(output[i].torque_ff_nm) > config->joint[i].device.tmax ||
            fabsf(output[i].velocity_rad_s) > config->joint[i].device.vmax) { return false; }
        report.target_acceleration_rad_s2[i] = acceleration[i];
        report.position_error_rad[i] = output[i].position_rad - feedback[i].position_rad;
        report.velocity_error_rad_s[i] = output[i].velocity_rad_s - velocity[i];
    }
    report.valid = true;
    report.phase = next.phase;
    report.elapsed_s = (float)next.elapsed_s;
    report.duration_s = (float)next.duration_s;
    *state = next;
    *status = report;
    memcpy(commands, output, sizeof(output));
    return true;
}
