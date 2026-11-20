#include "motor_task.h"

#include <float.h>
#include <math.h>
#include <string.h>
#include "bsp_key.h"
#include "joint_coordinates.h"

typedef enum { RECOVERY_BUS, RECOVERY_DISABLE, RECOVERY_FEEDBACK } RecoveryPhase;

static struct {
    bool initialized, key_pending, stop_pending;
    MotorTask_Config_t config;
    DM_Motor_t *motor[MOTOR_TASK_JOINT_COUNT];
    MotorTask_Status_t status;
    DM_Motor_Feedback_t reference_feedback[MOTOR_TASK_JOINT_COUNT];
    MotorMode_State_t mode;
    uint32_t arm_count[MOTOR_TASK_JOINT_COUNT];
    uint32_t enabled_at_ms, last_control_ms, last_query_ms;
    RecoveryPhase recovery_phase;
    MotorTask_State_t recovery_return;
    bool recovery_resume, recovery_hold, recovery_stable, recovery_success;
    bool recovery_exhausted;
    uint8_t recovery_attempts;
    uint32_t recovery_at_ms, recovery_stable_ms, recovery_last_try_ms;
    uint32_t recovery_count[MOTOR_TASK_JOINT_COUNT], recovery_success_ms;
    uint32_t recovery_stable_count[MOTOR_TASK_JOINT_COUNT];
} task;

static bool positive(float value)
{
    return isfinite(value) && value >= FLT_MIN;
}

static bool valid_limits(const MotorTask_JointConfig_t *config)
{
    if (config == NULL || !positive(config->device.pmax) ||
        !isfinite(config->lower_rad) || !isfinite(config->upper_rad)) { return false; }
    const float quantum = config->device.pmax / 65535.0f * 2.0f;
    return positive(quantum) && config->lower_rad >= -config->device.pmax &&
        config->upper_rad <= config->device.pmax &&
        config->upper_rad - config->lower_rad > 4.0f * quantum &&
        config->upper_rad - config->lower_rad < JOINT_TURN_RAD - 4.0f * quantum;
}

BSP_CAN_Status_t MotorTask_ClampTarget(const MotorTask_JointConfig_t *config,
                                      float requested, float *target)
{
    if (target == NULL || !isfinite(requested) || !valid_limits(config)) {
        return BSP_CAN_INVALID_ARG;
    }
    const float quantum = config->device.pmax / 65535.0f * 2.0f;
    *target = fminf(config->upper_rad - quantum,
                   fmaxf(config->lower_rad + quantum, requested));
    return BSP_CAN_OK;
}

static bool valid_config(const MotorTask_Config_t *config)
{
    if (!MotorMode_ValidConfig(MOTOR_MODE_DRAG, config) || config->control_period_ms == 0U ||
        config->control_period_ms > config->max_step_ms ||
        config->max_step_ms > 1000U || config->query_period_ms == 0U ||
        config->query_period_ms > config->feedback_max_age_ms ||
        config->feedback_max_age_ms > 10000U ||
        config->feedback_max_age_ms > config->enable_timeout_ms ||
        config->enable_timeout_ms > 10000U ||
        config->key_debounce_ms == 0U || config->key_debounce_ms > 1000U ||
        config->power_settle_ms > 10000U || config->status_period_ms == 0U ||
        config->status_period_ms > 10000U ||
        (config->stop_mode != MOTOR_STOP_UNCONFIRMED &&
         config->stop_mode != MOTOR_STOP_GRAVITY_HOLD)) { return false; }
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const MotorTask_JointConfig_t *joint = &config->joint[i];
        if (!valid_limits(joint) || !positive(joint->device.vmax) ||
            !positive(joint->device.tmax) || !positive(joint->tracking_error_rad) ||
            !positive(joint->start_speed_rad_s) ||
            joint->start_speed_rad_s > joint->device.vmax ||
            joint->temperature_c == 0U || joint->temperature_c > 120U ||
            (joint->protection_flags & ~MOTOR_PROTECT_ALL) != 0U) { return false; }
    }
    return true;
}

static void reset_mode(void)
{
    ++task.status.mode_generation;
    MotorMode_Reset(&task.mode);
    memset(&task.status.drag, 0, sizeof(task.status.drag));
    memset(&task.status.motion, 0, sizeof(task.status.motion));
    task.status.mode = task.mode.id;
    task.status.mode_phase = MOTOR_MODE_OFF;
}

static void update_policy(void)
{
    task.status.policy_ready = task.config.stop_mode == MOTOR_STOP_GRAVITY_HOLD &&
        task.config.fault_disable_approved && task.config.drag_reference_confirmed == 1U;
}

BSP_CAN_Status_t MotorTask_Init(const MotorTask_Config_t *config,
                               DM_Motor_t *const motors[MOTOR_TASK_JOINT_COUNT],
                               uint32_t now_ms)
{
    if (__get_IPSR() != 0U || !valid_config(config) || motors == NULL ||
        motors[0] == NULL || motors[1] == NULL || motors[0] == motors[1]) {
        return BSP_CAN_INVALID_ARG;
    }
    if (task.initialized) { return BSP_CAN_BUSY; }
    task.config = *config;
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.motor[i] = motors[i];
        task.status.disable_status[i] = BSP_CAN_NOT_READY;
        task.status.reference_rad[i] = NAN;
        task.status.reference_offset_rad[i] = NAN;
    }
    update_policy();
    task.last_control_ms = task.last_query_ms = now_ms;
    task.initialized = true;
    return BSP_CAN_OK;
}

static bool active(void)
{
    return task.status.state == MOTOR_TASK_ARMING ||
        task.status.state == MOTOR_TASK_RUNNING ||
        task.status.state == MOTOR_TASK_STOPPING ||
        task.status.state == MOTOR_TASK_HOLDING;
}

static void submit_disable(void)
{
    task.status.disable_submitted_mask = 0U;
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.status.disable_status[i] = DM_Motor_Disable(task.motor[i]);
        if (task.status.disable_status[i] == BSP_CAN_OK) {
            task.status.disable_submitted_mask |= (uint8_t)(1U << i);
        }
    }
}

static bool begin_recovery(uint32_t now_ms, bool resume, bool hold)
{
    /* Current application has one CAN owner and two motors on the same bus. */
    if (task.recovery_exhausted ||
        task.config.joint[0].device.bus != task.config.joint[1].device.bus ||
        task.recovery_attempts >= MOTOR_CAN_RECOVERY_ATTEMPTS ||
        BSP_CAN_RecoverBusOff(task.config.joint[0].device.bus) != BSP_CAN_BUSY) {
        return false;
    }
    ++task.recovery_attempts;
    /* Inactive recovery returns to IDLE or the existing FAULT latch. Active
     * recovery deliberately has FAULT as its fallback: only recovery_resume
     * may re-enter output, via start() at a fresh pose, never the old state. */
    task.recovery_return = task.status.state;
    task.recovery_resume = resume;
    task.recovery_hold = hold;
    task.recovery_success = task.recovery_stable = false;
    task.recovery_at_ms = task.recovery_last_try_ms = now_ms;
    task.recovery_phase = RECOVERY_BUS;
    task.status.state = MOTOR_TASK_RECOVERING;
    reset_mode();
    return true;
}

static void fault(MotorTask_Reason_t reason)
{
    if (task.status.state == MOTOR_TASK_FAULT) { return; }
    const bool resume = active() &&
        (reason == MOTOR_REASON_TRANSPORT || reason == MOTOR_REASON_STALE);
    const bool hold = task.status.state == MOTOR_TASK_STOPPING ||
        task.status.state == MOTOR_TASK_HOLDING || task.stop_pending;
    task.status.state = MOTOR_TASK_FAULT;
    task.status.reason = reason;
    ++task.status.mode_generation;
    MotorMode_Reset(&task.mode);
    task.status.mode_phase = MOTOR_MODE_OFF;
    task.status.drag.valid = false;
    task.status.motion.valid = false;
    if (task.config.fault_disable_approved) { submit_disable(); }
    if (resume) { (void)begin_recovery(HAL_GetTick(), true, hold); }
}

static uint32_t snapshot_feedback(void)
{
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        if (DM_Motor_GetFeedback(task.motor[i], &task.status.feedback[i]) == BSP_CAN_OK) {
            task.status.feedback_valid_mask |= (uint8_t)(1U << i);
            const MotorTask_JointConfig_t *cfg = &task.config.joint[i];
            const float raw = task.status.feedback[i].position_rad;
            const float reference = active() && isfinite(task.status.reference_rad[i]) ?
                raw + JointCoordinates_Offset(raw, task.status.reference_rad[i]) :
                JointCoordinates_Reference(raw, cfg->lower_rad, cfg->upper_rad);
            /* Use the SAME branch inverse for the actual MIT position target. */
            task.status.reference_offset_rad[i] = reference - raw;
            task.status.reference_rad[i] = reference;
            task.reference_feedback[i] = task.status.feedback[i];
            task.reference_feedback[i].position_rad = task.status.reference_rad[i];
        }
    }
    /* Take the age-check clock AFTER copying ISR samples. A receive interrupt
     * between the caller's timestamp and this snapshot may advance HAL tick.
     * Unsigned subtraction with that older clock would falsely report STALE. */
    return HAL_GetTick();
}

static MotorTask_Reason_t check_feedback(uint32_t now_ms, bool starting)
{
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const MotorTask_JointConfig_t *cfg = &task.config.joint[i];
        const DM_Motor_Feedback_t *fb = &task.reference_feedback[i];
        if ((task.status.feedback_valid_mask & (1U << i)) == 0U) {
            return MOTOR_REASON_NO_FEEDBACK;
        }
        if (now_ms - fb->received_at_ms > task.config.feedback_max_age_ms) {
            return MOTOR_REASON_STALE;
        }
        if (!isfinite(fb->position_rad) || fb->position_rad < cfg->lower_rad ||
            fb->position_rad > cfg->upper_rad) { return MOTOR_REASON_ANGLE; }
        if (fb->state > 1U ||
            (starting && fb->state !=
                (task.status.state == MOTOR_TASK_IDLE ||
                 task.status.state == MOTOR_TASK_RECOVERING ? 0U : 1U)) ||
            (!starting && task.status.state != MOTOR_TASK_ARMING && fb->state != 1U)) {
            return MOTOR_REASON_STATE;
        }
        if ((cfg->protection_flags & MOTOR_PROTECT_TEMP) != 0U &&
            (fb->mos_temperature_c >= cfg->temperature_c ||
             fb->rotor_temperature_c >= cfg->temperature_c)) {
            return MOTOR_REASON_TEMPERATURE;
        }
        /* Protocol/numeric validity is not a runtime overspeed limit. */
        if (!isfinite(fb->velocity_rad_s) || fabsf(fb->velocity_rad_s) > cfg->device.vmax) {
            return MOTOR_REASON_STATE;
        }
        if (starting && fabsf(fb->velocity_rad_s) > cfg->start_speed_rad_s) {
            return MOTOR_REASON_NOT_STILL;
        }
        /* 自由拖动没有固定位置目标，不能用旧目标的跟随误差阻止拖动。 */
        if (!starting && MotorMode_TracksPosition(&task.mode) &&
            (cfg->protection_flags & MOTOR_PROTECT_TRACKING) != 0U &&
            fabsf(task.status.target_reference_rad[i] - fb->position_rad) >
                cfg->tracking_error_rad) {
            return MOTOR_REASON_TRACKING;
        }
    }
    return MOTOR_REASON_NONE;
}

static bool mode_update(float dt_s, DM_Motor_MIT_Command_t commands[2])
{
    MotorMode_Diagnostics_t diagnostics = {0};
    const bool ok = MotorMode_Update(&task.config, task.reference_feedback, dt_s,
                                    &task.mode, &diagnostics, commands);
    task.status.drag = diagnostics.drag;
    task.status.motion = diagnostics.motion;
    return ok;
}

static bool send_targets(float dt_s)
{
    DM_Motor_MIT_Command_t commands[MOTOR_TASK_JOINT_COUNT];
    if (!mode_update(dt_s, commands)) {
        fault(MOTOR_REASON_POLICY);
        return false;
    }
    task.status.mode_phase = MotorMode_Phase(&task.mode);
    if (task.status.mode_phase == MOTOR_MODE_HOLDING) {
        task.status.state = MOTOR_TASK_HOLDING;
    }
    /* 同一控制步先计算/校验两轴，再顺序发 CAN；不宣称两帧同时到达。 */
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        /* Mode负责目标/轨迹可行性；提交层只做参考圈逆映射，不独立裁剪
         * 各轴的位置/速度来破坏轨迹。现有角度/失联/器件故障底线不变。 */
        task.status.target_reference_rad[i] = commands[i].position_rad;
        /* Invert the SAME branch used for feedback. Holding across a signed
         * encoder wrap must not send the old branch as a multi-turn PD target. */
        commands[i].position_rad -= task.status.reference_offset_rad[i];
        if (!isfinite(commands[i].position_rad) ||
            fabsf(commands[i].position_rad) > task.config.joint[i].device.pmax) {
            fault(MOTOR_REASON_ANGLE);
            return false;
        }
    }
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.status.target_rad[i] = commands[i].position_rad;
        task.status.target_velocity_rad_s[i] = commands[i].velocity_rad_s;
        task.status.target_torque_ff_nm[i] = commands[i].torque_ff_nm;
        task.status.target_kp[i] = commands[i].kp;
        task.status.target_kd[i] = commands[i].kd;
        task.status.last_transport = DM_Motor_SendMIT(task.motor[i], &commands[i]);
        if (task.status.last_transport != BSP_CAN_OK) {
            fault(MOTOR_REASON_TRANSPORT);
            return false;
        }
    }
    return true;
}

static void arm(uint32_t now_ms)
{
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.status.target_rad[i] = task.status.feedback[i].position_rad;
        task.arm_count[i] = task.status.feedback[i].received_count;
    }
    if (!send_targets(0.0f)) { return; }
    task.status.state = MOTOR_TASK_ARMING;
    task.enabled_at_ms = now_ms;
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.status.last_transport = DM_Motor_Enable(task.motor[i]);
        if (task.status.last_transport != BSP_CAN_OK) { fault(MOTOR_REASON_TRANSPORT); return; }
    }
}

static void start(uint32_t now_ms)
{
    if (!task.status.policy_ready) { task.status.reason = MOTOR_REASON_POLICY; return; }
    task.status.reason = check_feedback(now_ms, true);
    if (task.status.reason != MOTOR_REASON_NONE) { return; }
    task.stop_pending = false;
    if (task.status.state == MOTOR_TASK_HOLDING) {
        /* 保留支撑/前馈，Kp 渐退；不重复使能、不恢复历史往返。 */
        if (!MotorMode_Resume(&task.mode)) { fault(MOTOR_REASON_POLICY); return; }
        task.status.state = MOTOR_TASK_RUNNING;
        return;
    }
    reset_mode();
    arm(now_ms);
}

static void begin_stop(void)
{
    ++task.status.mode_generation;
    if (!MotorMode_Stop(&task.config, task.reference_feedback, &task.mode,
                        task.status.target_reference_rad)) {
        fault(MOTOR_REASON_POLICY);
        return;
    }
    task.status.mode_phase = MotorMode_Phase(&task.mode);
    task.status.state = MOTOR_TASK_STOPPING;
    task.stop_pending = false;
}

static void recovery_failed(uint32_t now_ms, RecoveryPhase next)
{
    task.recovery_stable = false;
    if (task.recovery_attempts >= MOTOR_CAN_RECOVERY_ATTEMPTS) {
        task.recovery_resume = false;
        task.recovery_exhausted = true;
        task.status.state = MOTOR_TASK_FAULT;
        if (task.status.reason == MOTOR_REASON_NONE) {
            task.status.reason = MOTOR_REASON_TRANSPORT;
        }
        return;
    }
    ++task.recovery_attempts;
    task.recovery_phase = next;
    task.recovery_at_ms = task.recovery_last_try_ms = now_ms;
}

static void cancel_auto_output(void)
{
    task.recovery_resume = false;
    task.recovery_return = MOTOR_TASK_FAULT;
    task.status.reason = MOTOR_REASON_MANUAL_DISABLE;
    task.key_pending = task.stop_pending = false;
}

static void recover(uint32_t now_ms)
{
    if (now_ms - task.recovery_at_ms >= MOTOR_CAN_RECOVERY_TIMEOUT_MS) {
        recovery_failed(now_ms, task.recovery_phase == RECOVERY_BUS ?
                         RECOVERY_BUS : RECOVERY_DISABLE);
        return;
    }
    if (task.recovery_phase == RECOVERY_BUS) {
        if (now_ms - task.recovery_at_ms < MOTOR_CAN_RECOVERY_RETRY_MS) { return; }
        const BSP_CAN_Status_t result =
            BSP_CAN_RecoverBusOff(task.config.joint[0].device.bus);
        if (result == BSP_CAN_BUSY) { return; }
        if (result != BSP_CAN_OK) {
            recovery_failed(now_ms, RECOVERY_BUS);
            return;
        }
        task.recovery_phase = RECOVERY_DISABLE;
        task.recovery_last_try_ms = now_ms - MOTOR_CAN_RECOVERY_RETRY_MS;
    }
    if (task.recovery_phase == RECOVERY_DISABLE) {
        if (now_ms - task.recovery_last_try_ms < MOTOR_CAN_RECOVERY_RETRY_MS) { return; }
        task.recovery_last_try_ms = now_ms;
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            task.recovery_count[i] = task.status.feedback[i].received_count;
        }
        submit_disable();
        if (task.status.disable_submitted_mask != 3U) {
            /* A second Bus-Off is eligible; plain BUSY/HAL errors do not
             * justify a controller reset or a new unbounded submission loop. */
            if (BSP_CAN_RecoverBusOff(task.config.joint[0].device.bus) == BSP_CAN_BUSY) {
                recovery_failed(now_ms, RECOVERY_BUS);
            }
            return;
        }
        task.recovery_phase = RECOVERY_FEEDBACK;
    }
    bool new_disabled = true;
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const DM_Motor_Feedback_t *fb = &task.status.feedback[i];
        if (fb->received_count == task.recovery_count[i] || fb->state != 0U) {
            new_disabled = false;
        }
    }
    /* RECOVERING uses the disabled startup gates, without an idle window. */
    const MotorTask_Reason_t reason = check_feedback(now_ms, true);
    if (reason == MOTOR_REASON_ANGLE || reason == MOTOR_REASON_TEMPERATURE ||
        reason == MOTOR_REASON_STATE || reason == MOTOR_REASON_POLICY) {
        /* Enabled feedback can precede the disable ACK; device fault states
         * (>1) and invalid numeric feedback must still permanently veto resume. */
        bool only_enabled = reason == MOTOR_REASON_STATE;
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            const DM_Motor_Feedback_t *fb = &task.status.feedback[i];
            if (fb->state > 1U || !isfinite(fb->velocity_rad_s) ||
                fabsf(fb->velocity_rad_s) > task.config.joint[i].device.vmax) {
                only_enabled = false;
            }
        }
        if (!only_enabled) {
            task.recovery_resume = false;
            task.recovery_return = MOTOR_TASK_FAULT;
            task.status.reason = reason;
        }
    }
    if (!new_disabled || reason != MOTOR_REASON_NONE) {
        task.recovery_stable = false;
        return;
    }
    if (!task.recovery_stable) {
        task.recovery_stable = true;
        task.recovery_stable_ms = now_ms;
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            task.recovery_stable_count[i] = task.status.feedback[i].received_count;
        }
        return;
    }
    if (now_ms - task.recovery_stable_ms < MOTOR_CAN_RECOVERY_STABLE_MS) { return; }
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        if (task.status.feedback[i].received_count == task.recovery_stable_count[i]) {
            return; /* A single cached disabled sample cannot prove stability. */
        }
    }
    const bool resume = task.recovery_resume && task.status.policy_ready;
    const bool hold = task.recovery_hold;
    task.recovery_resume = false;
    task.recovery_success = true;
    task.recovery_success_ms = now_ms;
    task.status.state = resume ? MOTOR_TASK_IDLE : task.recovery_return;
    if (task.status.state == MOTOR_TASK_IDLE) { task.status.reason = MOTOR_REASON_NONE; }
    if (resume) {
        task.status.reason = MOTOR_REASON_NONE;
        start(now_ms); /* Fresh pose, OFF Mode -> zero Kp/FF -> normal enable ACK. */
        if (task.status.state == MOTOR_TASK_ARMING) { task.stop_pending = hold; }
    }
}

static void preview(float dt_s, uint32_t now_ms)
{
    task.status.drag.valid = false;
    task.status.motion.valid = false;
    if (task.status.feedback_valid_mask != 3U) { return; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (now_ms - task.status.feedback[i].received_at_ms >
            task.config.feedback_max_age_ms) { return; }
    }
    DM_Motor_MIT_Command_t unused[2];
    (void)mode_update(dt_s, unused);
    task.status.mode_phase = MotorMode_Phase(&task.mode);
}

BSP_CAN_Status_t MotorTask_Run(uint32_t now_ms)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    if (BSP_KEY_Scan(now_ms) == BSP_KEY_EVENT_PRESSED) { task.key_pending = true; }
    if (now_ms - task.last_control_ms < task.config.control_period_ms) {
        return task.status.state == MOTOR_TASK_FAULT ||
            task.status.state == MOTOR_TASK_RECOVERING ? BSP_CAN_ERROR : BSP_CAN_OK;
    }
    now_ms = snapshot_feedback();
    const uint32_t gap = now_ms - task.last_control_ms;
    task.last_control_ms = now_ms;
    if (active() && gap > task.config.max_step_ms) { fault(MOTOR_REASON_TIMING); }
    if (active()) {
        const MotorTask_Reason_t reason = check_feedback(now_ms, false);
        if (reason != MOTOR_REASON_NONE) { fault(reason); }
    }
    if (task.key_pending) {
        task.key_pending = false;
        if (task.status.state == MOTOR_TASK_RECOVERING) { cancel_auto_output(); }
        else if (task.status.state == MOTOR_TASK_IDLE || task.status.state == MOTOR_TASK_HOLDING) {
            start(now_ms);
        } else if (task.status.state == MOTOR_TASK_RUNNING) { begin_stop(); }
        else if (task.status.state == MOTOR_TASK_ARMING) { task.stop_pending = true; }
    }
    if (task.status.state == MOTOR_TASK_RECOVERING) { recover(now_ms); }
    if (task.status.state == MOTOR_TASK_ARMING) {
        bool enabled = true;
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            const DM_Motor_Feedback_t *fb = &task.status.feedback[i];
            if (fb->state != 1U || fb->received_count == task.arm_count[i]) { enabled = false; }
        }
        if (enabled) {
            if (MotorMode_Phase(&task.mode) == MOTOR_MODE_OFF &&
                !MotorMode_Enter(&task.config, task.reference_feedback, &task.mode)) {
                fault(MOTOR_REASON_POLICY);
            } else if (task.stop_pending) { begin_stop(); }
            else { task.status.state = MOTOR_TASK_RUNNING; }
        } else if (now_ms - task.enabled_at_ms >= task.config.enable_timeout_ms) {
            fault(MOTOR_REASON_ENABLE_TIMEOUT);
        }
    }
    if (active()) {
        /* A prepared capture keeps its exact center, but gains/support do not
         * ramp until BOTH motors acknowledge the enable handshake. */
        (void)send_targets(task.status.state == MOTOR_TASK_ARMING ?
                           0.0f : (float)gap * 0.001f);
    }
    else if (task.status.state == MOTOR_TASK_IDLE) {
        preview((float)(gap > task.config.max_step_ms ? task.config.control_period_ms : gap)
                * 0.001f, now_ms);
    }
    /* Each MIT command elicits motor feedback. ARMING/RUNNING/STOPPING/HOLDING
     * already submit both axes every control step; don't add 0x7FF queries.
     * Inactive observation still polls at the configured rate, after KEY and
     * fault handling. No fallback query hides missing MIT feedback: the same
     * feedback-age/enable checks remain responsible for detecting that loss. */
    if (!active() && now_ms - task.last_query_ms >= task.config.query_period_ms) {
        task.last_query_ms = now_ms;
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            (void)DM_Motor_RequestFeedback(task.motor[i]);
        }
    }
    if ((task.status.state == MOTOR_TASK_IDLE || task.status.state == MOTOR_TASK_FAULT) &&
        now_ms - task.recovery_last_try_ms >= MOTOR_CAN_RECOVERY_RETRY_MS) {
        task.recovery_last_try_ms = now_ms;
        (void)begin_recovery(now_ms, false, false);
    }
    if (task.recovery_success && task.status.state != MOTOR_TASK_RECOVERING &&
        task.status.state != MOTOR_TASK_FAULT &&
        now_ms - task.recovery_success_ms >= MOTOR_CAN_RECOVERY_HEALTHY_MS) {
        task.recovery_attempts = 0U;
        task.recovery_success = false;
    }
    return task.status.state == MOTOR_TASK_FAULT ||
        task.status.state == MOTOR_TASK_RECOVERING ? BSP_CAN_ERROR : BSP_CAN_OK;
}

BSP_CAN_Status_t MotorTask_GetStatus(MotorTask_Status_t *status)
{
    if (status == NULL || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    *status = task.status;
    return BSP_CAN_OK;
}

BSP_CAN_Status_t MotorTask_GetConfig(MotorTask_Config_t *config)
{
    if (config == NULL || __get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    *config = task.config;
    return BSP_CAN_OK;
}

static bool inactive_disabled(uint32_t now_ms)
{
    if (task.status.state != MOTOR_TASK_IDLE && task.status.state != MOTOR_TASK_FAULT) {
        return false;
    }
    now_ms = snapshot_feedback();
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const DM_Motor_Feedback_t *fb = &task.status.feedback[i];
        if ((task.status.feedback_valid_mask & (1U << i)) == 0U ||
            now_ms - fb->received_at_ms > task.config.feedback_max_age_ms ||
            fb->state != 0U || !isfinite(fb->position_rad) ||
            !isfinite(fb->velocity_rad_s) ||
            fabsf(fb->velocity_rad_s) > task.config.joint[i].start_speed_rad_s) { return false; }
    }
    return true;
}

BSP_CAN_Status_t MotorTask_ApplyConfig(const MotorTask_Config_t *config, uint32_t now_ms)
{
    if (__get_IPSR() != 0U || !valid_config(config)) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        const DM_Motor_Config_t *a = &config->joint[i].device, *b = &task.config.joint[i].device;
        if (a->model != b->model || a->bus != b->bus || a->can_id != b->can_id ||
            a->master_id != b->master_id || a->pmax != b->pmax ||
            a->vmax != b->vmax || a->tmax != b->tmax ||
            a->protocol != b->protocol) { return BSP_CAN_INVALID_ARG; }
    }
    if (config->control_period_ms != task.config.control_period_ms ||
        config->query_period_ms != task.config.query_period_ms ||
        config->key_debounce_ms != task.config.key_debounce_ms ||
        config->power_settle_ms != task.config.power_settle_ms ||
        config->status_period_ms != task.config.status_period_ms ||
        config->stop_mode != task.config.stop_mode ||
        config->fault_disable_approved != task.config.fault_disable_approved) {
        return BSP_CAN_INVALID_ARG;
    }
    if (!inactive_disabled(now_ms)) { return BSP_CAN_BUSY; }
    task.config = *config;
    /* Inactive-only config commit may change the reference interval. Re-anchor
     * explicitly; never re-anchor an active hold target. */
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        task.status.reference_rad[i] = NAN;
    }
    (void)snapshot_feedback();
    update_policy();
    reset_mode();
    return BSP_CAN_OK;
}

BSP_CAN_Status_t MotorTask_Command(MotorTask_Command_t command, uint32_t now_ms)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    if (command == MOTOR_COMMAND_START) {
        if (task.status.state != MOTOR_TASK_IDLE && task.status.state != MOTOR_TASK_HOLDING) {
            return BSP_CAN_BUSY;
        }
        now_ms = snapshot_feedback();
        start(now_ms);
        return task.status.state == MOTOR_TASK_ARMING ||
               task.status.state == MOTOR_TASK_RUNNING ? BSP_CAN_OK : BSP_CAN_ERROR;
    }
    if (command == MOTOR_COMMAND_STOP) {
        if (task.status.state == MOTOR_TASK_RECOVERING) { cancel_auto_output(); }
        else if (task.status.state == MOTOR_TASK_RUNNING) {
            now_ms = snapshot_feedback();
            const MotorTask_Reason_t reason = check_feedback(now_ms, false);
            if (reason != MOTOR_REASON_NONE) { fault(reason); } else { begin_stop(); }
        } else if (task.status.state == MOTOR_TASK_ARMING) { task.stop_pending = true; }
        return task.status.state == MOTOR_TASK_FAULT ? BSP_CAN_ERROR : BSP_CAN_OK;
    }
    if (command == MOTOR_COMMAND_DISABLE) {
        if (task.status.state == MOTOR_TASK_RECOVERING) {
            cancel_auto_output();
            submit_disable();
        } else if (task.status.state != MOTOR_TASK_FAULT) { fault(MOTOR_REASON_MANUAL_DISABLE); }
        else { submit_disable(); } /* 只在人工再次请求时重试，不自动重发。 */
        task.key_pending = false;
        return task.status.disable_submitted_mask == 3U ? BSP_CAN_OK : BSP_CAN_ERROR;
    }
    if (command == MOTOR_COMMAND_CLEAR) {
        if (!inactive_disabled(now_ms)) { return BSP_CAN_BUSY; }
        task.status.state = MOTOR_TASK_IDLE;
        task.status.reason = MOTOR_REASON_NONE;
        task.status.disable_submitted_mask = 0U;
        task.key_pending = task.stop_pending = false;
        task.recovery_resume = task.recovery_exhausted = task.recovery_success = false;
        task.recovery_attempts = 0U;
        reset_mode();
        for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
            task.status.target_rad[i] = task.status.feedback[i].position_rad;
            task.status.target_reference_rad[i] = task.status.reference_rad[i];
            task.status.target_velocity_rad_s[i] = task.status.target_torque_ff_nm[i] = 0.0f;
            task.status.target_kp[i] = task.status.target_kd[i] = 0.0f;
        }
        return BSP_CAN_OK;
    }
    return BSP_CAN_INVALID_ARG;
}

BSP_CAN_Status_t MotorTask_SelectMode(MotorMode_Id_t mode)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    if (mode != MOTOR_MODE_DRAG && mode != MOTOR_MODE_MOTION) {
        return BSP_CAN_INVALID_ARG;
    }
    if (!MotorMode_Available(mode)) { return BSP_CAN_NOT_READY; }
    if (task.status.state != MOTOR_TASK_IDLE && task.status.state != MOTOR_TASK_FAULT) {
        return BSP_CAN_BUSY;
    }
    task.mode.id = mode;
    reset_mode();
    return BSP_CAN_OK;
}

BSP_CAN_Status_t MotorTask_CaptureHold(uint32_t now_ms)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    if (task.status.state == MOTOR_TASK_FAULT &&
        task.status.reason == MOTOR_REASON_MANUAL_DISABLE) {
        const BSP_CAN_Status_t result = MotorTask_Command(MOTOR_COMMAND_CLEAR, now_ms);
        if (result != BSP_CAN_OK) { return result; }
    }
    const bool idle = task.status.state == MOTOR_TASK_IDLE;
    if (!idle && task.status.state != MOTOR_TASK_RUNNING &&
        task.status.state != MOTOR_TASK_STOPPING &&
        task.status.state != MOTOR_TASK_HOLDING) { return BSP_CAN_BUSY; }
    if (!task.status.policy_ready) {
        task.status.reason = MOTOR_REASON_POLICY;
        return BSP_CAN_ERROR;
    }
    now_ms = snapshot_feedback();
    const MotorTask_Reason_t reason = check_feedback(now_ms, idle);
    if (reason != MOTOR_REASON_NONE) {
        if (idle) { task.status.reason = reason; }
        else { fault(reason); }
        return BSP_CAN_ERROR;
    }
    DM_Motor_MIT_Command_t previous[2] = {0};
    for (unsigned i = 0U; i < MOTOR_TASK_JOINT_COUNT; ++i) {
        previous[i].kd = task.status.target_kd[i];
        previous[i].torque_ff_nm = task.status.target_torque_ff_nm[i];
    }
    if (!MotorMode_CaptureHold(&task.config, task.reference_feedback,
                              idle ? NULL : previous, &task.mode)) {
        return BSP_CAN_INVALID_ARG;
    }
    ++task.status.mode_generation;
    task.status.mode = task.mode.id;
    task.status.mode_phase = MotorMode_Phase(&task.mode);
    memset(&task.status.drag, 0, sizeof(task.status.drag));
    memset(&task.status.motion, 0, sizeof(task.status.motion));
    task.status.reason = MOTOR_REASON_NONE;
    task.key_pending = task.stop_pending = false;
    if (idle) { arm(now_ms); }
    else { task.status.state = MOTOR_TASK_RUNNING; }
    return task.status.state == MOTOR_TASK_ARMING ||
           task.status.state == MOTOR_TASK_RUNNING ? BSP_CAN_OK : BSP_CAN_ERROR;
}

BSP_CAN_Status_t MotorTask_Move(const MotionMode_Request_t *request, uint32_t now_ms)
{
    if (__get_IPSR() != 0U || request == NULL) { return BSP_CAN_INVALID_ARG; }
    if (!task.initialized) { return BSP_CAN_NOT_READY; }
    if (task.mode.id != MOTOR_MODE_MOTION || task.status.state != MOTOR_TASK_HOLDING) {
        return BSP_CAN_BUSY;
    }
    now_ms = snapshot_feedback();
    const MotorTask_Reason_t reason = check_feedback(now_ms, false);
    if (reason != MOTOR_REASON_NONE) { fault(reason); return BSP_CAN_ERROR; }
    if (!MotorMode_Move(&task.config, request, &task.mode)) { return BSP_CAN_INVALID_ARG; }
    ++task.status.mode_generation;
    task.status.state = MOTOR_TASK_RUNNING;
    task.status.mode_phase = MotorMode_Phase(&task.mode);
    return BSP_CAN_OK;
}
