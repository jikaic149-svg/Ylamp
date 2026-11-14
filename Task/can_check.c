#include "can_check.h"

#include <math.h>

volatile uint32_t g_can_check_request;
volatile CanCheck_Status_t g_can_check_status;
volatile uint32_t g_can_probe_submitted, g_can_probe_failed;

static DM_Motor_t *handles[MOTOR_CAN_JOINT_COUNT];
static CanCheck_Status_t status;
static uint32_t last_run_ms, last_query_ms, phase_start_ms, command_count;
static bool query_only, j3_only, legacy_disable, legacy_stream;
static uint32_t stream_start_count;
static unsigned scan_bus, scan_address, scan_kind;

static void publish(uint32_t now_ms)
{
    const uint32_t even = status.sequence + 2U;
    g_can_check_status.sequence = even - 1U;
    status.tick_ms = now_ms;
    status.sequence = even - 1U;
    status.sequence_end = even;
    g_can_check_status = status;
    g_can_check_status.sequence = even;
    status.sequence = even;
}

static bool accepted(CanCheck_Result_t *r, BSP_CAN_Status_t result)
{
    r->last_transport = (uint32_t)result;
    return result == BSP_CAN_OK;
}

static void begin_motor(uint32_t now_ms)
{
    DM_Motor_Feedback_t f;
    CanCheck_Result_t *r = &status.motor[status.motor_index];
    if (DM_Motor_GetFeedback(handles[status.motor_index], &f) == BSP_CAN_OK) {
        r->feedback_start_count = f.received_count;
    }
    status.phase = CAN_CHECK_QUERY;
    phase_start_ms = now_ms;
    last_query_ms = now_ms - g_motor_config.query_period_ms;
}

static void next_motor(uint32_t now_ms)
{
    ++status.completed;
    if (j3_only || status.completed == MOTOR_CAN_JOINT_COUNT) {
        status.phase = CAN_CHECK_DONE;
    } else {
        ++status.motor_index;
        begin_motor(now_ms);
    }
}

static void disable_motor(CanCheck_Result_t *r, uint32_t now_ms)
{
    command_count = r->feedback_valid != 0U
        ? r->feedback.received_count : r->feedback_start_count;
    if (accepted(r, DM_Motor_Disable(handles[status.motor_index]))) {
        ++r->disable_submitted;
    } else {
        r->reason = CAN_CHECK_TRANSPORT;
    }
    status.phase = CAN_CHECK_DISABLING;
    phase_start_ms = now_ms;
}

static void enable_motor(CanCheck_Result_t *r, uint32_t now_ms)
{
    /* Do not reuse an earlier motion target on enable. */
    const DM_Motor_MIT_Command_t neutral = {.position_rad = r->feedback.position_rad};
    const BSP_CAN_Status_t preload = isfinite(neutral.position_rad)
        ? DM_Motor_SendMIT(handles[status.motor_index], &neutral) : BSP_CAN_INVALID_ARG;
    if (legacy_stream) {
        if (preload == BSP_CAN_OK) { ++r->mit_submitted; }
        else { ++r->mit_failed; }
    }
    if (!isfinite(neutral.position_rad) || !accepted(r, preload)) {
        r->reason = CAN_CHECK_TRANSPORT;
        next_motor(now_ms); /* Fresh disabled state; no enable submitted. */
        return;
    }
    command_count = r->feedback.received_count;
    if (accepted(r, DM_Motor_Enable(handles[status.motor_index]))) {
        ++r->enable_submitted;
        status.phase = CAN_CHECK_ENABLING;
        phase_start_ms = now_ms;
        if (legacy_stream) { last_query_ms = now_ms; }
    } else {
        r->reason = CAN_CHECK_TRANSPORT;
        disable_motor(r, now_ms);
    }
}

void CanCheck_Init(DM_Motor_t *const motors[MOTOR_CAN_JOINT_COUNT], uint32_t now_ms)
{
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        handles[i] = motors[i];
    }
    status = (CanCheck_Status_t){0};
    query_only = false;
    j3_only = false;
    legacy_disable = false;
    legacy_stream = false;
    g_can_check_request = 0U;
    g_can_probe_submitted = g_can_probe_failed = 0U;
    last_run_ms = now_ms;
    publish(now_ms);
}

void CanCheck_Run(uint32_t now_ms)
{
    if (now_ms - last_run_ms < g_motor_config.control_period_ms) { return; }
    last_run_ms = now_ms;
    const uint32_t request = g_can_check_request;
    g_can_check_request = 0U;
    if (status.phase == CAN_CHECK_IDLE && request == 1U) { begin_motor(now_ms); }
    if ((status.phase == CAN_CHECK_IDLE || status.phase == CAN_CHECK_DONE) &&
        (request >= 2U && request <= 6U)) {
        const uint32_t sequence = status.sequence;
        status = (CanCheck_Status_t){.sequence = sequence};
        query_only = true;
        legacy_disable = request == 5U;
        legacy_stream = request == 6U;
        j3_only = request == 4U || legacy_disable || legacy_stream;
        if (j3_only) { status.motor_index = 2U; }
        if (request != 3U) { begin_motor(now_ms); }
        else {
            scan_bus = scan_kind = 0U;
            scan_address = 1U;
            g_can_probe_submitted = g_can_probe_failed = 0U;
            status.phase = CAN_CHECK_SCANNING;
            last_query_ms = now_ms - g_motor_config.query_period_ms;
        }
        if (legacy_disable || legacy_stream) {
            CanCheck_Result_t *r = &status.motor[status.motor_index];
            disable_motor(r, now_ms);
            if (r->reason == CAN_CHECK_TRANSPORT) { status.phase = CAN_CHECK_STOPPED; }
            else if (legacy_stream) { status.phase = CAN_CHECK_PREPARING; }
            publish(now_ms);
            return;
        }
    }
    if (status.phase == CAN_CHECK_SCANNING) {
        if (scan_bus < 2U &&
            now_ms - last_query_ms >= g_motor_config.query_period_ms) {
            last_query_ms = now_ms;
            FDCAN_HandleTypeDef *bus = scan_bus == 0U
                ? g_motor_can_config[2].bus : g_motor_can_config[0].bus;
            if (DM_Motor_ProbeReadOnly(bus, (uint16_t)scan_address,
                                      scan_kind != 0U) == BSP_CAN_OK) {
                ++g_can_probe_submitted;
            } else { ++g_can_probe_failed; }
            if (++scan_kind == 2U) {
                scan_kind = 0U;
                ++status.completed; /* Address transactions, not motors found. */
                if (++scan_address == 16U) { scan_address = 1U; ++scan_bus; }
            }
            phase_start_ms = now_ms;
        }
        if (scan_bus == 2U &&
            now_ms - phase_start_ms >= g_motor_config.feedback_max_age_ms) {
            status.phase = CAN_CHECK_DONE;
        }
        publish(now_ms);
        return;
    }
    if (status.phase == CAN_CHECK_IDLE || status.phase == CAN_CHECK_DONE ||
        status.phase == CAN_CHECK_STOPPED) { return; }

    CanCheck_Result_t *r = &status.motor[status.motor_index];
    DM_Motor_t *motor = handles[status.motor_index];
    if (status.phase == CAN_CHECK_QUERY &&
        g_motor_can_config[status.motor_index].protocol ==
            DM_MOTOR_PROTOCOL_LEGACY_MIT) {
        r->reason = CAN_CHECK_QUERY_UNSUPPORTED;
        next_motor(now_ms);
        publish(now_ms);
        return;
    }
    if (!legacy_disable && !legacy_stream &&
        now_ms - last_query_ms >= g_motor_config.query_period_ms) {
        last_query_ms = now_ms;
        if (DM_Motor_RequestFeedback(motor) == BSP_CAN_OK) { ++r->query_submitted; }
        else { ++r->query_failed; }
    }
    if (legacy_stream &&
        (status.phase == CAN_CHECK_ENABLING || status.phase == CAN_CHECK_STREAMING) &&
        now_ms - last_query_ms >= g_motor_config.query_period_ms) {
        last_query_ms = now_ms;
        const DM_Motor_MIT_Command_t neutral = {.position_rad = r->feedback.position_rad};
        if (accepted(r, DM_Motor_SendMIT(motor, &neutral))) { ++r->mit_submitted; }
        else {
            ++r->mit_failed;
            r->reason = CAN_CHECK_TRANSPORT;
            disable_motor(r, now_ms);
        }
    }
    DM_Motor_Feedback_t f;
    if (DM_Motor_GetFeedback(motor, &f) == BSP_CAN_OK) {
        if (r->feedback_valid == 0U) {
            r->minimum_rad = r->maximum_rad = f.position_rad;
        }
        r->feedback_valid = 1U;
        r->feedback = f;
        r->replies = f.received_count - r->feedback_start_count;
        if (f.position_rad < r->minimum_rad) { r->minimum_rad = f.position_rad; }
        if (f.position_rad > r->maximum_rad) { r->maximum_rad = f.position_rad; }
    }
    now_ms = HAL_GetTick(); /* The receive interrupt may be newer than caller. */
    const bool fresh = r->feedback_valid != 0U && r->replies != 0U &&
        now_ms - r->feedback.received_at_ms <= g_motor_config.feedback_max_age_ms;
    (void)BSP_CAN_GetDiagnostics(g_motor_can_config[status.motor_index].bus, &r->can);

    if (status.phase == CAN_CHECK_PREPARING) {
        if (fresh && r->feedback.received_count != command_count &&
            r->feedback.state == 0U) {
            ++r->disable_confirmed;
            if (r->feedback.mos_temperature_c >= g_motor_config.joint[0].temperature_c ||
                r->feedback.rotor_temperature_c >= g_motor_config.joint[0].temperature_c) {
                r->reason = CAN_CHECK_TEMPERATURE;
                next_motor(now_ms);
            } else { enable_motor(r, now_ms); }
        } else if (now_ms - phase_start_ms >= g_motor_config.enable_timeout_ms) {
            r->reason = CAN_CHECK_DISABLE_TIMEOUT;
            status.phase = CAN_CHECK_STOPPED;
        }
    } else if (status.phase == CAN_CHECK_QUERY &&
        now_ms - phase_start_ms >= g_motor_config.enable_timeout_ms) {
        if (query_only) {
            if (!fresh) { r->reason = CAN_CHECK_NO_REPLY; }
            next_motor(now_ms);
        } else if (!fresh) {
            r->reason = CAN_CHECK_NO_REPLY;
            next_motor(now_ms);
        } else if (r->feedback.state != 0U) {
            r->reason = CAN_CHECK_STATE;
            if (r->feedback.state == 1U) { disable_motor(r, now_ms); }
            else { next_motor(now_ms); }
        } else if (r->feedback.mos_temperature_c >= g_motor_config.joint[0].temperature_c ||
                   r->feedback.rotor_temperature_c >= g_motor_config.joint[0].temperature_c) {
            r->reason = CAN_CHECK_TEMPERATURE;
            next_motor(now_ms);
        } else {
            enable_motor(r, now_ms);
        }
    } else if (status.phase == CAN_CHECK_ENABLING) {
        if (fresh && r->feedback.received_count != command_count &&
            r->feedback.state == 1U) {
            r->enable_confirmed = 1U;
            if (legacy_stream) {
                stream_start_count = r->feedback.received_count;
                status.phase = CAN_CHECK_STREAMING;
                phase_start_ms = now_ms;
            } else { disable_motor(r, now_ms); }
        } else if (fresh && r->feedback.state > 1U) {
            r->reason = CAN_CHECK_STATE;
            disable_motor(r, now_ms);
        } else if (now_ms - phase_start_ms >= g_motor_config.enable_timeout_ms) {
            r->reason = CAN_CHECK_ENABLE_TIMEOUT;
            disable_motor(r, now_ms);
        }
    } else if (status.phase == CAN_CHECK_STREAMING) {
        r->stream_replies = r->feedback.received_count - stream_start_count;
        r->stream_elapsed_ms = now_ms - phase_start_ms;
        if (!fresh || r->feedback.state != 1U) {
            r->reason = fresh ? CAN_CHECK_STATE : CAN_CHECK_NO_REPLY;
            disable_motor(r, now_ms);
        } else if (r->feedback.mos_temperature_c >= g_motor_config.joint[0].temperature_c ||
                   r->feedback.rotor_temperature_c >= g_motor_config.joint[0].temperature_c) {
            r->reason = CAN_CHECK_TEMPERATURE;
            disable_motor(r, now_ms);
        } else if (r->stream_elapsed_ms >= MOTOR_CAN_CHECK_STREAM_MS) {
            disable_motor(r, now_ms);
        }
    } else if (status.phase == CAN_CHECK_DISABLING) {
        if (fresh && r->feedback.received_count != command_count &&
            r->feedback.state == 0U) {
            ++r->disable_confirmed;
            next_motor(now_ms);
        } else if (now_ms - phase_start_ms >= g_motor_config.enable_timeout_ms) {
            r->reason = CAN_CHECK_DISABLE_TIMEOUT;
            status.phase = CAN_CHECK_STOPPED; /* No subsequent enables. */
        }
    }
    publish(now_ms);
}
