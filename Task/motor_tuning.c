#include "motor_tuning.h"
#include "joint_calibration.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

typedef enum { F32, U32, U8 } Storage;
typedef struct {
    uint8_t scope; /* 0 global, 1 both joints */
    uint8_t id;
    size_t offset;
    Storage storage;
    float scale;
    float minimum;
    float maximum;
} Field;

#define DEG 0.01745329251994329577f
#define RAW 1.0f
#define TUNE_JOINT(id, member, type, unit, lo, hi, label, group) \
    {1U, id, offsetof(MotorTask_JointConfig_t, member), type, unit, lo, hi},
#define TUNE_GLOBAL(id, member, type, unit, lo, hi, label, group) \
    {0U, id, offsetof(MotorTask_Config_t, member), type, unit, lo, hi},
static const Field fields[] = {
#include "motor_tuning_fields.def"
};
#undef TUNE_JOINT
#undef TUNE_GLOBAL
#undef DEG
#undef RAW

#define FIELD_COUNT (sizeof(fields) / sizeof(fields[0]))
#define FIFO_CAPACITY 8U

static struct {
    FDCAN_HandleTypeDef *bus;
    BSP_CAN_Frame_t fifo[FIFO_CAPACITY];
    uint32_t arrived[FIFO_CAPACITY];
    volatile uint8_t head;
    volatile uint8_t tail;
    MotorTask_Config_t staged;
    uint32_t written[3]; /* Per-scope field masks, IDs must be <32. */
    uint32_t revision;
    uint32_t staged_at_ms;
    bool staging;
    JointCalibration_Status_t calibration_snapshot;
    uint32_t calibration_snapshot_ms;
    bool have_calibration_snapshot;
    MotorTask_Status_t drag_snapshot;
    uint32_t drag_snapshot_ms;
    bool have_drag_snapshot;
    MotionMode_Request_t motion_request;
    uint8_t motion_written[2];
    bool motion_staging;
    uint32_t motion_staged_ms, motion_generation;
    MotorTask_Status_t motion_snapshot;
    uint32_t motion_snapshot_ms;
    bool have_motion_snapshot;
} tuning;

static uint32_t read_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_u32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i) {
        data[i] = (uint8_t)(value >> (8U * i));
    }
}

static BSP_CAN_Status_t motion_rpc(uint8_t scope, uint8_t id, uint32_t input,
                                   uint32_t now_ms, uint32_t *output)
{
    if (scope == 0U && id == 255U) {
        *output = MOTOR_MOTION_VERSION;
        return BSP_CAN_OK;
    }
    MotorTask_Status_t status;
    BSP_CAN_Status_t result = MotorTask_GetStatus(&status);
    if (result != BSP_CAN_OK) { return result; }
    if (tuning.motion_staging &&
        (now_ms - tuning.motion_staged_ms > 3000U ||
         status.mode_generation != tuning.motion_generation)) {
        tuning.motion_staging = false;
    }
    if (scope == 0U && id == 0U) {
        *output = status.mode;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 1U) {
        tuning.motion_staging = false;
        return tuning.staging ? BSP_CAN_BUSY : MotorTask_SelectMode((MotorMode_Id_t)input);
    }
    if (scope == 0U && id == 8U) {
        tuning.motion_staging = false;
        return tuning.staging || input != tuning.revision ? BSP_CAN_BUSY :
            MotorTask_CaptureHold(now_ms);
    }
    if (scope == 0U && id == 2U) {
        if (tuning.staging || tuning.motion_staging || input != tuning.revision ||
            status.mode != MOTOR_MODE_MOTION || status.state != MOTOR_TASK_HOLDING ||
            status.mode_phase != MOTOR_MODE_HOLDING) { return BSP_CAN_BUSY; }
        memset(&tuning.motion_request, 0, sizeof(tuning.motion_request));
        memset(tuning.motion_written, 0, sizeof(tuning.motion_written));
        tuning.motion_staging = true;
        tuning.motion_staged_ms = now_ms;
        tuning.motion_generation = status.mode_generation;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 4U) {
        tuning.motion_staging = false;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 3U) {
        const bool complete = tuning.motion_staging && !tuning.staging &&
            input == tuning.revision && tuning.motion_written[0] == 15U &&
            tuning.motion_written[1] == 15U;
        tuning.motion_staging = false;
        return complete ? MotorTask_Move(&tuning.motion_request, now_ms) : BSP_CAN_INVALID_ARG;
    }
    if (scope >= 1U && scope <= 2U && id >= 1U && id <= 4U) {
        if (!tuning.motion_staging) { return BSP_CAN_NOT_READY; }
        float value;
        memcpy(&value, &input, sizeof(value));
        value *= 0.01745329251994329577f;
        if (!isfinite(value) || (id != 1U && value <= 0.0f)) {
            return BSP_CAN_INVALID_ARG;
        }
        float *axis = id == 1U ? tuning.motion_request.target_rad :
            id == 2U ? tuning.motion_request.velocity_rad_s :
            id == 3U ? tuning.motion_request.acceleration_rad_s2 :
                        tuning.motion_request.jerk_rad_s3;
        axis[scope - 1U] = value;
        tuning.motion_written[scope - 1U] |= (uint8_t)(1U << (id - 1U));
        tuning.motion_staged_ms = now_ms;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 5U) {
        tuning.motion_snapshot = status;
        tuning.motion_snapshot_ms = now_ms;
        tuning.have_motion_snapshot = true;
        *output = 0xE1000000UL | ((uint32_t)status.mode << 16) |
            ((uint32_t)status.mode_phase << 8) | status.motion.valid;
        return BSP_CAN_OK;
    }
    if (!tuning.have_motion_snapshot ||
        now_ms - tuning.motion_snapshot_ms > 1000U) { return BSP_CAN_NOT_READY; }
    const MotionMode_Status_t *motion = &tuning.motion_snapshot.motion;
    float value = NAN;
    if (scope == 0U && (id == 6U || id == 7U)) {
        if (motion->valid) { value = id == 6U ? motion->elapsed_s : motion->duration_s; }
    } else if (scope >= 1U && scope <= 2U && id >= 11U && id <= 14U) {
        if (motion->valid) {
            const unsigned i = scope - 1U;
            value = id == 11U ? motion->model.gravity_nm[i] :
                    id == 12U ? motion->model.coupling_nm[i] :
                    id == 13U ? motion->model.inertial_nm[i] :
                                motion->target_acceleration_rad_s2[i];
        }
    } else { return BSP_CAN_INVALID_ARG; }
    memcpy(output, &value, sizeof(value));
    return BSP_CAN_OK;
}

static const Field *find_field(uint8_t scope, uint8_t id)
{
    if (scope > MOTOR_TASK_JOINT_COUNT) { return NULL; }
    for (unsigned i = 0U; i < FIELD_COUNT; ++i) {
        if (fields[i].id == id && fields[i].scope == (scope == 0U ? 0U : 1U)) {
            return &fields[i];
        }
    }
    return NULL;
}

static uint8_t *location(MotorTask_Config_t *config, uint8_t scope, const Field *field)
{
    return (scope == 0U ? (uint8_t *)config :
                         (uint8_t *)&config->joint[scope - 1U]) + field->offset;
}

static float get_value(MotorTask_Config_t *config, uint8_t scope, const Field *field)
{
    const uint8_t *data = location(config, scope, field);
    float value;
    if (field->storage == F32) { memcpy(&value, data, sizeof(value)); }
    else if (field->storage == U32) {
        uint32_t integer;
        memcpy(&integer, data, sizeof(integer));
        value = (float)integer;
    } else { value = (float)*data; }
    return value / field->scale;
}

static BSP_CAN_Status_t set_value(uint8_t scope, const Field *field, float value)
{
    if (!isfinite(value) || value < field->minimum || value > field->maximum ||
        (field->storage != F32 && value != floorf(value))) {
        return BSP_CAN_INVALID_ARG;
    }
    uint8_t *data = location(&tuning.staged, scope, field);
    value *= field->scale;
    if (field->storage == F32) { memcpy(data, &value, sizeof(value)); }
    else if (field->storage == U32) {
        const uint32_t integer = (uint32_t)value;
        memcpy(data, &integer, sizeof(integer));
    } else { *data = (uint8_t)value; }
    tuning.written[scope] |= 1UL << field->id;
    return BSP_CAN_OK;
}

static BSP_CAN_Status_t read_calibration(uint8_t scope, uint8_t id,
                                         uint32_t now_ms, uint32_t *output)
{
    if (scope == 0U && id == 255U) {
        *output = JOINT_CALIBRATION_VERSION;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 0U) {
        const BSP_CAN_Status_t status = JointCalibration_GetStatus(
            now_ms, &tuning.calibration_snapshot);
        tuning.have_calibration_snapshot = status == BSP_CAN_OK;
        tuning.calibration_snapshot_ms = now_ms;
        if (status != BSP_CAN_OK) { return status; }
    } else if (!tuning.have_calibration_snapshot ||
               now_ms - tuning.calibration_snapshot_ms > 1000U) {
        return BSP_CAN_NOT_READY;
    }
    const JointCalibration_Status_t *snapshot = &tuning.calibration_snapshot;
    if (scope == 0U && id <= 2U) {
        *output = id == 0U ? 0xC1000000UL | snapshot->flags |
            ((uint32_t)snapshot->axis << 8) | ((uint32_t)snapshot->valid_mask << 16) :
            id == 1U ? snapshot->imu_age_ms : snapshot->stable_ms;
        return BSP_CAN_OK;
    }
    float value;
    if (scope == 0U && id == 3U) {
        value = g_calibration_config.vertical_tolerance_rad * 57.295779513f;
    } else if (scope == 0U && id >= 4U && id <= 9U) {
        value = id <= 6U ? snapshot->accel_m_s2[id - 4U] :
                          snapshot->gyro_rad_s[id - 7U] * 57.295779513f;
    } else if (scope >= 1U && scope <= 2U && id >= 1U && id <= 4U) {
        const unsigned joint = scope - 1U;
        value = id == 1U ? snapshot->zero_rad[joint] :
            id == 2U ? snapshot->raw_rad[joint] :
            id == 3U ? snapshot->reference_rad[joint] - snapshot->zero_rad[joint] :
            snapshot->reference_rad[joint];
        value *= 57.295779513f;
    } else { return BSP_CAN_INVALID_ARG; }
    memcpy(output, &value, sizeof(*output));
    return BSP_CAN_OK;
}

/* Read-only, frozen for one second. Requested MIT values are not measured torque.
 * c and M*ddq fields remain UNSCALED nominal components, not applied torque.
 * Requested FF includes gravity plus the bounded partial dynamics contribution. */
static BSP_CAN_Status_t read_drag(uint8_t scope, uint8_t id,
                                 uint32_t now_ms, uint32_t *output)
{
    if (scope == 0U && id == 255U) {
        *output = 0x594C4402UL;
        return BSP_CAN_OK;
    }
    if (scope == 0U && id == 0U) {
        MotorTask_Config_t config;
        BSP_CAN_Status_t result = MotorTask_GetStatus(&tuning.drag_snapshot);
        if (result == BSP_CAN_OK) { result = MotorTask_GetConfig(&config); }
        tuning.have_drag_snapshot = result == BSP_CAN_OK;
        tuning.drag_snapshot_ms = now_ms;
        if (result != BSP_CAN_OK) { return result; }
        for (unsigned i = 0U; i < 2U; ++i) {
            if ((tuning.drag_snapshot.feedback_valid_mask & (1U << i)) == 0U ||
                now_ms - tuning.drag_snapshot.feedback[i].received_at_ms >
                    config.feedback_max_age_ms) { tuning.drag_snapshot.drag.valid = false; }
        }
        *output = 0xD1000000UL | (uint32_t)tuning.drag_snapshot.drag.valid |
            ((uint32_t)tuning.drag_snapshot.drag.phase << 8) |
            ((uint32_t)tuning.drag_snapshot.state << 16);
        return BSP_CAN_OK;
    }
    if (!tuning.have_drag_snapshot || now_ms - tuning.drag_snapshot_ms > 1000U) {
        return BSP_CAN_NOT_READY;
    }
    const MotorTask_Status_t *snapshot = &tuning.drag_snapshot;
    float value = NAN;
    if (scope == 0U && id >= 1U && id <= 3U) {
        if (snapshot->drag.valid) { value = snapshot->drag.inertia[id - 1U]; }
    } else if (scope >= 1U && scope <= 2U && id >= 1U && id <= 14U) {
        const unsigned i = scope - 1U;
        if (snapshot->drag.valid &&
            (id <= 6U || id >= 13U || snapshot->drag.phase != DRAG_MODE_OFF)) {
            switch (id) {
            case 1: value = snapshot->feedback[i].position_rad * 57.295779513f; break;
            case 2: value = snapshot->feedback[i].velocity_rad_s * 57.295779513f; break;
            case 3: value = snapshot->feedback[i].torque_nm; break;
            case 4: value = snapshot->drag.gravity_nm[i]; break;
            case 5: value = snapshot->drag.velocity_coupling_nm[i]; break;
            case 6: value = snapshot->drag.inertial_nm[i]; break;
            case 7: value = snapshot->target_torque_ff_nm[i]; break;
            case 8: value = snapshot->target_kd[i]; break;
            case 9: value = snapshot->target_kp[i]; break;
            case 10: value = snapshot->target_rad[i] * 57.295779513f; break;
            case 11: value = (snapshot->target_reference_rad[i] -
                snapshot->reference_rad[i]) * 57.295779513f; break;
            case 12: value = -snapshot->target_kd[i] *
                snapshot->feedback[i].velocity_rad_s; break;
            case 13: value = snapshot->reference_rad[i] * 57.295779513f; break;
            case 14: value = snapshot->reference_offset_rad[i] * 57.295779513f; break;
            default: break;
            }
        }
    } else { return BSP_CAN_INVALID_ARG; }
    memcpy(output, &value, sizeof(*output));
    return BSP_CAN_OK;
}

static void receive(FDCAN_HandleTypeDef *bus, const BSP_CAN_Frame_t *frame,
                     void *context)
{
    (void)bus;
    (void)context;
    if (frame->len != 8U) { return; }
    const uint8_t next = (uint8_t)((tuning.head + 1U) % FIFO_CAPACITY);
    if (next == tuning.tail) { return; } /* Drop, never overwrite an action. */
    tuning.fifo[tuning.head] = *frame;
    tuning.arrived[tuning.head] = HAL_GetTick();
    __DMB();
    tuning.head = next;
}

BSP_CAN_Status_t MotorTuning_Init(FDCAN_HandleTypeDef *bus)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (tuning.revision != 0U) { return BSP_CAN_BUSY; }
    MotorTask_Config_t current;
    BSP_CAN_Status_t result = MotorTask_GetConfig(&current);
    if (result != BSP_CAN_OK) { return result; }
    if (bus != NULL) {
        result = BSP_CAN_RegisterRxCallback(bus, MOTOR_TUNING_REQUEST_ID, receive, NULL);
    }
    if (result == BSP_CAN_OK) {
        tuning.bus = bus;
        tuning.revision = 1U;
    }
    return result;
}

BSP_CAN_Status_t MotorTuning_Run(uint32_t now_ms)
{
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (tuning.revision == 0U) { return BSP_CAN_NOT_READY; }
    if (tuning.staging && now_ms - tuning.staged_at_ms > 3000U) {
        tuning.staging = false;
    }
    if (tuning.bus == NULL) { return BSP_CAN_OK; }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (tuning.tail == tuning.head) {
        __set_PRIMASK(primask);
        return BSP_CAN_OK;
    }
    const BSP_CAN_Frame_t request = tuning.fifo[tuning.tail];
    const uint32_t arrived = tuning.arrived[tuning.tail];
    tuning.tail = (uint8_t)((tuning.tail + 1U) % FIFO_CAPACITY);
    __set_PRIMASK(primask);

    BSP_CAN_Frame_t response = {.id = MOTOR_TUNING_RESPONSE_ID, .len = 8U};
    const BSP_CAN_Status_t result = MotorTuning_HandleRequest(
        request.data, arrived, HAL_GetTick(), response.data);
    return result == BSP_CAN_OK ? BSP_CAN_Send(tuning.bus, &response) : result;
}

BSP_CAN_Status_t MotorTuning_HandleRequest(const uint8_t request[8],
    uint32_t arrived_ms, uint32_t now_ms, uint8_t response[8])
{
    if (__get_IPSR() != 0U || request == NULL || response == NULL) {
        return BSP_CAN_INVALID_ARG;
    }
    if (tuning.revision == 0U) { return BSP_CAN_NOT_READY; }
    if (tuning.staging && now_ms - tuning.staged_at_ms > 3000U) {
        tuning.staging = false;
    }
    response[0] = request[0];
    response[2] = request[2];
    response[3] = request[3];
    const uint8_t op = request[1], scope = request[2], id = request[3];
    const uint32_t input = read_u32(&request[4]);
    uint32_t output = 0U;
    BSP_CAN_Status_t result = BSP_CAN_INVALID_ARG;
    const Field *field = find_field(scope, id);
    MotorTask_Config_t current;
    MotorTask_Status_t status;
    if (now_ms - arrived_ms > 500U) {
        result = BSP_CAN_NOT_READY; /* Do not execute a queued stale START. */
    } else if (op == 0U && scope == 0U && id == 0U) {
        output = MOTOR_TUNING_VERSION;
        result = BSP_CAN_OK;
    } else if (op == 1U && scope >= 1U && scope <= MOTOR_TASK_JOINT_COUNT &&
               id >= 240U && id <= 242U) {
        /* MCU owns 0x7FF motor queries. The PC reads our decode scales by RPC
         * instead of issuing competing motor-register requests on that ID. */
        result = MotorTask_GetConfig(&current);
        if (result == BSP_CAN_OK) {
            const DM_Motor_Config_t *device = &current.joint[scope - 1U].device;
            const float value = id == 240U ? device->pmax :
                                id == 241U ? device->vmax : device->tmax;
            memcpy(&output, &value, sizeof(output));
        }
    } else if (op == 1U && field != NULL) {
        result = MotorTask_GetConfig(&current);
        if (result == BSP_CAN_OK) {
            const float value = get_value(&current, scope, field);
            memcpy(&output, &value, sizeof(output));
        }
    } else if (op == 2U && scope == 0U && id == 0U) {
        tuning.motion_staging = false;
        if (tuning.staging || input != tuning.revision) { result = BSP_CAN_BUSY; }
        else {
            result = MotorTask_GetConfig(&tuning.staged);
            if (result == BSP_CAN_OK) {
                memset(tuning.written, 0, sizeof(tuning.written));
                tuning.staging = true;
                tuning.staged_at_ms = now_ms;
            }
        }
    } else if (op == 3U && field != NULL && tuning.staging) {
        float value;
        memcpy(&value, &input, sizeof(value));
        result = set_value(scope, field, value);
        tuning.staged_at_ms = now_ms;
    } else if (op == 4U && scope == 0U && id == 0U && tuning.staging) {
        bool complete = true;
        for (unsigned scope_i = 0U; scope_i <= MOTOR_TASK_JOINT_COUNT; ++scope_i) {
            for (unsigned i = 0U; i < FIELD_COUNT; ++i) {
                if (fields[i].scope == (scope_i == 0U ? 0U : 1U) &&
                    (tuning.written[scope_i] & (1UL << fields[i].id)) == 0U) {
                    complete = false;
                }
            }
        }
        result = complete ? MotorTask_ApplyConfig(&tuning.staged, now_ms) :
                            BSP_CAN_INVALID_ARG;
        tuning.staging = false; /* Failed commit also needs a new explicit BEGIN. */
        if (result == BSP_CAN_OK) { ++tuning.revision; }
        output = tuning.revision;
    } else if (op == 5U && scope == 0U && id == 0U) {
        tuning.staging = false;
        result = BSP_CAN_OK;
    } else if (op == 6U && scope == 0U && id <= 1U) {
        result = MotorTask_GetStatus(&status);
        if (result == BSP_CAN_OK) {
            output = id == 1U ? tuning.revision :
                (uint32_t)status.state | ((uint32_t)status.reason << 8) |
                ((uint32_t)status.policy_ready << 16) |
                ((uint32_t)status.feedback_valid_mask << 24);
        }
    } else if (op == 7U && scope == 0U) {
        tuning.motion_staging = false;
        if (id == MOTOR_COMMAND_START &&
            (tuning.staging || input != tuning.revision)) { result = BSP_CAN_BUSY; }
        else { result = MotorTask_Command((MotorTask_Command_t)id, now_ms); }
    } else if (op == 8U) {
        result = read_calibration(scope, id, now_ms, &output);
    } else if (op == 9U && (scope == 1U || scope == 2U)) {
        if (tuning.staging || input != tuning.revision) { result = BSP_CAN_BUSY; }
        else if (scope == 1U && id != 0U) { result = BSP_CAN_INVALID_ARG; }
        else { result = JointCalibration_Mark(scope, id, now_ms); }
        /* Offsets never change motion config/revision, enable state or limits. */
        tuning.have_calibration_snapshot = false;
    } else if (op == 10U && scope == 0U && id <= 1U) {
        if (id == 1U && (tuning.staging || input != tuning.revision)) {
            result = BSP_CAN_BUSY;
        } else { result = JointCalibration_SetIMU(id == 1U, now_ms); }
        tuning.have_calibration_snapshot = false;
    } else if (op == 11U) {
        result = read_drag(scope, id, now_ms, &output);
    } else if (op == 12U) {
        result = motion_rpc(scope, id, input, now_ms, &output);
    }
    response[1] = (uint8_t)result;
    write_u32(&response[4], output);
    return BSP_CAN_OK;
}
