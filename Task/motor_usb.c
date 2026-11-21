#include "motor_usb.h"
#include "bsp_usb.h"
#include "five_control.h"
#include "motor_telemetry.h"
#include "joint_calibration.h"
#include "yummy_imu.h"
#include "usb_config.h"
#include "motor_snapshot.h"
#include <string.h>
#include <math.h>

_Static_assert(USB_FOCUS_PERIOD_MS>0 && USB_FOCUS_MAX_AGE_MS>0 &&
               USB_FOCUS_MAX_AGE_MS<=UINT16_MAX,"invalid focus cadence/age");
enum { FOCUS_HEADER_SIZE=14, FOCUS_IMU_SIZE=16, FOCUS_IMU_COUNT=2,
       FOCUS_PAYLOAD_SIZE=FOCUS_HEADER_SIZE+MOTOR_CAN_JOINT_COUNT*MOTOR_SNAPSHOT_RECORD_SIZE+
                          FOCUS_IMU_COUNT*FOCUS_IMU_SIZE };
_Static_assert(FOCUS_PAYLOAD_SIZE<=UINT8_MAX &&
               FOCUS_PAYLOAD_SIZE+7<=BSP_USB_TX_CAPACITY,"unified observer exceeds USB capacity");
static struct {
    bool initialized, session_valid;
    uint8_t frame[31], used, session[8], last_request[24], last_response[20];
    uint8_t response[27], response_length;
    uint32_t epoch, last_byte_ms, last_id;
    double target[FIVE_AXES];
    unsigned staged;
    uint32_t staged_at;
    YummyIMU_Status_t imu_snapshot;
    uint32_t imu_snapshot_at;
    bool imu_snapshot_valid;
    uint8_t focus_mode;
    uint32_t focus_at, focus_sequence;
    bool focus_owns_bmi;
} link;

static void end_session(void)
{
    if (link.session_valid) { FiveControl_Gravity(); }
    link.session_valid=false; link.staged=0; link.used=0; link.response_length=0;
    link.focus_mode=0;
    if (link.focus_owns_bmi) {
        (void)JointCalibration_SetIMU(false,HAL_GetTick());
        link.focus_owns_bmi=false; /* Normal asynchronous stop; no retry/reset. */
    }
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 |
        (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}

static void put32(uint8_t *p, uint32_t value)
{
    for (unsigned i=0;i<4;++i) { p[i]=(uint8_t)(value>>(8U*i)); }
}

static void put_float(uint8_t *p, float value)
{
    uint32_t bits; memcpy(&bits,&value,sizeof bits); put32(p,bits);
}

static void focus_imu(uint8_t *p,uint8_t id,uint32_t now,uint32_t at,
                      uint32_t count,float angle,bool ready)
{
    const bool received=count && isfinite(angle);
    p[0]=id;
    p[1]=(received ? 1U : 0U) |
        (received && ready && now-at<=USB_FOCUS_MAX_AGE_MS ? 2U : 0U);
    put32(p+4,received ? at : 0);
    put32(p+8,received ? count : 0);
    put_float(p+12,received ? angle : NAN);
}

static void focus_sample(uint32_t now)
{
    if (now-link.focus_at<USB_FOCUS_PERIOD_MS) { return; }
    link.focus_at=now;
    const FiveControl_Status *s=FiveControl_StatusNow();
    BMI088_Sample_t bmi={0};
    float pitch=NAN;
    if (BMI088_GetSample(&bmi)==HAL_OK &&
        hypotf(bmi.accel_m_s2[0],hypotf(bmi.accel_m_s2[1],bmi.accel_m_s2[2]))>.001f) {
        pitch=atan2f(-bmi.accel_m_s2[0],
            hypotf(bmi.accel_m_s2[1],bmi.accel_m_s2[2]))*57.29577951308232f;
    }
    YummyIMU_Status_t yummy;YummyIMU_GetStatus(&yummy);
    /* Timestamp AFTER copying sources; the envelope is not a hardware trigger. */
    const uint32_t sampled=HAL_GetTick();
    JointCalibration_Status_t cal;
    const bool warm=JointCalibration_GetStatus(sampled,&cal)==BSP_CAN_OK &&
        (cal.flags & (CAL_IMU_FRESH|CAL_WARM))==(CAL_IMU_FRESH|CAL_WARM);
    uint8_t frame[FOCUS_PAYLOAD_SIZE+7]={0xA5,0x5A,1,0x94,FOCUS_PAYLOAD_SIZE};
    uint8_t *p=frame+5;
    put32(p,++link.focus_sequence);put32(p+4,sampled);
    p[8]=MOTOR_CAN_JOINT_COUNT;p[9]=FOCUS_IMU_COUNT;
    p[10]=MOTOR_SNAPSHOT_RECORD_SIZE;p[11]=FOCUS_IMU_SIZE;
    p[12]=(uint8_t)USB_FOCUS_MAX_AGE_MS;p[13]=(uint8_t)(USB_FOCUS_MAX_AGE_MS>>8);
    p+=FOCUS_HEADER_SIZE;
    for (unsigned i=0;i<MOTOR_CAN_JOINT_COUNT;++i) {
        const DM_Motor_Feedback_t *f=s ? &s->feedback[i] : NULL;
        const bool valid=f && f->received_count && isfinite(f->position_rad) &&
            isfinite(f->velocity_rad_s) && isfinite(f->torque_nm);
        const DM_Motor_Config_t *cfg=&g_motor_can_config[i];
        uint8_t packet[MOTOR_SNAPSHOT_LENGTH];
        MotorSnapshot_Encode(packet,MOTOR_CAN_JOINT_COUNT,(uint8_t)i,
            cfg->bus==&hfdcan1 ? 1U : 2U,link.focus_sequence,sampled,
            USB_FOCUS_MAX_AGE_MS,cfg,valid ? f : NULL);
        memcpy(p,packet+16,MOTOR_SNAPSHOT_RECORD_SIZE);p+=MOTOR_SNAPSHOT_RECORD_SIZE;
    }
    focus_imu(p,1,sampled,bmi.received_at_ms,bmi.sequence,pitch,warm);
    focus_imu(p+FOCUS_IMU_SIZE,2,sampled,yummy.received_at_ms,yummy.sequence,
              yummy.pitch_deg,yummy.rx_running);
    const uint16_t crc=MotorTelemetry_CRC16(frame+2,FOCUS_PAYLOAD_SIZE+3);
    frame[sizeof frame-2]=(uint8_t)crc;frame[sizeof frame-1]=(uint8_t)(crc>>8);
    /* BSP copies before returning; BUSY drops this sample, never catches up. */
    (void)BSP_USB_Send(frame,sizeof frame);
}

static void imu_request(const uint8_t *rpc, uint32_t now_ms, uint8_t *reply)
{
    BSP_CAN_Status_t result = BSP_CAN_INVALID_ARG;
    uint32_t value = 0U;
    const uint8_t field = rpc[3];
    if (rpc[2] == 0U && get32(&rpc[4]) == 0U) {
        if (field == 255U) { result = BSP_CAN_OK; value = MOTOR_UART_IMU_VERSION; }
        else if (field <= 9U) {
            if (field == 0U) {
                YummyIMU_GetStatus(&link.imu_snapshot);
                link.imu_snapshot_at = now_ms;
                link.imu_snapshot_valid = true;
            }
            if (!link.imu_snapshot_valid || now_ms - link.imu_snapshot_at > 1000U) {
                result = BSP_CAN_NOT_READY;
            } else {
                const YummyIMU_Status_t *s = &link.imu_snapshot;
                const uint32_t age = s->sequence == 0U ? UINT32_MAX :
                    link.imu_snapshot_at - s->received_at_ms;
                result = BSP_CAN_OK;
                if (field == 0U) {
                    value = 0xB1000000UL | (uint32_t)s->format << 8 |
                        (s->rx_running ? 1U : 0U) | (s->sequence != 0U ? 2U : 0U) |
                        (s->rx_running && age <= 500U ? 4U : 0U);
                } else if (field == 1U) { value = age; }
                else if (field == 2U) { value = s->sequence; }
                else if (field >= 3U && field <= 5U) {
                    const float angles[3] = {s->pitch_deg, s->roll_deg, s->yaw_deg};
                    memcpy(&value, &angles[field - 3U], sizeof(value));
                } else if (field == 6U) { value = s->rx_bytes; }
                else if (field == 7U) { value = s->bad_frames; }
                else if (field == 8U) { value = s->overflows; }
                else { value = s->uart_error; }
            }
        }
    }
    reply[1] = (uint8_t)result;
    memcpy(&reply[4], &value, sizeof(value));
}

static void joint_reference_request(const uint8_t *rpc, uint8_t *reply)
{
    BSP_CAN_Status_t result = BSP_CAN_INVALID_ARG;
    uint32_t value = 0U;
    const uint8_t scope = rpc[2], field = rpc[3];
    if (get32(&rpc[4]) == 0U) {
        if (scope == 0U && field == 255U) {
            result = BSP_CAN_OK;
            value = MOTOR_JOINT_REFERENCE_VERSION;
        } else if (scope >= 1U && scope <= MOTOR_CAN_JOINT_COUNT && field <= 3U) {
            const MotorJointReference_Config_t *r = &g_joint_reference_config[scope - 1U];
            result = r->kind == 0U ? BSP_CAN_NOT_READY : BSP_CAN_OK;
            if (field == 0U) { value = r->kind; }
            else {
                const float fields[] = {
                    r->zero_rad, r->observed_lower_rad, r->observed_upper_rad
                };
                memcpy(&value, &fields[field - 1U], sizeof(value));
            }
        }
    }
    reply[1] = (uint8_t)result;
    for (unsigned i = 0U; i < 4U; ++i) { reply[4U+i] = (uint8_t)(value >> (8U*i)); }
}

static void five_request(const uint8_t *r,uint32_t arrived,uint32_t now,uint8_t *reply)
{
    const FiveControl_Status *s=FiveControl_StatusNow();
    const unsigned scope=r[2], field=r[3];
    const uint32_t arg=get32(r+4);
    uint32_t value=0;
    BSP_CAN_Status_t rc=BSP_CAN_INVALID_ARG;
    if (!s) { rc=BSP_CAN_NOT_READY; }
#ifdef YLAMP_J3_REVERSE_TEST
    else if (!scope && !arg && field==254) { rc=BSP_CAN_OK;value=MOTOR_J3_REVERSE_VERSION; }
#endif
    else if (!scope && !arg && field==253) { rc=BSP_CAN_OK;value=MOTOR_J3_READ_VERSION; }
    else if (!scope && !arg && (field<=3 || field==255)) {
        rc=BSP_CAN_OK;
        if (field==255) { value=MOTOR_FIVE_VERSION; }
        else if (!field) { value=s->phase | (uint32_t)s->fresh<<8 | (uint32_t)s->enabled<<16 | (uint32_t)s->submitted<<24; }
        else if (field==1) { value=s->fault; }
        else if (field==2) { value=s->gravity_valid ? now-s->gravity_at : UINT32_MAX; }
        else { value=s->limited; }
    } else if (scope>=1 && scope<=5 && !arg && field==4) {
        rc=s->gravity_valid ? BSP_CAN_OK : BSP_CAN_NOT_READY;
        if (rc==BSP_CAN_OK) { memcpy(&value,&s->gravity_nm[scope-1],4); }
    } else if (now-arrived>500U) { rc=BSP_CAN_NOT_READY; }
    else if (scope>=1 && scope<=5 && field==5) {
        float deg; memcpy(&deg,&arg,sizeof deg);
        if (isfinite(deg)) {
            if (!link.staged || now-link.staged_at>2000U) { link.staged=0;link.staged_at=now; }
            link.target[scope-1]=deg; link.staged|=1U<<(scope-1); rc=BSP_CAN_OK;
        }
    } else if (!scope && field==6) {
        rc=(link.staged==FIVE_ALL && now-link.staged_at<=2000U) ? FiveControl_Move(link.target,arg) : BSP_CAN_NOT_READY;
        link.staged=0;
    } else if (!scope && !arg && field==7) { FiveControl_Gravity(); link.staged=0;rc=BSP_CAN_OK; }
    else if (!scope && !arg && field==8) { rc=FiveControl_Acknowledge(); }
    else if (!scope && field==9 && arg==MOTOR_COLD_SUPPORTED) { rc=FiveControl_ColdStart(); }
    else if (scope==3 && field==11 && arg==MOTOR_J3_READ_SUPPORTED) { rc=FiveControl_ReadLegacyJ3(); }
#ifdef YLAMP_J3_REVERSE_TEST
    else if (scope==3 && field==10 && arg==MOTOR_J3_REVERSE_SUPPORTED) { rc=FiveControl_ReverseJ3(); }
#endif
    reply[1]=(uint8_t)rc; memcpy(reply+4,&value,4);
}

static void request(uint32_t now_ms)
{
    const uint8_t *p = &link.frame[5];
    uint8_t type = 0x92U, length = 8U;
    if (link.frame[3] == 0x13U) {
        /* Nonce-matched observation command, deliberately outside control
         * SESSION: start/pause must never invoke an actuator or mode switch. */
        type=0x93U; length=18U;
        const uint8_t mode=p[8];
        BSP_CAN_Status_t rc=mode<=2 ? BSP_CAN_OK : BSP_CAN_INVALID_ARG;
        if (mode==1 && !link.focus_owns_bmi) {
            JointCalibration_Status_t cal;
            rc=JointCalibration_GetStatus(now_ms,&cal);
            if (rc==BSP_CAN_OK && (cal.flags & (CAL_IMU_ACTIVE|CAL_IMU_STOPPING|CAL_IMU_STOP_FAILED))) {
                rc=BSP_CAN_BUSY; /* Never take over somebody else's IMU session. */
            }
            if (rc==BSP_CAN_OK) {
                rc=JointCalibration_SetIMU(true,now_ms);
                if (rc==BSP_CAN_OK) { link.focus_owns_bmi=true; }
            }
        } else if (mode==0 && link.focus_owns_bmi) {
            rc=JointCalibration_SetIMU(false,now_ms);
            if (rc==BSP_CAN_OK) { link.focus_owns_bmi=false; }
        }
        if (rc==BSP_CAN_OK) {
            link.focus_mode=mode;
            link.focus_at=now_ms-USB_FOCUS_PERIOD_MS;
        }
        memcpy(link.response+5,p,8);
        link.response[13]=mode; link.response[14]=(uint8_t)rc;
        put32(link.response+15,MOTOR_FOCUS_VERSION);
        put32(link.response+19,now_ms);
    } else if (link.frame[3] == 0x12U) {
        if (!link.session_valid || memcmp(link.session, p, 8U) != 0) {
            end_session();
            memcpy(link.session, p, 8U);
            link.last_id = 0U;
            link.session_valid = true;
        }
        memcpy(&link.response[5], p, 8U);
    } else {
        type = 0x91U; length = 20U;
        uint8_t *response = &link.response[5], *reply = &response[12];
        const uint8_t *rpc = &p[16];
        const uint32_t id = get32(&p[8]);
        memcpy(response, p, 12U);
        memset(reply, 0, 8U);
        reply[0] = rpc[0]; reply[1] = BSP_CAN_NOT_READY;
        reply[2] = rpc[2]; reply[3] = rpc[3];
        if (link.session_valid && memcmp(p, link.session, 8U) == 0 && id != 0U) {
            if (id == link.last_id) {
                if (memcmp(p, link.last_request, 24U) == 0) {
                    memcpy(response, link.last_response, 20U);
                } else { reply[1] = BSP_CAN_INVALID_ARG; }
            } else if (link.last_id != UINT32_MAX && id == link.last_id + 1U) {
                if (rpc[1] == 16U) { five_request(rpc,get32(&p[12]),now_ms,reply); }
                else if (rpc[1] == 15U) { joint_reference_request(rpc, reply); }
                else if (rpc[1] == 14U) { imu_request(rpc, now_ms, reply); }
                else if (rpc[1]==10 && !rpc[2] && rpc[3]<=1 && !get32(rpc+4) && now_ms-get32(&p[12])<=500U) {
                    reply[1]=(uint8_t)JointCalibration_SetIMU(rpc[3]!=0,now_ms);
                } else { reply[1]=BSP_CAN_INVALID_ARG; }
                /* Cache even rejected requests before TX; retry cannot execute
                 * a command twice, or change an expired intent's timestamp. */
                memcpy(link.last_request, p, 24U);
                memcpy(link.last_response, response, 20U);
                link.last_id = id;
            }
        }
    }
    uint8_t *out = link.response;
    out[0] = 0xA5U; out[1] = 0x5AU; out[2] = 1U;
    out[3] = type; out[4] = length;
    const uint16_t crc = MotorTelemetry_CRC16(&out[2], (unsigned)length + 3U);
    out[5U + length] = (uint8_t)crc;
    out[6U + length] = (uint8_t)(crc >> 8);
    link.response_length = (uint8_t)(length + 7U);
}

void MotorUSB_Init(void)
{
    memset(&link, 0, sizeof(link));
    link.initialized = true;
}

bool MotorUSB_Run(void)
{
    if (!link.initialized || __get_IPSR() != 0U) { return true; }
    BSP_USB_Diagnostics_t diag = {0};
    (void)BSP_USB_GetDiagnostics(&diag);
    if (!diag.configured || !diag.dtr || BSP_USB_RxEpoch() != link.epoch) {
        end_session();
    }
    /* Runs even when USB TX is busy or the host is gone; stopping must not
     * depend on the next RPC response getting a telemetry slot. */
    /* Single consumer; ISR only enqueues bytes. Work bounded to 64 bytes/tick.
     * Check the RX epoch before sending a cached reply too. */
    for (unsigned budget = 0U; budget < 64U; ++budget) {
        uint8_t byte;
        uint32_t at, epoch;
        /* A pending reply must not consume the following command. */
        const uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (BSP_USB_RxEpoch() != link.epoch) {
            link.epoch = BSP_USB_RxEpoch();
            end_session();
        }
        if (link.response_length != 0U) {
            BSP_USB_Diagnostics_t diag = {0};
            (void)BSP_USB_GetDiagnostics(&diag);
            if (!diag.dtr || !diag.configured) {
                link.response_length = 0U; link.session_valid = false; link.used = 0U;
            } else {
                const BSP_USB_Status_t sent = BSP_USB_Send(link.response, link.response_length);
                if (sent == BSP_USB_OK) { link.response_length = 0U; }
                __set_PRIMASK(mask);
                return false;
            }
        }
        const BSP_USB_Status_t got = BSP_USB_Read(&byte, &at, &epoch);
        if (epoch != link.epoch) {
            link.epoch = epoch; end_session();
        }
        __set_PRIMASK(mask);
        if (got != BSP_USB_OK) { break; }
        if (link.used != 0U && at - link.last_byte_ms > 100U) { link.used = 0U; }
        link.last_byte_ms = at;
        link.frame[link.used++] = byte;
        while (link.used >= 5U) {
            const uint8_t *p = link.frame;
            const bool header = p[0] == 0xA5U && p[1] == 0x5AU && p[2] == 1U &&
                ((p[3] == 0x12U && p[4] == 8U) || (p[3] == 0x11U && p[4] == 24U) ||
                 (p[3] == 0x13U && p[4] == 9U));
            const unsigned total = (unsigned)p[4] + 7U;
            if (header && link.used < total) { break; }
            if (header && MotorTelemetry_CRC16(&p[2], total - 4U) ==
                (uint16_t)(p[total-2U] | (uint16_t)p[total-1U]<<8)) {
                /* Accept atomically with epoch check, but dispatch outside ISR
                 * and outside IRQ masking. Later overflow invalidates next input. */
                __disable_irq();
                const bool accept = BSP_USB_RxEpoch() == link.epoch;
                __set_PRIMASK(mask);
                const uint32_t now_ms = HAL_GetTick();
                if (accept && now_ms - at <= 500U) { request(now_ms); }
                link.used = 0U;
                return link.response_length == 0U;
            }
            memmove(link.frame, link.frame + 1, --link.used);
        }
    }
    if (link.focus_mode) {
        if (link.focus_mode==1) { focus_sample(HAL_GetTick()); }
        return false; /* Includes PAUSE: no fallback five-axis stream. */
    }
    return true;
}
