#include "motor_usb.h"
#include "bsp_usb.h"
#include "five_control.h"
#include "motor_telemetry.h"
#include "joint_calibration.h"
#include "yummy_imu.h"
#include "usb_config.h"
#include "motor_snapshot.h"
#include "five_history.h"
#include "lookaround_store.h"
#include <string.h>
#include <math.h>

_Static_assert(USB_FOCUS_PERIOD_MS>0 && USB_FOCUS_MAX_AGE_MS>0 &&
               USB_FOCUS_MAX_AGE_MS<=UINT16_MAX,"invalid focus cadence/age");
enum { FOCUS_HEADER_SIZE=14, FOCUS_IMU_SIZE=16, FOCUS_IMU_COUNT=2,
       FOCUS_PAYLOAD_SIZE=FOCUS_HEADER_SIZE+MOTOR_CAN_JOINT_COUNT*MOTOR_SNAPSHOT_RECORD_SIZE+
                          FOCUS_IMU_COUNT*FOCUS_IMU_SIZE };
_Static_assert(FOCUS_PAYLOAD_SIZE<=UINT8_MAX &&
               FOCUS_PAYLOAD_SIZE+7<=BSP_USB_TX_CAPACITY,"unified observer exceeds USB capacity");
_Static_assert(MOTOR_USB_MOTION_STATE_LEN+7<=UINT8_MAX &&
               MOTOR_USB_MOTION_STATE_LEN+7<=BSP_USB_TX_CAPACITY &&
               MOTOR_USB_TRAJ_POINT_LEN+7<=255 && /* 256-byte BSP ring, one slot reserved. */
               FIVE_AXES==5 && FIVE_TRAJECTORY_CAPACITY==32 && sizeof(float)==4,
               "bundled motion wire dimensions");
static struct {
    bool initialized, session_valid;
    uint8_t frame[MOTOR_USB_TRAJ_POINT_LEN+7], used, session[8];
    uint8_t last_type, last_request[MOTOR_USB_TRAJ_POINT_LEN];
    uint8_t last_response[MOTOR_USB_MOTION_STATE_LEN];
    uint8_t response[MOTOR_USB_MOTION_STATE_LEN+7], response_length;
    uint32_t epoch, last_byte_ms, last_id, plan_id, start_id, preset_id;
    double target[FIVE_AXES];
    unsigned staged;
    uint32_t staged_at, cruise_ramp_ms;
    unsigned speed_joint;
    float speed_rad_s;
    YummyIMU_Status_t imu_snapshot;
    uint32_t imu_snapshot_at;
    bool imu_snapshot_valid;
    uint8_t focus_mode;
    uint32_t focus_at, focus_sequence;
    bool focus_owns_bmi;
} link;

static void end_session(void)
{
    if (link.session_valid) { FiveControl_Release(); }
    link.session_valid=false; link.staged=0; link.used=0; link.response_length=0;
    link.plan_id=0;link.start_id=0;
    link.preset_id=0;LookaroundStore_CancelUpload();
    link.cruise_ramp_ms=0;link.speed_joint=0;
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
    else if (!scope && !arg && field==253) { rc=BSP_CAN_OK;value=MOTOR_J3_READ_VERSION; }
    else if (!scope && !arg && field==252) { rc=BSP_CAN_OK;value=MOTOR_CRUISE_VERSION; }
    else if (!scope && !arg && field==251) { rc=BSP_CAN_OK;value=MOTOR_DRAG_VERSION; }
    else if (!scope && !arg && field==250) { rc=BSP_CAN_OK;value=MOTOR_HISTORY_VERSION; }
    else if (!scope && !arg && field==249) { rc=BSP_CAN_OK;value=MOTOR_IMPEDANCE_TEST_VERSION; }
    else if (!scope && !arg && field==248) { rc=BSP_CAN_OK;value=MOTOR_TRAJECTORY_VERSION; }
    else if (!scope && !arg && field==247) { rc=BSP_CAN_OK;value=MOTOR_MOTION_BUNDLE_VERSION; }
    else if (!scope && !arg && field==246) { rc=BSP_CAN_OK;value=MOTOR_LOOKAROUND_VERSION; }
    else if (!arg && ((!scope && field>=36 && field<=39) ||
             (scope>=1 && scope<=5 && (field==40 || field==42)) ||
             (scope>=1 && scope<=FIVE_TRAJECTORY_CAPACITY && field==41))) {
        const FiveTrajectory_Status path=FiveControl_TrajectoryStatus();
        rc=BSP_CAN_OK;
        if (field==36) {
            value=path.count|path.expected<<8|(uint32_t)path.uploading<<16|(uint32_t)path.active<<17;
        } else if (field==37) { value=path.start_ms; }
        else if (field==38) { value=path.duration_ms; }
        else if (field==39) { value=path.checksum; }
        else if (!path.active) { rc=BSP_CAN_NOT_READY; }
        else if (field==40) { memcpy(&value,&path.origin_deg[scope-1],4); }
        else if (field==42) { memcpy(&value,&path.origin_velocity_rad_s[scope-1],4); }
        else if (scope>path.count) { rc=BSP_CAN_INVALID_ARG; }
        else { value=path.knot_ms[scope-1]; }
    }
    else if (!arg && !scope && (field==24 || field==25)) {
        rc=BSP_CAN_OK;
        value=field==24 ? s->impedance_test_joint : s->impedance_test_started_ms;
    } else if (!arg && scope>=1 && scope<=5 && field==24 && s->phase==FIVE_IMPEDANCE) {
        rc=BSP_CAN_OK;memcpy(&value,&s->impedance_target_raw_rad[scope-1],4);
    }
    else if (!arg && field>=16 && field<=21) {
        const FiveHistory_Status *history=FiveHistory_StatusNow();
        if (!scope && field<=20) {
            rc=BSP_CAN_OK;
            if (field==16) { value=history->flags; }
            else if (field==17) { value=history->sequence; }
            else if (field==18) { value=history->errors; }
            else if (field==19) { value=history->writes; }
            else { value=history->saved_mask|((uint32_t)history->restored_mask<<8); }
        } else if (scope>=1 && scope<=5) {
            const unsigned i=scope-1;
            float scalar=NAN;
            if (field==18) { scalar=history->step_deg[i]; }
            else if (field==19) { value=history->corrections[i];rc=BSP_CAN_OK; }
            else if (field==17 && (history->saved_mask&(1U<<i))) { scalar=history->saved_rad[i]; }
            else if (s->fresh&(1U<<i)) {
                const float q=s->reference_rad[i];
                const float turns=floorf(q/6.2831853071795864769f);
                if (field==16) { scalar=q; }
                else if (field==20 && isfinite(turns) && fabsf(turns)<2147483000.f) {
                    value=(uint32_t)(int32_t)turns;rc=BSP_CAN_OK;
                } else if (field==21) { scalar=q-turns*6.2831853071795864769f; }
            }
            if (isfinite(scalar)) { memcpy(&value,&scalar,4);rc=BSP_CAN_OK; }
        }
    }
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
    } else if (link.plan_id &&
               ((scope>=1 && scope<=5 && field>=30 && field<=32) ||
                (!scope && (field==33 || field==35)))) {
        /* A bundled upload cannot be modified or started by legacy setters. */
        rc=BSP_CAN_INVALID_ARG;
    } else if (now-arrived>500U) {
        rc=BSP_CAN_NOT_READY;
        if (!scope && (field==6 || field==13)) {
            link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
        }
        if (!scope && field==35) { FiveControl_TrajectoryDiscard(); }
    }
    else if (scope>=1 && scope<=5 && field==5) {
        float deg; memcpy(&deg,&arg,sizeof deg);
        if (isfinite(deg)) {
            if (!link.staged || now-link.staged_at>2000U) {
                link.staged=0;link.staged_at=now;link.cruise_ramp_ms=0;link.speed_joint=0;
            }
            link.target[scope-1]=deg; link.staged|=1U<<(scope-1); rc=BSP_CAN_OK;
        }
    } else if (scope>=1 && scope<=5 && field>=30 && field<=32) {
        float scalar;memcpy(&scalar,&arg,4);
        rc=FiveControl_TrajectorySet(scope-1,field-30,scalar);
    } else if (!scope && field==34) {
        rc=FiveControl_TrajectoryBegin(arg);
        if (rc==BSP_CAN_OK) { link.plan_id=0;link.start_id=0; }
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    } else if (!scope && field==33) {
        rc=FiveControl_TrajectoryAppend(arg);
        if (rc==BSP_CAN_OK) { value=FiveControl_TrajectoryStatus().count; }
    } else if (!scope && field==35 && arg==MOTOR_TRAJECTORY_REQUEST) {
        rc=FiveControl_TrajectoryStart();
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    } else if (!scope && field==6) {
        rc=(link.staged==FIVE_ALL && now-link.staged_at<=2000U) ? FiveControl_Move(link.target,arg) : BSP_CAN_NOT_READY;
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    } else if (scope>=1 && scope<=5 && field==12) {
        float velocity;memcpy(&velocity,&arg,sizeof velocity);
        link.staged=0;link.speed_joint=0;link.cruise_ramp_ms=0;
        if (isfinite(velocity) && velocity!=0) {
            link.speed_joint=scope;link.speed_rad_s=velocity;link.staged_at=now;rc=BSP_CAN_OK;
        }
    } else if (!scope && field==14) {
        if (link.speed_joint && now-link.staged_at<=2000U && arg>0 && arg<60000U) {
            link.cruise_ramp_ms=arg;rc=BSP_CAN_OK;
        } else { link.cruise_ramp_ms=0; }
    } else if (!scope && field==13) {
        rc=(link.speed_joint && now-link.staged_at<=2000U && link.cruise_ramp_ms) ?
            FiveControl_Speed(link.speed_joint,link.speed_rad_s,arg,link.cruise_ramp_ms) : BSP_CAN_NOT_READY;
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    } else if (!scope && !arg && field==7) {
        FiveControl_Idle();link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;rc=BSP_CAN_OK;
    }
    else if (!scope && !arg && field==8) { rc=FiveControl_Acknowledge(); }
    else if (!scope && field==22 && arg==MOTOR_IMPEDANCE_REQUEST) {
        rc=FiveControl_Impedance();
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    } else if (scope>=1 && scope<=5 && field==23) {
        float offset;memcpy(&offset,&arg,4);
        rc=FiveControl_ImpedanceTest(scope,offset);
        if (rc==BSP_CAN_OK) { value=s->impedance_test_duration_ms; }
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    }
    else if (!scope && field==15 && arg==MOTOR_DRAG_REQUEST) {
        rc=FiveControl_Drag();
        link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    }
    else if (!scope && field==9 && arg==MOTOR_COLD_SUPPORTED) { rc=FiveControl_ColdStart(); }
    else if (scope==3 && field==11 && arg==MOTOR_J3_READ_SUPPORTED) { rc=FiveControl_ReadLegacyJ3(); }
    reply[1]=(uint8_t)rc; memcpy(reply+4,&value,4);
}

static uint8_t path_flags(const FiveTrajectory_Status *path)
{
    return (path->uploading ? 1U : 0U) | (path->active ? 2U : 0U);
}

static void motion_state(uint8_t *out,const FiveControl_Status *s)
{
    const FiveTrajectory_Status path=FiveControl_TrajectoryStatus();
    const uint32_t sampled=HAL_GetTick();
    out[13]=(uint8_t)s->phase;out[14]=s->fresh;out[15]=s->enabled;
    out[16]=s->submitted;out[17]=s->limited;
    out[18]=path_flags(&path);out[19]=(uint8_t)path.count;out[20]=(uint8_t)path.expected;
    put32(out+24,sampled);put32(out+28,s->fault);
    put32(out+32,s->gravity_valid ? sampled-s->gravity_at : UINT32_MAX);
    put32(out+36,out[18] ? link.plan_id : 0);
    put32(out+40,path.active ? link.start_id : 0);
    put32(out+44,path.checksum);put32(out+48,path.start_ms);put32(out+52,path.duration_ms);
    for (unsigned i=0;i<FIVE_AXES;++i) {
        if (s->gravity_valid) { put_float(out+56+4*i,s->gravity_nm[i]); }
        if (path.active) {
            put_float(out+76+4*i,path.origin_deg[i]);
            put_float(out+96+4*i,path.origin_velocity_rad_s[i]);
        }
    }
    if (path.active) {
        for (unsigned i=0;i<path.count;++i) { put32(out+116+4*i,path.knot_ms[i]); }
    }
}

static BSP_CAN_Status_t trajectory_write(uint8_t type,const uint8_t *p,uint32_t now)
{
    const uint32_t id=get32(p+8);
    if (type==MOTOR_USB_TRAJ_BEGIN) {
        if (now-get32(p+12)>500U) { return BSP_CAN_NOT_READY; }
        const BSP_CAN_Status_t rc=FiveControl_TrajectoryBegin(p[16]);
        if (rc==BSP_CAN_OK) {
            link.plan_id=id;link.start_id=0;
            link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
        }
        return rc;
    }
    const FiveTrajectory_Status path=FiveControl_TrajectoryStatus();
    if (!link.plan_id || get32(p+16)!=link.plan_id ||
        !path.uploading || path.active) { return BSP_CAN_NOT_READY; }
    if (now-get32(p+12)>500U) {
        if (type==MOTOR_USB_TRAJ_START) { FiveControl_TrajectoryDiscard(); }
        return BSP_CAN_NOT_READY;
    }
    if (type==MOTOR_USB_TRAJ_POINT) {
        if (p[20]!=path.count) { return BSP_CAN_INVALID_ARG; }
        FiveTrajectory_Point point;FiveTrajectory_DecodePoint(&point,p+21);
        return FiveControl_TrajectoryAppendPoint(&point);
    }
    if (path.count!=path.expected || path.checksum!=get32(p+20)) { return BSP_CAN_INVALID_ARG; }
    const BSP_CAN_Status_t rc=FiveControl_TrajectoryStart();
    link.staged=0;link.cruise_ramp_ms=0;link.speed_joint=0;
    if (rc==BSP_CAN_OK) { link.start_id=id; }
    return rc;
}

static void preset_state(uint8_t *out)
{
    const LookaroundStore_Status *s=LookaroundStore_StatusNow();
    out[13]=(uint8_t)s->staged_count;out[14]=(uint8_t)s->expected;out[15]=(uint8_t)s->count;
    put32(out+16,s->flags);put32(out+20,s->sequence);put32(out+24,s->checksum);
    put32(out+28,s->staged_checksum);put32(out+32,link.preset_id);put32(out+36,s->errors);
}
static BSP_CAN_Status_t preset_write(uint8_t type,const uint8_t *p,uint32_t now)
{
    const FiveControl_Status *s=FiveControl_StatusNow();
    if (!s || now-get32(p+12)>500U || FiveControl_LocalTrajectoryOwned() ||
        s->phase==FIVE_MOTION || s->phase==FIVE_POSITION ||
        FiveControl_TrajectoryStatus().uploading) { return BSP_CAN_NOT_READY; }
    if (type==MOTOR_USB_PRESET_BEGIN) {
        if (!LookaroundStore_Begin(p[16])) { return BSP_CAN_NOT_READY; }
        link.preset_id=get32(p+8);return BSP_CAN_OK;
    }
    if (!link.preset_id || link.preset_id!=get32(p+16)) { return BSP_CAN_NOT_READY; }
    return (type==MOTOR_USB_PRESET_POINT ? LookaroundStore_Append(p[20],p+21) :
        LookaroundStore_Save(get32(p+20))) ? BSP_CAN_OK : BSP_CAN_INVALID_ARG;
}
static uint8_t request_length(uint8_t type)
{
    switch (type) {
    case 0x11U: return 24U;
    case 0x12U: return 8U;
    case 0x13U: return 9U;
    case MOTOR_USB_TRAJ_BEGIN: return MOTOR_USB_TRAJ_BEGIN_LEN;
    case MOTOR_USB_TRAJ_POINT: return MOTOR_USB_TRAJ_POINT_LEN;
    case MOTOR_USB_TRAJ_START: return MOTOR_USB_TRAJ_START_LEN;
    case MOTOR_USB_MOTION_QUERY: return MOTOR_USB_MOTION_QUERY_LEN;
    case MOTOR_USB_PRESET_BEGIN: return MOTOR_USB_TRAJ_BEGIN_LEN;
    case MOTOR_USB_PRESET_POINT: return MOTOR_USB_TRAJ_POINT_LEN;
    case MOTOR_USB_PRESET_SAVE: return MOTOR_USB_TRAJ_START_LEN;
    case MOTOR_USB_PRESET_QUERY: return MOTOR_USB_MOTION_QUERY_LEN;
    case MOTOR_USB_PRESET_READ: return MOTOR_USB_PRESET_READ_LEN;
    default: return 0;
    }
}

static void session_request(uint8_t req,const uint8_t *p,uint32_t now,
                            uint8_t *out,uint8_t length)
{
    const uint32_t id=get32(p+8);
    const bool legacy=req==0x11U;
    uint8_t *rc=out+(legacy ? 13 : 12);
    memcpy(out,p,12);
    memset(out+12,0,length-12);
    *rc=BSP_CAN_NOT_READY;
    if (legacy) {
        out[12]=p[16];out[14]=p[18];out[15]=p[19];
    }
    if (!link.session_valid || memcmp(p,link.session,8)!=0 || !id) { return; }
    if (req==MOTOR_USB_MOTION_QUERY) {
        const FiveControl_Status *s=FiveControl_StatusNow();
        if (s) { *rc=BSP_CAN_OK;motion_state(out,s); }
        return;
    }
    if (req==MOTOR_USB_PRESET_QUERY) { *rc=BSP_CAN_OK;preset_state(out);return; }
    if (req==MOTOR_USB_PRESET_READ) {
        const uint32_t offset=get32(p+12);
        put32(out+16,offset);
        if (offset>=8192) { *rc=BSP_CAN_INVALID_ARG;return; }
        const unsigned n=8192-offset>224 ? 224 : 8192-offset;
        out[13]=(uint8_t)n;
        *rc=LookaroundStore_Read(offset,out+20,n) ? BSP_CAN_OK : BSP_CAN_BUSY;
        return;
    }
    const uint8_t size=request_length(req);
    if (id==link.last_id) {
        if (req==link.last_type && memcmp(p,link.last_request,size)==0) {
            memcpy(out,link.last_response,length);
        } else { *rc=BSP_CAN_INVALID_ARG; }
        return;
    }
    if (link.last_id==UINT32_MAX || id!=link.last_id+1U) { return; }
    if (legacy) {
        const uint8_t *rpc=p+16;
        if (rpc[1]==16U) { five_request(rpc,get32(p+12),now,out+12); }
        else if (rpc[1]==15U) { joint_reference_request(rpc,out+12); }
        else if (rpc[1]==14U) { imu_request(rpc,now,out+12); }
        else if (rpc[1]==10 && !rpc[2] && rpc[3]<=1 && !get32(rpc+4) && now-get32(p+12)<=500U) {
            *rc=(uint8_t)JointCalibration_SetIMU(rpc[3]!=0,now);
        } else { *rc=BSP_CAN_INVALID_ARG; }
    } else if (req>=MOTOR_USB_PRESET_BEGIN && req<=MOTOR_USB_PRESET_SAVE) {
        *rc=(uint8_t)preset_write(req,p,now);preset_state(out);
    } else {
        *rc=(uint8_t)trajectory_write(req,p,now);
        if (*rc==BSP_CAN_OK) {
            if (req==MOTOR_USB_TRAJ_START) {
                motion_state(out,FiveControl_StatusNow());
            } else {
                const FiveTrajectory_Status path=FiveControl_TrajectoryStatus();
                out[13]=(uint8_t)path.count;out[14]=(uint8_t)path.expected;
                out[15]=path_flags(&path);
                put32(out+16,link.plan_id);put32(out+20,path.checksum);
            }
        }
    }
    /* Cache even rejected writes. Read-only queries do not advance this ID. */
    memcpy(link.last_request,p,size);
    memcpy(link.last_response,out,length);
    link.last_type=req;link.last_id=id;
}

static void request(uint32_t now_ms)
{
    const uint8_t *p = &link.frame[5];
    const uint8_t req=link.frame[3];
    uint8_t type = 0x92U, length = 8U;
    if (req == 0x13U) {
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
    } else if (req == 0x12U) {
        if (!link.session_valid || memcmp(link.session, p, 8U) != 0) {
            end_session();
            memcpy(link.session, p, 8U);
            link.last_id = 0U;
            link.session_valid = true;
        }
        memcpy(&link.response[5], p, 8U);
    } else {
        if (req==0x11U) { type=0x91U;length=20U; }
        else if (req==MOTOR_USB_PRESET_READ) {
            type=MOTOR_USB_PRESET_DATA;length=MOTOR_USB_PRESET_DATA_LEN;
        } else if (req>=MOTOR_USB_PRESET_BEGIN && req<=MOTOR_USB_PRESET_QUERY) {
            type=req==MOTOR_USB_PRESET_QUERY ? MOTOR_USB_PRESET_STATE : MOTOR_USB_PRESET_ACK;
            length=MOTOR_USB_PRESET_STATE_LEN;
        } else if (req==MOTOR_USB_TRAJ_BEGIN || req==MOTOR_USB_TRAJ_POINT) {
            type=MOTOR_USB_TRAJ_ACK;length=MOTOR_USB_TRAJ_ACK_LEN;
        } else {
            type=req==MOTOR_USB_TRAJ_START ? MOTOR_USB_TRAJ_RESULT : MOTOR_USB_MOTION_STATE;
            length=MOTOR_USB_MOTION_STATE_LEN;
        }
        session_request(req,p,now_ms,link.response+5,length);
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
                __set_PRIMASK(mask);
                end_session();
                return false;
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
                p[4]!=0 && request_length(p[3])==p[4];
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
