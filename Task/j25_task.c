#include "j25_task.h"
#include "joint_coordinates.h"
#include "motor_config.h"
#include <math.h>

static J25_Check check(J25_Result r,unsigned joint)
{
    return (J25_Check){.result=r,.joint=joint,.bus=BSP_CAN_NOT_READY};
}

static J25_Check read_axes(DM_Motor_t *const motors[4],bool enabled,FiveMIT_State *x,unsigned *refresh)
{
    *refresh=0;
    if (!motors || !x || __get_IPSR()!=0U) { return check(J25_HANDLE,0); }
    DM_Motor_Feedback_t f[4];
    J25_Check c=check(J25_OK,0);
    for (unsigned j=0;j<4;++j) {
        if (!DM_Motor_MatchesConfig(motors[j],&g_motor_can_config[j+1])) {
            return check(J25_HANDLE,j+2);
        }
        if (DM_Motor_GetFeedback(motors[j],&f[j])!=BSP_CAN_OK) {
            *refresh|=1U<<j;
            if (c.result==J25_OK) { c=check(J25_NO_FEEDBACK,j+2); }
        }
    }
    const uint32_t now=HAL_GetTick();
    uint32_t newest=UINT32_MAX;
    FiveMIT_State next={0}; /* Fixed base, not a measured or commanded J1. */
    for (unsigned j=0;j<4;++j) {
        const unsigned i=j+1,axis=j+2;
        if (*refresh&(1U<<j)) { continue; }
        const uint32_t age=now-f[j].received_at_ms;
        if (age>g_motor_config.feedback_max_age_ms) {
            *refresh|=1U<<j;
            if (c.result==J25_OK) { c=check(J25_STALE,axis); }
            continue;
        }
        if (age<newest) { newest=age; }
        if (f[j].state>1 || (enabled && f[j].state!=1)) { return check(J25_STATE,axis); }
        /* Same existing bench temperature gate as the legacy J3 stream. */
        if (f[j].mos_temperature_c>=g_motor_config.joint[0].temperature_c ||
            f[j].rotor_temperature_c>=g_motor_config.joint[0].temperature_c) {
            return check(J25_TEMPERATURE,axis);
        }
        const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
        const float raw=f[j].position_rad;
        const float ref=JointCoordinates_Reference(raw,r->observed_lower_rad,r->observed_upper_rad);
        if (!r->kind || !(ref>=r->observed_lower_rad && ref<=r->observed_upper_rad) ||
            !isfinite(f[j].velocity_rad_s)) { return check(J25_ANGLE,axis); }
        next.software_deg[i]=JointCoordinates_Angle(ref,r->zero_rad)*(180.0/3.14159265358979323846);
        next.raw_rad[i]=raw;next.velocity_rad_s[i]=f[j].velocity_rad_s;
    }
    for (unsigned j=0;j<4;++j) {
        if (!(*refresh&(1U<<j)) && now-f[j].received_at_ms-newest>g_motor_config.max_step_ms) {
            *refresh|=1U<<j;
            if (c.result==J25_OK) { c=check(J25_SKEW,j+2); }
        }
    }
    if (c.result!=J25_OK) { return c; }
    *x=next;return check(J25_OK,0);
}

J25_Check J25_Read(DM_Motor_t *const motors[4],bool enabled,FiveMIT_State *x)
{
    unsigned refresh;
    return read_axes(motors,enabled,x,&refresh);
}

static bool needs_feedback(J25_Result r)
{
    return r==J25_NO_FEEDBACK || r==J25_STALE || r==J25_SKEW;
}

static J25_Check submit_step(J25_Step *s,DM_Motor_t *const motors[4],uint32_t now)
{
    unsigned refresh;
    FiveMIT_State x;
    J25_Check c=read_axes(motors,true,&x,&refresh);
    c.submitted=s->check.submitted;
    s->last_ms=now;
    s->phase=J25_FAILED;
    if (needs_feedback(c.result)) {
        if (now-s->started_ms<g_motor_config.max_step_ms) {
            s->phase=J25_WAIT;
            for (unsigned j=0;j<4;++j) {
                const unsigned bit=1U<<j;
                if ((refresh&bit) && !(s->queried&bit) &&
                    g_motor_can_config[j+1].protocol!=DM_MOTOR_PROTOCOL_LEGACY_MIT) {
                    s->queried|=bit;
                    BSP_CAN_Status_t bus=DM_Motor_RequestFeedback(motors[j]);
                    if (c.joint==j+2) { c.bus=bus; }
                }
            }
        }
        return s->check=c;
    }
    if (c.result!=J25_OK) { return s->check=c; }
    FiveMIT_Output out;
    if (!FiveMIT_SampleJ25(&s->plan,s->elapsed_s,&x,&out)) {
        c.result=J25_SAMPLE;return s->check=c;
    }
    for (unsigned j=c.submitted;j<4;++j) {
        c.bus=DM_Motor_SendMIT(motors[j],&out.motor[j+1]);
        if (c.bus!=BSP_CAN_OK) {
            c.result=J25_TRANSPORT;c.joint=j+2;
            if (c.bus==BSP_CAN_BUSY && !s->saw_busy && now-s->started_ms<g_motor_config.max_step_ms) {
                s->saw_busy=true;s->phase=J25_WAIT;
            }
            return s->check=c;
        }
        ++c.submitted;
    }
    s->phase=J25_READY;
    return s->check=c;
}

J25_Check J25_SendStep(J25_Step *s,DM_Motor_t *const motors[4],const FiveMIT_Plan *p,double t)
{
    if (!s || __get_IPSR()!=0U) { return check(J25_HANDLE,0); }
    if (s->phase!=J25_READY) { return s->check; }
    *s=(J25_Step){.phase=J25_FAILED};
    if (!p || !(p->seconds>0) || !isfinite(p->seconds) || !isfinite(t) || t<0 || t>p->seconds) {
        return s->check=check(J25_PLAN,0);
    }
    s->plan=*p;s->elapsed_s=t;s->started_ms=HAL_GetTick();
    return submit_step(s,motors,s->started_ms);
}

J25_Check J25_RetryStep(J25_Step *s,DM_Motor_t *const motors[4])
{
    if (!s || __get_IPSR()!=0U) { return check(J25_HANDLE,0); }
    if (s->phase!=J25_WAIT) { return s->check; }
    const uint32_t now=HAL_GetTick();
    if (now-s->started_ms>g_motor_config.max_step_ms) {
        s->phase=J25_FAILED;return s->check;
    }
    if (now-s->last_ms<g_motor_config.control_period_ms) { return s->check; }
    return submit_step(s,motors,now);
}

volatile J25Shadow_Status g_j25_shadow_status;
volatile float g_j25_target_deg[4];
volatile uint32_t g_j25_duration_ms,g_j25_request;
static DM_Motor_t *handles[4];
static FiveMIT_Box boxes[3];
static FiveMIT_Plan plan;
static J25Shadow_Status status;
static uint32_t last_ms,start_ms,move_ms,wait_ms;
static bool waiting_feedback;

void J25Shadow_Init(DM_Motor_t *const motors[4])
{
    for (unsigned j=0;j<4;++j) { handles[j]=motors[j]; }
    for (unsigned k=0;k<3;++k) {
        (void)FiveMIT_CheckBox(&boxes[k],g_j25_preview_boxes[k][0],g_j25_preview_boxes[k][1]);
    }
    plan.seconds=0;status=(J25Shadow_Status){0};g_j25_shadow_status=status;
    g_j25_request=0;last_ms=HAL_GetTick();waiting_feedback=false;
}
void J25Shadow_Tick(uint32_t now)
{
    const uint32_t gap=now-last_ms;
    if (gap<g_motor_config.control_period_ms) { return; }
    last_ms=now;
    uint32_t req=g_j25_request;
    if (!req && !status.running) { return; }
    status.valid=0;
    if (req==2U) {
        status.running=0;status.check=check(J25_OK,0);
    } else if ((req && (req!=1U || status.running)) || (!req && (gap>g_motor_config.max_step_ms ||
            (waiting_feedback && now-wait_ms>g_motor_config.max_step_ms)))) {
        status.running=0;status.check=check(req?J25_PLAN:J25_TIMING,0);
    } else {
        FiveMIT_State x;
        status.check=J25_Read(handles,false,&x);
        if (status.running && needs_feedback(status.check.result)) {
            /* App keeps its normal V13 read-only queries running during the wait. */
            if (!waiting_feedback) { waiting_feedback=true;wait_ms=now; }
            if (now-wait_ms>=g_motor_config.max_step_ms) { status.running=0; }
        } else if (waiting_feedback && status.check.result==J25_OK) {
            start_ms+=now-wait_ms;
            waiting_feedback=false;
        }
        if (status.check.result==J25_OK && req==1U) {
            double to[5]={0};for (unsigned j=0;j<4;++j) { to[j+1]=g_j25_target_deg[j]; }
            move_ms=g_j25_duration_ms;start_ms=now;status.samples=0;status.running=0;
            for (unsigned k=0;k<3;++k) {
                if (move_ms<=INT32_MAX && FiveMIT_PrepareInBox(&plan,&boxes[k],x.software_deg,to,move_ms*.001)) {
                    status.running=1;break;
                }
            }
            if (!status.running) { status.check=check(J25_PLAN,0); }
        }
        if (status.check.result==J25_OK && status.running) {
            const uint32_t elapsed=now-start_ms;
            FiveMIT_Output out;
            if (!FiveMIT_SampleJ25(&plan,elapsed>=move_ms?plan.seconds:elapsed*.001,&x,&out)) {
                status.check=check(J25_SAMPLE,0);
            } else {
                for (unsigned j=0;j<4;++j) {
                    status.measured_deg[j]=(float)x.software_deg[j+1];
                    status.target_raw_rad[j]=out.motor[j+1].position_rad;
                    status.ff_nm[j]=out.motor[j+1].torque_ff_nm;
                }
                status.valid=1;++status.samples;
                if (elapsed>=move_ms) { status.running=0; }
            }
        }
        if (status.check.result!=J25_OK && !needs_feedback(status.check.result)) { status.running=0; }
    }
    if (!status.running) { waiting_feedback=false; }
    status.tick_ms=now;
    const uint32_t even=status.sequence_end+2U;
    g_j25_shadow_status.sequence=even-1U;status.sequence=even-1U;
    status.sequence_end=even;g_j25_shadow_status=status;g_j25_shadow_status.sequence=even;
    if (req) { g_j25_request=0; }
}
