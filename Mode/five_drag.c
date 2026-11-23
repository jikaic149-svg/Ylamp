#include "five_drag.h"
#include "motor_config.h"
#include "joint_coordinates.h"
#include <math.h>
#include <string.h>

#define RAD .01745329251994329577

static bool valid(const FiveMIT_State *state)
{
    if (!state || !isfinite(g_five_drag_filter_s) || g_five_drag_filter_s<=0 ||
        !isfinite(g_five_drag_ramp_s) || g_five_drag_ramp_s<=0) { return false; }
    for (unsigned i=0;i<FIVE_AXES;++i) {
        const FiveDrag_Axis *a=&g_five_drag[i];
        if (!isfinite(state->software_deg[i]) || !isfinite(state->raw_rad[i]) ||
            !isfinite(state->velocity_rad_s[i]) || !isfinite(a->fraction) ||
            a->fraction<0 || a->fraction>=1 || !isfinite(a->damping_nm_s_rad) ||
            a->damping_nm_s_rad<0 || !isfinite(a->deadband_rad_s) ||
            a->deadband_rad_s<=0 || !isfinite(a->assist_max_nm) ||
            a->assist_max_nm<0 || !isfinite(a->total_max_nm) ||
            a->total_max_nm<g_five_mit_hw[i].model_max_nm ||
            a->total_max_nm>g_five_mit_tracking[i].max_nm) { return false; }
    }
    return true;
}

void FiveDrag_Stop(FiveDrag_State *mode)
{
    if (mode) { memset(mode,0,sizeof *mode); }
}
bool FiveDrag_Enter(FiveDrag_State *mode,const FiveMIT_State *state,uint32_t now)
{
    if (!mode || mode->active || !valid(state)) { return false; }
    /* Start the velocity filter at zero and ramp assistance, not gravity. */
    *mode=(FiveDrag_State){.active=true,.entered_ms=now,.sampled_ms=now};
    return true;
}
bool FiveDrag_Sample(FiveDrag_State *mode,const FiveMIT_State *state,
                     const float gravity_nm[FIVE_AXES],uint32_t now,
                     float torque_nm[FIVE_AXES],unsigned *limited)
{
    if (!mode || !mode->active || !gravity_nm || !torque_nm || !limited ||
        !valid(state)) { return false; }
    FiveDrag_State next=*mode;
    float q[FIVE_AXES],nominal_g[FIVE_AXES],bending[FIVE_AXES],torque[FIVE_AXES];
    for (unsigned i=0;i<FIVE_AXES;++i) {
        if (!isfinite(gravity_nm[i]) ||
            fabsf(gravity_nm[i])>g_five_mit_hw[i].model_max_nm ||
            !isfinite(next.velocity_rad_s[i])) { return false; }
        q[i]=(float)((state->software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*RAD);
    }
    if (!FiveDynamics_GravityLoad(q,nominal_g,bending)) { return false; }
    const float dt=(uint32_t)(now-next.sampled_ms)*.001f;
    const float filter=dt/(g_five_drag_filter_s+dt);
    const float t=fminf(1.f,(uint32_t)(now-next.entered_ms)*.001f/g_five_drag_ramp_s);
    const float blend=t*t*t*(10.f+t*(-15.f+6.f*t));
    unsigned clipped=0;
    for (unsigned i=0;i<FIVE_AXES;++i) {
        const FiveDrag_Axis *a=&g_five_drag[i];
        if (a->fraction==0 && a->damping_nm_s_rad==0) {
            torque[i]=gravity_nm[i];continue; /* User keeps J3/J4/J5 exactly G. */
        }
        const float raw=state->velocity_rad_s[i];
        next.velocity_rad_s[i]+=filter*(raw-next.velocity_rad_s[i]);
        if (!isfinite(next.velocity_rad_s[i])) { return false; }
        /* A stopped or reversed measured axis must not receive assistance in
         * the filter's old direction. This is signal conditioning, not an
         * all-axis rest gate or a restriction on a commanded move. */
        float speed=0;
        if (raw*next.velocity_rad_s[i]>0) {
            const float magnitude=fminf(fabsf(raw),fabsf(next.velocity_rad_s[i]));
            speed=copysignf(fmaxf(0,magnitude-a->deadband_rad_s),raw);
        }
        /* No active targets are sent to J3-J5. Their static position is not
         * a drag-entry gate. At an assisted axis's original travel end,
         * suppress only outward help; retain inward help and gravity. */
        const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
        const float ref=(float)(state->software_deg[i]*RAD)+r->zero_rad;
        if (!isfinite(ref)) { return false; }
        if ((ref<=r->observed_lower_rad && speed<0) ||
            (ref>=r->observed_upper_rad && speed>0)) { speed=0; }
        float friction=0;
        if (!FiveMIT_SpeedFrictionTorque(i,speed,nominal_g[i],bending[i],&friction)) {
            return false;
        }
        const float assist=fmaxf(-a->assist_max_nm,
            fminf(a->assist_max_nm,a->fraction*friction));
        /* Damping follows the actual measurement, not a delayed filter sign. */
        const float damping=copysignf(fmaxf(0,fabsf(raw)-a->deadband_rad_s),raw);
        const float overlay=blend*(assist-a->damping_nm_s_rad*damping);
        const float room=fmaxf(0,a->total_max_nm-fabsf(gravity_nm[i]));
        if (!isfinite(overlay)) { return false; }
        const float bounded=fmaxf(-room,fminf(room,overlay));
        if (bounded!=overlay) { clipped|=1U<<i; }
        torque[i]=gravity_nm[i]+bounded;
        if (!isfinite(torque[i])) { return false; }
    }
    next.sampled_ms=now;
    *mode=next;*limited=clipped;memcpy(torque_nm,torque,sizeof torque);
    return true;
}
