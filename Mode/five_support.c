#include "five_support.h"
#include "motor_config.h"
#include <math.h>
#include <string.h>

bool FiveSupport_GravityModel(const float qCAD[FIVE_AXES],float j3_software_rad,
                             float gravity_nm[FIVE_AXES])
{
    if (!gravity_nm || !isfinite(j3_software_rad)) { return false; }
    const float zero[FIVE_AXES]={0};
    float next[FIVE_AXES],residual[FIVE_AXES];
    if (!FiveDynamics_Inverse(qCAD,zero,zero,next) ||
        !FiveDynamics_GravityResidual(qCAD,g_five_j2_gravity_delta,residual)) { return false; }
    next[1]+=residual[1];
    next[2]*=g_five_j3_gravity_scale;
    next[2]+=g_five_j3_gravity_cosine_nm*cosf(j3_software_rad);
    for (unsigned i=0;i<FIVE_AXES;++i) {
        if (!isfinite(next[i])) { return false; }
    }
    memcpy(gravity_nm,next,sizeof next);
    return true;
}

void FiveSupport_Gravity(FiveSupport_State *mode)
{
    if (mode) {
        mode->impedance=false;
        mode->test_joint=0;
#ifdef YLAMP_J3_REVERSE_TEST
        mode->reverse_j3=false;
#endif
    }
}
bool FiveSupport_EnterImpedance(FiveSupport_State *mode,const FiveMIT_State *state)
{
    if (!mode || !state) { return false; }
    for (unsigned i=0;i<FIVE_AXES;++i) {
        const float raw=state->raw_rad[i];
        const FiveSupport_Gains *gain=&g_five_support_gains[i];
        const float cap=g_five_impedance_max_nm[i];
        const float pd_cap=g_five_impedance_pd_max_nm[i];
        const DM_Motor_MIT_Command_t command={
            .position_rad=raw,.kp=gain->kp,.kd=gain->kd};
        BSP_CAN_Frame_t frame;
        if (!isfinite(state->software_deg[i]) || !isfinite(state->velocity_rad_s[i]) ||
            !(gain->kp>0) || !isfinite(cap) ||
            !isfinite(pd_cap) || pd_cap<=0 || pd_cap>cap ||
            cap<g_five_mit_hw[i].model_max_nm || cap>g_motor_can_config[i].tmax ||
            DM_Motor_EncodeMIT(&g_motor_can_config[i],&command,&frame)!=BSP_CAN_OK) { return false; }
    }
    FiveSupport_Gravity(mode);
    memcpy(mode->anchor_raw_rad,state->raw_rad,sizeof mode->anchor_raw_rad);
    mode->impedance=true;
    return true;
}
uint32_t FiveSupport_TestDuration(void)
{
    const FiveSupport_TestConfig *cfg=&g_five_impedance_test;
    const uint64_t total=4ULL*((uint64_t)cfg->ramp_ms+cfg->hold_ms);
    return cfg->ramp_ms>=100 && cfg->hold_ms && total<=120000 &&
        isfinite(cfg->max_offset_deg) && cfg->max_offset_deg>0 ? (uint32_t)total : 0;
}
bool FiveSupport_StartTest(FiveSupport_State *mode,unsigned joint,float offset_deg,uint32_t now)
{
    if (!mode || !mode->impedance || mode->test_joint || joint<1 || joint>FIVE_AXES ||
        !isfinite(offset_deg) || !offset_deg ||
        fabsf(offset_deg)>g_five_impedance_test.max_offset_deg || !FiveSupport_TestDuration()) {
        return false;
    }
    const float center=mode->anchor_raw_rad[joint-1];
    const float delta=(float)(offset_deg*.01745329251994329577);
    if (!isfinite(center) || fabsf(center)+fabsf(delta)>g_motor_can_config[joint-1].pmax) {
        return false;
    }
    mode->test_center_raw_rad=center;mode->test_offset_rad=delta;
    mode->test_started_ms=now;mode->test_joint=joint;
    return true;
}
void FiveSupport_TestTick(FiveSupport_State *mode,uint32_t now)
{
    if (!mode || !mode->impedance || !mode->test_joint) { return; }
    const uint32_t elapsed=now-mode->test_started_ms,total=FiveSupport_TestDuration();
    const unsigned axis=mode->test_joint-1;
    if (elapsed>=total) {
        mode->anchor_raw_rad[axis]=mode->test_center_raw_rad;
        mode->test_joint=0;return;
    }
    const uint32_t span=g_five_impedance_test.ramp_ms+g_five_impedance_test.hold_ms;
    const unsigned leg=elapsed/span;
    const double t=fmin(1.,(double)(elapsed%span)/g_five_impedance_test.ramp_ms);
    const double s=t*t*t*(10+t*(-15+6*t));
    /* Ramp only to the two deflected targets. At each return boundary restore
     * the SAME fixed center immediately, so the native spring response is
     * measured rather than hidden behind a slowly moving return reference. */
    const double offset=leg==0 ? s : leg==2 ? -s : 0;
    mode->anchor_raw_rad[axis]=(float)(mode->test_center_raw_rad+mode->test_offset_rad*offset);
}
#ifdef YLAMP_J3_REVERSE_TEST
bool FiveSupport_EnterReverseJ3(FiveSupport_State *mode,uint32_t now,float gravity_j3_nm)
{
    if (!mode || mode->impedance || mode->reverse_j3 || !isfinite(gravity_j3_nm) ||
        fabsf(gravity_j3_nm)>g_five_j3_reverse.max_nm) { return false; }
    mode->reverse_started_ms=now;
    mode->reverse_j3=true;
    return true;
}
void FiveSupport_ApplyReverseJ3(FiveSupport_State *mode,uint32_t now,float torque_nm[FIVE_AXES])
{
    if (!mode || !torque_nm || !mode->reverse_j3) { return; }
    const uint32_t elapsed=now-mode->reverse_started_ms;
    const FiveSupport_ReverseConfig *cfg=&g_five_j3_reverse;
    if (mode->impedance || elapsed>=cfg->total_ms || !isfinite(torque_nm[2]) ||
        fabsf(torque_nm[2])>cfg->max_nm) {
        mode->reverse_j3=false;
        return;
    }
    const uint32_t edge=elapsed<cfg->total_ms-elapsed ? elapsed : cfg->total_ms-elapsed;
    const float t=edge<cfg->ramp_ms ? (float)edge/cfg->ramp_ms : 1.f;
    const float blend=t*t*t*(10.f+t*(-15.f+6.f*t));
    torque_nm[2]*=1.f-2.f*blend;
}
#endif
bool FiveSupport_Sample(const FiveSupport_State *mode,const FiveMIT_State *state,
                        const float gravity_nm[FIVE_AXES],
                        DM_Motor_MIT_Command_t command[FIVE_AXES],unsigned *limited)
{
    if (!mode || !state || !gravity_nm || !command || !limited) { return false; }
    DM_Motor_MIT_Command_t next[FIVE_AXES]={{0}};
    unsigned clipped=0;
    for (unsigned i=0;i<FIVE_AXES;++i) {
        const float cap=g_five_mit_hw[i].model_max_nm, g=gravity_nm[i];
        if (!isfinite(g) || !isfinite(cap) || cap<=0 || fabsf(g)>cap) { return false; }
        next[i].torque_ff_nm=g;
        if (mode->impedance) {
            const FiveSupport_Gains *gain=&g_five_support_gains[i];
            const float hold_cap=g_five_impedance_max_nm[i];
            const float pd_cap=g_five_impedance_pd_max_nm[i];
            const float raw=state->raw_rad[i], v=state->velocity_rad_s[i];
            if (!isfinite(raw) || fabsf(raw)>g_motor_can_config[i].pmax ||
                !isfinite(v) || fabsf(v)>g_motor_can_config[i].vmax ||
                !isfinite(gain->kp) || gain->kp<=0 || gain->kp>500 ||
                !isfinite(gain->kd) || gain->kd<0 || gain->kd>5 ||
                !isfinite(hold_cap) || hold_cap<cap ||
                !isfinite(pd_cap) || pd_cap<=0 || pd_cap>hold_cap ||
                hold_cap>g_motor_can_config[i].tmax) { return false; }
            const double pd=gain->kp*((double)mode->anchor_raw_rad[i]-raw)-gain->kd*v;
            if (!isfinite(pd)) { return false; }
            const float room=fminf(pd_cap,fmaxf(0,hold_cap-fabsf(g)));
            const float scale=fabs(pd)>room ? (float)(room/fabs(pd)) : 1.f;
            if (scale<1) { clipped|=1U<<i; }
            next[i].position_rad=mode->anchor_raw_rad[i];
            next[i].kp=gain->kp*scale; next[i].kd=gain->kd*scale;
        }
        BSP_CAN_Frame_t frame;
        if (DM_Motor_EncodeMIT(&g_motor_can_config[i],&next[i],&frame)!=BSP_CAN_OK) { return false; }
    }
    memcpy(command,next,sizeof next);*limited=clipped;
    return true;
}
