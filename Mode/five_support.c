#include "five_support.h"
#include <math.h>
#include <string.h>

#define RAD .01745329251994329577

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
#ifdef YLAMP_J3_REVERSE_TEST
        mode->reverse_j3=false;
#endif
    }
}
bool FiveSupport_EnterImpedance(FiveSupport_State *mode,const FiveMIT_State *state)
{
    if (!mode || !state) { return false; }
    for (unsigned i=0;i<FIVE_AXES;++i) {
        if (!isfinite(state->software_deg[i]) || !isfinite(state->velocity_rad_s[i])) { return false; }
    }
    FiveSupport_Gravity(mode);
    memcpy(mode->anchor_deg,state->software_deg,sizeof mode->anchor_deg);
    mode->impedance=true;
    return true;
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
                        const float gravity_nm[FIVE_AXES],float torque_nm[FIVE_AXES])
{
    if (!mode || !state || !gravity_nm || !torque_nm) { return false; }
    float next[FIVE_AXES];
    for (unsigned i=0;i<FIVE_AXES;++i) {
        const float cap=g_five_mit_hw[i].model_max_nm, g=gravity_nm[i];
        if (!isfinite(g) || !isfinite(cap) || cap<=0 || fabsf(g)>cap) { return false; }
        next[i]=g;
        if (!mode->impedance) { continue; }
        const FiveSupport_Gains *gain=&g_five_support_gains[i];
        const double spring=gain->kp*(mode->anchor_deg[i]-state->software_deg[i])*RAD;
        const double pd=spring-gain->kd*state->velocity_rad_s[i];
        if (!isfinite(pd)) { return false; }
        const float room=fmaxf(0,cap-fabsf(g));
        next[i]+=fmaxf(-room,fminf(room,(float)pd));
    }
    memcpy(torque_nm,next,sizeof next);
    return true;
}
