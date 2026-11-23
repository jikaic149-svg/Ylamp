#include "five_mit.h"
#include "five_workspace.h"
#include "motor_config.h"
#include <math.h>
#include <string.h>

#define RAD (0.01745329251994329577)
_Static_assert(FIVE_AXES==MOTOR_CAN_JOINT_COUNT,"one MIT command per CAN joint");

/* Zero first/second endpoint derivatives; symmetric near 1 to avoid overshoot.
 * Reuse for target progress and PD entry, never for measured feedback. */
static double quintic_progress(double t)
{
    if (t<=0) { return 0; }
    if (t>=1) { return 1; }
    const double z=t<=.5 ? t : 1-t;
    const double ramp=z*z*z*(10+z*(-15+6*z));
    return t<=.5 ? ramp : 1-ramp;
}

static bool prepare_checked(FiveMIT_Plan *p,const double from[5],const double to[5],
                             double seconds,double ramp)
{
    if (!isfinite(seconds) || seconds<=0 || !isfinite(ramp) || ramp<0 ||
        (ramp>0 && ramp>=seconds*.5)) {
        p->seconds=0; return false;
    }
    FiveMIT_Plan checked={.seconds=seconds,.cruise_ramp_s=ramp};
    memcpy(checked.start_deg,from,sizeof checked.start_deg);
    memcpy(checked.end_deg,to,sizeof checked.end_deg);
    /* Ramps each travel v*ramp/2, so the cruise travels v*(total-ramp). */
    for (int i=0;i<5;++i) {
        const double speed=ramp>0 ? RAD*fabs(to[i]-from[i])/(seconds-ramp) :
                                    1.875*RAD*fabs(to[i]-from[i])/seconds;
        if (speed>g_motor_can_config[i].vmax) {
            p->seconds=0;return false;
        }
    }
    *p=checked;return true;
}
bool FiveMIT_Prepare(FiveMIT_Plan *p,const double from[5],const double to[5],double seconds)
{
    if (!p) { return false; }
    if (!FiveWorkspace_Line(from,to)) { p->seconds=0;return false; }
    return prepare_checked(p,from,to,seconds,0);
}
bool FiveMIT_PrepareCruise(FiveMIT_Plan *p,const double from[5],const double to[5],
                           double seconds,double ramp)
{
    if (!p) { return false; }
    if (!(ramp>0) || !FiveWorkspace_Line(from,to)) { p->seconds=0;return false; }
    return prepare_checked(p,from,to,seconds,ramp);
}

/* Validate a per-axis travel box once, not collision clearance. A quintic
 * between points in this box stays inside because its scalar progress is 0..1. */
bool FiveMIT_CheckBox(FiveMIT_Box *box,const double lo[5],const double hi[5])
{
    if (!box) { return false; }
    box->valid=false;
    if (!lo || !hi || !isfinite(hi[2]) || lo[2]>hi[2]) { return false; }
    double upper[5],band[2];memcpy(upper,hi,sizeof upper);upper[2]=lo[2];
    if (!FiveWorkspace_BoxJ3Band(lo,upper,band) || hi[2]>band[1]) { return false; }
    memcpy(box->lo,lo,sizeof box->lo);memcpy(box->hi,hi,sizeof box->hi);
    box->valid=true;return true;
}
bool FiveMIT_PrepareInBox(FiveMIT_Plan *p,const FiveMIT_Box *box,const double from[5],
                          const double to[5],double seconds)
{
    if (!p) { return false; }
    p->seconds=0;
    if (!box || !box->valid || !from || !to) { return false; }
    for (unsigned i=0;i<5;++i) {
        if (!(from[i]>=box->lo[i] && from[i]<=box->hi[i] &&
              to[i]>=box->lo[i] && to[i]<=box->hi[i])) { return false; }
    }
    return prepare_checked(p,from,to,seconds,0);
}

bool FiveMIT_Sample(const FiveMIT_Plan *p,double elapsed,const FiveMIT_State *x,
                    const FiveMIT_Axis profile[5],FiveMIT_Output *out)
{
    if (!p || !x || !profile || !out || !(p->seconds>0) || !isfinite(p->seconds) ||
        !isfinite(elapsed) || elapsed<0 || elapsed>p->seconds ||
        !isfinite(p->cruise_ramp_s) || p->cruise_ramp_s<0 ||
        (p->cruise_ramp_s>0 && p->cruise_ramp_s>=p->seconds*.5)) { return false; }
    const double t=elapsed/p->seconds, u=1-t;
    /* Use symmetric evaluation near the end; no overshoot at either end. */
    double s=quintic_progress(t);
    double ds=30*t*t*u*u/p->seconds, dds=60*t*u*(1-2*t)/p->seconds/p->seconds;
    if (p->cruise_ramp_s>0) {
        const double ramp=p->cruise_ramp_s, span=p->seconds-ramp;
        if (elapsed>=ramp && elapsed<=p->seconds-ramp) {
            s=(elapsed-.5*ramp)/span;ds=1/span;dds=0;
        } else {
            const bool ending=elapsed>ramp;
            const double z=(ending ? p->seconds-elapsed : elapsed)/ramp, w=1-z;
            /* Integral of quintic_progress(z); mirror near the final endpoint. */
            const double distance=ramp/span*z*z*z*z*(2.5+z*(-3+z));
            s=ending ? 1-distance : distance;
            ds=quintic_progress(z)/span;
            dds=(ending ? -1 : 1)*30*z*z*w*w/(ramp*span);
        }
    }
    float q[5],acc[5],tau[5];
    FiveMIT_Output next={0};
    for (int i=0;i<5;++i) {
        const FiveMIT_Axis *cfg=&profile[i];
        if (!isfinite(x->software_deg[i]) || !isfinite(x->raw_rad[i]) ||
            fabsf(x->raw_rad[i])>g_motor_can_config[i].pmax ||
            !isfinite(x->velocity_rad_s[i]) || fabsf(x->velocity_rad_s[i])>g_motor_can_config[i].vmax ||
            !isfinite(cfg->coulomb_nm) || cfg->coulomb_nm<0 ||
            !isfinite(cfg->viscous_nm_s_rad) || cfg->viscous_nm_s_rad<0 ||
            !isfinite(cfg->smooth_rad_s) || cfg->smooth_rad_s<=0 ||
            !isfinite(cfg->model_max_nm) || cfg->model_max_nm<=0) { return false; }
        const double delta=p->end_deg[i]-p->start_deg[i];
        const double goal=t==1?p->end_deg[i]:p->start_deg[i]+s*delta;
        q[i]=(float)((x->software_deg[i]+cfg->cad_offset_deg)*RAD);
        acc[i]=(float)(RAD*delta*dds);
        next.motor[i]=(DM_Motor_MIT_Command_t){
            .position_rad=(float)(x->raw_rad[i]+(goal-x->software_deg[i])*RAD),
            .velocity_rad_s=(float)(RAD*delta*ds), .kp=cfg->kp, .kd=cfg->kd};
    }
    if (!FiveDynamics_Inverse(q,x->velocity_rad_s,acc,tau)) { return false; }
    for (int i=0;i<5;++i) {
        const FiveMIT_Axis *cfg=&profile[i]; const float v=x->velocity_rad_s[i];
        DM_Motor_MIT_Command_t *c=&next.motor[i];
        c->torque_ff_nm=tau[i]+cfg->viscous_nm_s_rad*v+
            (cfg->coulomb_nm!=0 ? cfg->coulomb_nm*tanhf(v/cfg->smooth_rad_s) : 0);
        const float predicted=c->kp*(c->position_rad-x->raw_rad[i])+c->kd*(c->velocity_rad_s-v)+c->torque_ff_nm;
        if (!isfinite(predicted) || fabsf(predicted)>cfg->model_max_nm ||
            DM_Motor_EncodeMIT(&g_motor_can_config[i],c,&next.frame[i])!=BSP_CAN_OK) { return false; }
    }
    *out=next;return true;
}

bool FiveMIT_SpeedFrictionTorque(unsigned axis,float reference,float nominal_g,
                                 float bending,float *out)
{
    if (!out || axis>=FIVE_AXES || !isfinite(reference) || !isfinite(nominal_g) ||
        !isfinite(bending) || bending<0) { return false; }
    const FiveMIT_SpeedFriction *cfg=&g_five_speed_friction[axis];
    const float lo=g_five_speed_friction_range_rad_s[0];
    const float hi=g_five_speed_friction_range_rad_s[1];
    const float k16=g_five_speed_friction_knots_rad_s[0];
    const float k48=g_five_speed_friction_knots_rad_s[1];
    const float smooth=g_five_mit_hw[axis].smooth_rad_s;
    if (!isfinite(cfg->log_amplitude_nm) ||
        !isfinite(cfg->speed_exponent) || !isfinite(lo) || !(lo>0) ||
        !isfinite(hi) || hi<lo || !isfinite(smooth) || !(smooth>0) ||
        !isfinite(cfg->hinge_016) || !isfinite(cfg->hinge_048) ||
        !isfinite(k16) || !isfinite(k48) ||
        !(lo<k16 && k16<k48 && k48<hi)) { return false; }
    const float speed=fminf(hi,fmaxf(lo,fabsf(reference)));
    float log_amplitude=cfg->log_amplitude_nm+
        cfg->speed_exponent*logf(speed/lo)+
        cfg->hinge_016*fmaxf(0,logf(speed/k16))+
        cfg->hinge_048*fmaxf(0,logf(speed/k48));
    if (axis==1) {
        const float *load=g_five_speed_friction_j2;
        if (!isfinite(load[0]) || !isfinite(load[1])) { return false; }
        log_amplitude+=load[0]*bending+load[1]*fabsf(nominal_g);
    }
    const float torque=expf(log_amplitude)*tanhf(reference/smooth);
    if (!isfinite(log_amplitude) || !isfinite(torque)) { return false; }
    *out=torque;return true;
}

bool FiveMIT_SampleFeedback(const FiveMIT_Plan *p,double elapsed,const FiveMIT_State *x,
                            const float support_g[5],FiveMIT_Output *out)
{
    if (!support_g || !out || !isfinite(g_five_tracking_ramp_s) ||
        g_five_tracking_ramp_s<0) { return false; }
    FiveMIT_Output next;
    FiveMIT_Axis active[5];memcpy(active,g_five_mit_hw,sizeof active);
    for (unsigned i=0;i<5;++i) { active[i].model_max_nm=g_five_mit_tracking[i].max_nm; }
    if (!FiveMIT_Sample(p,elapsed,x,active,&next)) { return false; }
    float q[5],nominal_g[5],bending[5];
    for (unsigned i=0;i<5;++i) {
        q[i]=(float)((x->software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*RAD);
    }
    if (!FiveDynamics_GravityLoad(q,nominal_g,bending)) { return false; }
    const float blend=g_five_tracking_ramp_s>0 ?
        (float)quintic_progress(elapsed/g_five_tracking_ramp_s) : 1.f;
    for (unsigned i=0;i<5;++i) {
        DM_Motor_MIT_Command_t *cmd=&next.motor[i];
        const FiveMIT_Tracking *gain=&g_five_mit_tracking[i];
        const bool speed_axis=p->cruise_ramp_s>0 && p->end_deg[i]!=p->start_deg[i];
        const float kp=speed_axis ? g_five_speed_pi[i].ki : gain->kp;
        const float kd=speed_axis ? g_five_speed_pi[i].kp : gain->kd;
        float friction=0;
        if (speed_axis && !FiveMIT_SpeedFrictionTorque(i,cmd->velocity_rad_s,
                nominal_g[i],bending[i],&friction)) { return false; }
        const float error=cmd->position_rad-x->raw_rad[i];
        const float feedback=kp*error+kd*(cmd->velocity_rad_s-x->velocity_rad_s[i]);
        cmd->torque_ff_nm+=support_g[i]-nominal_g[i]+blend*feedback+friction;
        /* Keep formal G and actuator-side gains unchanged. */
        cmd->kp=cmd->kd=0;
        if (!isfinite(kp) || kp<0 || !isfinite(kd) || kd<0 ||
            !isfinite(support_g[i]) || !isfinite(cmd->torque_ff_nm) ||
            fabsf(cmd->torque_ff_nm)>gain->max_nm ||
            DM_Motor_EncodeMIT(&g_motor_can_config[i],cmd,&next.frame[i])!=BSP_CAN_OK) { return false; }
    }
    *out=next;return true;
}
