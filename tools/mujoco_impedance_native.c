/* Host-only MuJoCo 4.0.1 access and model evaluator. No hardware or CAN calls.
 * Compile against the installed headers and the original five_dynamics.c /
 * five_mit.c CONFIG tables, never the target firmware application.
 */
#include <mujoco/mujoco.h>
#include "five_dynamics.h"
#include "five_support.h"
#include "five_coordinates.h"
#include <math.h>
#include <string.h>

#define API __declspec(dllexport)

API int yi_layout(const mjModel *m)
{
    return m && m->nq==5 && m->nv==5 && m->nu==15 && m->nactuator==5;
}

API void yi_coefficients(const mjModel *m,int axis,double *out)
{
    memcpy(out,m->actuator_gainprm+axis*mjNGAIN,10*sizeof(double));
    memcpy(out+10,m->actuator_biasprm+axis*mjNBIAS,10*sizeof(double));
}

API int yi_gains(mjModel *m,const double *kp,const double *kd)
{
    if (!yi_layout(m)) return 0;
    for (int i=0;i<5;++i) {
        if (m->actuator_gaintype[i]!=mjGAIN_PID ||
            !isfinite(kp[i]) || kp[i]<0 || kp[i]>500 ||
            !isfinite(kd[i]) || kd[i]<0 || kd[i]>5) return 0;
    }
    for (int i=0;i<5;++i) {
        /* Native PID stores proportional/derivative coefficients in affine
         * bias; gainprm[0] is the integral gain. Verified on the compiled XML. */
        m->actuator_biasprm[i*mjNBIAS+1]=-kp[i];
        m->actuator_biasprm[i*mjNBIAS+2]=-kd[i];
    }
    return 1;
}

API void yi_environment(mjModel *m,double dt,int contacts,
                        const double *damping,const double *friction)
{
    m->opt.timestep=dt;
    if (contacts) m->opt.disableflags&=~mjDSBL_CONTACT;
    else m->opt.disableflags|=mjDSBL_CONTACT;
    for (int i=0;i<5;++i) {
        m->dof_damping[i]=damping[i];
        m->dof_frictionloss[i]=friction[i];
    }
}

API void yi_friction(mjModel *m,const double *friction)
{
    for (int i=0;i<5;++i) m->dof_frictionloss[i]=friction[i];
}

API void yi_read(const mjModel *m,const mjData *d,double *out)
{
    for (int i=0;i<5;++i) {
        out[i]=d->qpos[i];
        out[5+i]=d->qvel[i];
        out[10+i]=d->qfrc_actuator[i];
    }
    out[15]=d->time;
    out[16]=d->ncon;
    int warnings=0;
    for (int i=0;i<mjNWARNING;++i) warnings+=d->warning[i].number;
    out[17]=warnings;
    (void)m;
}

/* Same scoped G formula as FiveSupport_GravityModel, using its actual tables
 * and the original C rigid-body evaluator. Checked against Python/native.
 * F returns the moving-direction AMPLITUDE, not an identified stiction level.
 */
API int yi_model(const double *q,const double *v,double *gravity,double *amplitude)
{
    float position[5],g[5],bending[5],residual[5];
    for (int i=0;i<5;++i) position[i]=(float)q[i];
    if (!FiveDynamics_GravityLoad(position,g,bending) ||
        !FiveDynamics_GravityResidual(position,g_five_j2_gravity_delta,residual)) return 0;
    for (int i=0;i<5;++i) {
        const FiveMIT_SpeedFriction *f=&g_five_speed_friction[i];
        const float speed=fminf(g_five_speed_friction_range_rad_s[1],
            fmaxf(g_five_speed_friction_range_rad_s[0],fabsf((float)v[i])));
        float loga=f->log_amplitude_nm+
            f->speed_exponent*logf(speed/g_five_speed_friction_range_rad_s[0])+
            f->hinge_016*fmaxf(0,logf(speed/g_five_speed_friction_knots_rad_s[0]))+
            f->hinge_048*fmaxf(0,logf(speed/g_five_speed_friction_knots_rad_s[1]));
        if (i==1) loga+=g_five_speed_friction_j2[0]*bending[i]+
            g_five_speed_friction_j2[1]*fabsf(g[i]);
        amplitude[i]=expf(loga);
        float result=g[i];
        if (i==1) result+=residual[i];
        if (i==2) result=result*g_five_j3_gravity_scale+
            g_five_j3_gravity_cosine_nm*cosf(position[i]-(float)(J3_CAD*0.01745329251994329577));
        gravity[i]=fmaxf(-g_five_mit_hw[i].model_max_nm,
                         fminf(g_five_mit_hw[i].model_max_nm,result));
        if (!isfinite(gravity[i]) || !isfinite(amplitude[i])) return 0;
    }
    return 1;
}
