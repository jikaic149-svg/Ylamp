/* Offline only: production active controller + encoder, not the KEY preview.
 * Any attempted CAN/HAL access aborts. No target firmware is built or changed. */
#include <mujoco/mujoco.h>
#include "five_mit.h"
#include "five_support.h"
#include "motor_config.h"
#include <math.h>
#include <stdlib.h>

#define API __declspec(dllexport)
#define RAD 0.01745329251994329577

FDCAN_HandleTypeDef hfdcan1, hfdcan2;
uint32_t test_ipsr, test_primask, test_tick;
uint32_t HAL_GetTick(void) { abort(); }
BSP_CAN_Status_t BSP_CAN_Send(FDCAN_HandleTypeDef *bus, const BSP_CAN_Frame_t *frame)
{
    (void)bus; (void)frame; abort();
}
BSP_CAN_Status_t BSP_CAN_RegisterRxCallback(FDCAN_HandleTypeDef *bus, uint32_t id,
    BSP_CAN_RxCallback_t callback, void *context)
{
    (void)bus; (void)id; (void)callback; (void)context; abort();
}

int yi_model(const double *, const double *, double *, double *);

API void ys_info(double *out)
{
    for (unsigned i=0; i<5; ++i) {
        const DM_Motor_Config_t *motor=&g_motor_can_config[i];
        const MotorJointReference_Config_t *ref=&g_joint_reference_config[i];
        const FiveMIT_Tracking *gain=&g_five_mit_tracking[i];
        const double row[]={motor->pmax, motor->vmax, motor->tmax, ref->zero_rad,
            g_five_mit_hw[i].cad_offset_deg, gain->kp, gain->kd, gain->max_nm,
            ref->observed_lower_rad, ref->observed_upper_rad};
        for (unsigned j=0; j<10; ++j) { out[10*i+j]=row[j]; }
    }
}

/* Nearest measurement codes are an explicit assumption about the drive's
 * internal conversion. Command encoding below is the actual production C. */
static float measurement(double value, float maximum, unsigned count)
{
    double code=nearbyint((value/maximum+1)*.5*count);
    code=fmax(0, fmin(count, code));
    return ((float)code/count*2.f-1.f)*maximum;
}

/* Output rows: wire tau, requested tau, G, moving-F amplitude,
 * measured software q, measured v, counterfactual added F, measured CAD q. */
API int ys_control(const double *q, const double *v, const double *qr_deg,
    const double *vr, const double *ar, double elapsed, int quantized,
    int extra_friction, double *out)
{
    FiveMIT_State x={0};
    FiveMIT_Reference r={0};
    float cad[5], gravity[5], nominal[5], bend[5];
    for (unsigned i=0; i<5; ++i) {
        const DM_Motor_Config_t *motor=&g_motor_can_config[i];
        const float zero=g_joint_reference_config[i].zero_rad;
        double raw=q[i]-g_five_mit_hw[i].cad_offset_deg*RAD+zero;
        x.raw_rad[i]=quantized ? measurement(raw,motor->pmax,65535) : (float)raw;
        x.software_deg[i]=(x.raw_rad[i]-zero)/RAD;
        x.velocity_rad_s[i]=quantized ? measurement(v[i],motor->vmax,4095) : (float)v[i];
        cad[i]=(float)((x.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*RAD);
        r.position_deg[i]=qr_deg[i];
        r.velocity_rad_s[i]=(float)vr[i];
        r.acceleration_rad_s2[i]=(float)ar[i];
    }
    if (!FiveSupport_GravityModel(cad,(float)(x.software_deg[2]*RAD),gravity) ||
        !FiveDynamics_GravityLoad(cad,nominal,bend)) { return 0; }
    for (unsigned i=0; i<5; ++i) {
        if (fabsf(gravity[i])>g_five_mit_hw[i].model_max_nm) { return 0; }
    }
    FiveMIT_Output batch;
    /* Exact candidate ordinary Move/Ruckig branch, speed_mask selects gains only. */
    if (!FiveMIT_SampleReferenceFeedback(&r,elapsed,&x,gravity,&batch)) { return 0; }
    double model_g[5], amplitude[5];
    if (!yi_model(q,v,model_g,amplitude)) { return 0; }
    for (unsigned i=0; i<5; ++i) {
        DM_Motor_MIT_Command_t command=batch.motor[i];
        if (command.kp!=0 || command.kd!=0) { return 0; }
        float added=0;
        if (!FiveMIT_SpeedFrictionTorque(i,r.velocity_rad_s[i],nominal[i],bend[i],&added)) {
            return 0;
        }
        /* Candidate ALREADY contains ONE F. Disable only for the legacy causal
         * control, never add a second F to the candidate or select speed PI. */
        if (!extra_friction) { command.torque_ff_nm-=added; added=0; }
        if (fabsf(command.torque_ff_nm)>g_five_mit_tracking[i].max_nm) { return 0; }
        BSP_CAN_Frame_t frame;
        if (DM_Motor_EncodeMIT(&g_motor_can_config[i],&command,&frame)!=BSP_CAN_OK) {
            return 0;
        }
        const unsigned code=((unsigned)(frame.data[6]&15)<<8)|frame.data[7];
        const float wire=((float)code/4095.f*2.f-1.f)*g_motor_can_config[i].tmax;
        out[i]=quantized ? wire : command.torque_ff_nm;
        out[5+i]=command.torque_ff_nm;
        out[10+i]=gravity[i];
        out[15+i]=amplitude[i];
        out[20+i]=x.software_deg[i];
        out[25+i]=x.velocity_rad_s[i];
        out[30+i]=added;
        out[35+i]=cad[i];
    }
    return 1;
}

API int ys_setup(mjModel *m, double dt, int viscous, int stiff_friction)
{
    if (!m || m->nq!=5 || m->nv!=5 || m->nu!=15 || m->nactuator!=5) { return 0; }
    m->opt.timestep=dt;
    m->opt.disableflags|=mjDSBL_CONTACT;
    for (unsigned i=0; i<5; ++i) {
        /* Production active mode emits zero native Kp/Kd, sampled total tau. */
        m->actuator_biasprm[i*mjNBIAS+1]=0;
        m->actuator_biasprm[i*mjNBIAS+2]=0;
        if (!viscous) { m->dof_damping[i]=0; }
        m->dof_frictionloss[i]=0;
        if (stiff_friction) {
            /* Sensitivity case, not an identified breakaway/solver parameter. */
            m->dof_solimp[i*mjNIMP]=.999;
            m->dof_solimp[i*mjNIMP+1]=.999;
        }
    }
    return 1;
}

API int ys_plant(mjModel *m, mjData *d, double dry_scale, int matched_gravity)
{
    double q[5], v[5], support[5], amplitude[5];
    float position[5], nominal[5], bend[5];
    for (unsigned i=0; i<5; ++i) {
        q[i]=d->qpos[i]; v[i]=d->qvel[i]; position[i]=(float)q[i];
    }
    if (!yi_model(q,v,support,amplitude) ||
        !FiveDynamics_GravityLoad(position,nominal,bend)) { return 0; }
    for (unsigned i=0; i<5; ++i) {
        /* TOTAL moving loss used as a labelled dry proxy, never add B again.
         * Matched G isolates friction without treating accepted G as ground truth. */
        m->dof_frictionloss[i]=dry_scale*amplitude[i];
        d->qfrc_applied[i]=matched_gravity ? nominal[i]-support[i] : 0;
    }
    return 1;
}
