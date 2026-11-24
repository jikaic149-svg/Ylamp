#include "five_control.h"
#include "five_support.h"
#include "five_drag.h"
#include "five_motion.h"
#include "joint_coordinates.h"
#include "five_history.h"
#include <math.h>
#include <string.h>

#define RAD .01745329251994329577
#define J3_BIT (1U << 2)
enum { COLD_OFF, COLD_OBSERVE, COLD_J3_ACK, COLD_PRELOADED, COLD_WAIT_ACK };
static struct {
    DM_Motor_t *motor[5];
    FiveControl_Status s;
    FiveMIT_State x;
    FiveSupport_State support;
    FiveMIT_Plan plan;
    uint32_t last, query, start, duration, cold_start, baseline[5];
    unsigned cold, issued, known_enabled;
    bool init, moving, at_target;
    unsigned j3_read;
    uint32_t j3_read_at;
    bool j3_read_used;
    FiveDrag_State drag;
    bool return_drag;
} c;

static void cancel(void)
{
    c.moving=false; c.at_target=false;
    FiveSupport_Gravity(&c.support);
    FiveDrag_Stop(&c.drag);
    c.return_drag=false;
}
static void manual_idle(void)
{
    cancel();
    c.return_drag=true; /* Enter in the owner tick, after fresh observation. */
}
static void fault(unsigned why)
{
    cancel();
    c.s.fault |= why; c.cold=COLD_OFF;
}
bool FiveControl_Init(DM_Motor_t *const motors[5], uint32_t now)
{
    if (c.init || !motors || __get_IPSR()) { return false; }
    for (unsigned i=0;i<5;++i) {
        if (!DM_Motor_MatchesConfig(motors[i],&g_motor_can_config[i])) { return false; }
    }
    FiveHistory_Init(); /* Boot read only; never enables or formats foreign data. */
    memcpy(c.motor,motors,sizeof c.motor);
    c.last=now; c.query=now; c.init=true;
    return true;
}
const FiveControl_Status *FiveControl_StatusNow(void) { return c.init ? &c.s : NULL; }
/* A missed reply is not a disable acknowledgement. Reuse only a previously
 * computed GRAVITY vector, never velocity/acceleration/friction or an old move.
 * gravity_at exposes its age: this is degraded support, not a brake. */
static unsigned observe(bool check_travel)
{
    bool got[5];
    for (unsigned i=0;i<5;++i) {
        got[i]=DM_Motor_GetFeedback(c.motor[i],&c.s.feedback[i])==BSP_CAN_OK;
    }
    const uint32_t now=HAL_GetTick();
    uint32_t oldest=0, newest=UINT32_MAX;
    unsigned why=0, pose=0;
    float q[5], g[5];
    c.s.fresh=c.s.enabled=0;
    for (unsigned i=0;i<5;++i) {
        const unsigned bit=1U<<i;
        const DM_Motor_Feedback_t *f=&c.s.feedback[i];
        if (!got[i] || now-f->received_at_ms>FIVE_MAX_AGE_MS) {
            why|=FIVE_BAD_FEEDBACK; continue;
        }
        const uint32_t age=now-f->received_at_ms;
        if (age>oldest) { oldest=age; }
        if (age<newest) { newest=age; }
        c.s.fresh |= bit;
        if (f->state==1) { c.s.enabled|=bit; c.known_enabled|=bit; }
        else { c.known_enabled&=~bit; if (f->state>1) { why|=FIVE_BAD_STATE; } }
        if (f->mos_temperature_c>=FIVE_TEMP_LIMIT_C || f->rotor_temperature_c>=FIVE_TEMP_LIMIT_C) {
            why|=FIVE_HOT;
        }
        const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
        const float ref=FiveHistory_Reference(i,f);
        c.s.reference_rad[i]=ref;
        if (!isfinite(ref) || !isfinite(f->velocity_rad_s)) { why|=FIVE_BAD_FEEDBACK; continue; }
        /* Recorded travel bounds protect trajectories, not startup or support at rest. */
        const float quantization=(float)(FIVE_ENCODER_TOL_DEG*RAD);
        if (check_travel && (ref<r->observed_lower_rad-quantization ||
                            ref>r->observed_upper_rad+quantization)) { why|=FIVE_RANGE; }
        c.x.software_deg[i]=(ref-r->zero_rad)/RAD;
        c.x.raw_rad[i]=f->position_rad; c.x.velocity_rad_s[i]=f->velocity_rad_s;
        q[i]=(float)((c.x.software_deg[i]+g_five_mit_hw[i].cad_offset_deg)*RAD);
        pose |= bit;
    }
    if (pose!=FIVE_ALL || oldest-newest>FIVE_MAX_SKEW_MS) { return why|FIVE_BAD_FEEDBACK; }
    /* Travel limits reject a trajectory, not the model of static gravity. */
    if (!FiveSupport_GravityModel(q,(float)(c.x.software_deg[2]*RAD),g)) {
        return why|FIVE_BAD_FEEDBACK;
    }
    c.s.limited=0;
    for (unsigned i=0;i<5;++i) {
        float cap=g_five_mit_hw[i].model_max_nm;
        if (!isfinite(g[i])) { return why|FIVE_BAD_FEEDBACK; }
        if (fabsf(g[i])>cap) { c.s.limited|=1U<<i; why|=FIVE_LIMIT; }
        g[i]=fmaxf(-cap,fminf(cap,g[i]));
    }
    memcpy(c.s.gravity_nm,g,sizeof g);
    c.s.gravity_valid=true; c.s.gravity_at=now;
    return why;
}
static void gravity_commands(DM_Motor_MIT_Command_t command[5])
{
    for (unsigned i=0;i<5;++i) {
        command[i]=(DM_Motor_MIT_Command_t){.torque_ff_nm=c.s.gravity_nm[i]};
    }
}
static void support(unsigned mask)
{
    if (!c.s.gravity_valid) { return; }
    DM_Motor_MIT_Command_t command[5];
    float torque[5];
    unsigned limited=0;
    if (!FiveSupport_Sample(&c.support,&c.x,c.s.gravity_nm,command,&limited)) {
        fault(FIVE_BAD_FEEDBACK);
        gravity_commands(command);
    } else { c.s.limited|=limited; }
    if (c.drag.active) {
        if (!FiveDrag_Sample(&c.drag,&c.x,c.s.gravity_nm,HAL_GetTick(),torque,&limited)) {
            fault(FIVE_BAD_FEEDBACK);
            gravity_commands(command);
        } else {
            c.s.limited|=limited;
            for (unsigned i=0;i<5;++i) { command[i].torque_ff_nm=torque[i]; }
        }
    }
#ifdef YLAMP_J3_REVERSE_TEST
    for (unsigned i=0;i<5;++i) { torque[i]=command[i].torque_ff_nm; }
    FiveSupport_ApplyReverseJ3(&c.support,HAL_GetTick(),torque);
    for (unsigned i=0;i<5;++i) { command[i].torque_ff_nm=torque[i]; }
#endif
    /* A full FIFO on CAN2 must not prevent J3/J4/J5 on CAN1 from receiving G. */
    for (unsigned i=0;i<5;++i) {
        const unsigned bit=1U<<i;
        if (!(mask&bit)) { continue; }
        if (DM_Motor_SendMIT(c.motor[i],&command[i])==BSP_CAN_OK) { c.s.submitted|=bit; }
        else { fault(FIVE_TX); gravity_commands(command); }
    }
}
static bool still(void)
{
    for (unsigned i=0;i<5;++i) {
        if (fabsf(c.x.velocity_rad_s[i])>.04f) { return false; }
    }
    return true;
}
BSP_CAN_Status_t FiveControl_ReadLegacyJ3(void)
{
    if (__get_IPSR() || !c.init || c.cold || c.j3_read_used ||
        c.moving || c.at_target || c.s.gravity_valid ||
        (c.s.fault & ~FIVE_BAD_FEEDBACK)) { return BSP_CAN_NOT_READY; }
    const unsigned why=observe(false), others=FIVE_ALL & ~J3_BIT;
    if (why!=FIVE_BAD_FEEDBACK || c.s.fresh!=others || c.s.enabled!=others ||
        c.s.feedback[2].received_count || !still()) { return BSP_CAN_NOT_READY; }
    const DM_Motor_MIT_Command_t neutral={0};
    BSP_CAN_Status_t rc=DM_Motor_SendMIT(c.motor[2],&neutral);
    if (rc!=BSP_CAN_OK) { return rc; }
    c.j3_read_used=true; c.j3_read=1; c.j3_read_at=HAL_GetTick();
    return BSP_CAN_OK;
}
static void read_j3_tick(uint32_t now,unsigned why)
{
    if (!c.j3_read) { return; }
    const unsigned others=FIVE_ALL & ~J3_BIT;
    if ((c.s.fault & ~FIVE_BAD_FEEDBACK) || (why & ~FIVE_BAD_FEEDBACK) ||
        (c.s.fresh & others)!=others || (c.s.enabled & others)!=others ||
        !still() || now-c.j3_read_at>=FIVE_ENABLE_TIMEOUT_MS) {
        c.j3_read=0;
        fault(why ? why : FIVE_BAD_FEEDBACK);
        return; /* Last acquisition command is neutral; never re-enable/retry. */
    }
    if (!why && c.s.gravity_valid && c.s.fresh==FIVE_ALL && c.s.enabled==FIVE_ALL) {
        c.j3_read=0; return; /* Ordinary support() owns the handover this tick. */
    }
    BSP_CAN_Status_t rc=BSP_CAN_OK;
    if (c.j3_read==1 && now-c.j3_read_at>=FIVE_MAX_SKEW_MS) {
        rc=DM_Motor_Enable(c.motor[2]); /* Exactly once, only after explicit request. */
        if (rc==BSP_CAN_OK) { c.j3_read=2; }
    } else if (c.j3_read==2) {
        const DM_Motor_MIT_Command_t neutral={0};
        rc=DM_Motor_SendMIT(c.motor[2],&neutral); /* Legacy command-response, not 0xCC. */
    }
    if (rc!=BSP_CAN_OK) { c.j3_read=0; fault(FIVE_TX); }
}
#ifdef YLAMP_J3_REVERSE_TEST
BSP_CAN_Status_t FiveControl_ReverseJ3(void)
{
    if (__get_IPSR() || !c.init || c.cold || c.s.fault || c.moving || c.at_target ||
        c.drag.active || c.support.impedance || c.support.reverse_j3 || observe(false) ||
        c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL || !still()) { return BSP_CAN_NOT_READY; }
    return FiveSupport_EnterReverseJ3(&c.support,HAL_GetTick(),c.s.gravity_nm[2]) ?
        BSP_CAN_OK : BSP_CAN_NOT_READY;
}
#endif
BSP_CAN_Status_t FiveControl_ColdStart(void)
{
    if (__get_IPSR() || !c.init || c.cold || c.s.fault || c.known_enabled) { return BSP_CAN_NOT_READY; }
    /* No neutral can be sent merely because J3 is silent. This request is an
     * explicit declaration of unloaded support and independently known OFF.
     * Never use it to regain a missing drive during a live session. */
    unsigned issues=observe(false);
    if ((issues & ~FIVE_BAD_FEEDBACK) || c.known_enabled ||
        (c.s.fresh & ~J3_BIT)!=(FIVE_ALL & ~J3_BIT)) { return BSP_CAN_NOT_READY; }
    for (unsigned i=0;i<5;++i) {
        if ((c.s.fresh & (1U<<i)) && (c.s.feedback[i].state!=0 ||
            fabsf(c.s.feedback[i].velocity_rad_s)>.04f)) { return BSP_CAN_NOT_READY; }
    }
    const DM_Motor_MIT_Command_t neutral={0}; /* only on the explicitly OFF legacy drive */
    c.baseline[2]=c.s.feedback[2].received_count;
    if (DM_Motor_SendMIT(c.motor[2],&neutral)!=BSP_CAN_OK) { return BSP_CAN_BUSY; }
    c.issued=0; c.cold=COLD_OBSERVE; c.cold_start=HAL_GetTick(); return BSP_CAN_OK;
}
static void cold_tick(uint32_t now,unsigned why)
{
    if (now-c.cold_start>=FIVE_ENABLE_TIMEOUT_MS ||
        (why & ~FIVE_BAD_FEEDBACK)) { fault(why ? why : FIVE_BAD_STATE); return; }
    if (c.cold==COLD_WAIT_ACK) {
        /* Never repeat FC (enable). Only fresh, post-submission state=1 is an ACK. */
        unsigned ack=0;
        for (unsigned i=0;i<5;++i) {
            if ((c.s.enabled & (1U<<i)) && c.s.feedback[i].received_count!=c.baseline[i]) { ack|=1U<<i; }
        }
        if (ack==FIVE_ALL && !why) { c.cold=COLD_OFF; manual_idle(); }
        return;
    }
    if (c.known_enabled & ~c.issued) { fault(FIVE_BAD_STATE); return; }
    if (c.cold==COLD_OBSERVE && c.s.feedback[2].received_count==c.baseline[2] &&
        now-c.cold_start>=FIVE_MAX_SKEW_MS) {
        /* Some legacy drives stay silent until FC. The operator has declared
         * the arm supported and all drives OFF. Neutral was submitted first.
         * Wake J3 exactly once, leaving all four other drives OFF until its
         * measured position is available for the real gravity preload. */
        if ((c.s.fresh & ~J3_BIT)!=(FIVE_ALL & ~J3_BIT) || !still()) { return; }
        if (DM_Motor_Enable(c.motor[2])!=BSP_CAN_OK) { fault(FIVE_TX); return; }
        c.issued=J3_BIT; c.cold=COLD_J3_ACK; return;
    }
    if (c.cold==COLD_J3_ACK && !(c.s.enabled & J3_BIT)) { return; }
    if (why || !c.s.gravity_valid || !still() ||
        c.s.feedback[2].received_count==c.baseline[2]) { return; }
    if (c.cold==COLD_OBSERVE || c.cold==COLD_J3_ACK) {
        support(FIVE_ALL);
        if (c.s.submitted==FIVE_ALL) { c.cold=COLD_PRELOADED; }
        return;
    }
    for (unsigned i=0;i<5;++i) {
        if (c.issued & (1U<<i)) { continue; }
        c.baseline[i]=c.s.feedback[i].received_count;
        if (DM_Motor_Enable(c.motor[i])!=BSP_CAN_OK) { fault(FIVE_TX); return; }
        c.issued|=1U<<i;
    }
    c.cold=COLD_WAIT_ACK;
}
void FiveControl_Gravity(void)
{
    if (!__get_IPSR() && c.init) { cancel(); c.cold=COLD_OFF; c.j3_read=0; }
}
void FiveControl_Release(void)
{
    if (__get_IPSR() || !c.init) { return; }
    /* A status session must not reset a running drag filter or undo an
     * explicit pure-G selection. Only an active task needs an idle handover.
     * No observation/model computation here: USB may call with IRQs masked. */
    if (c.moving || c.at_target || c.support.impedance
#ifdef YLAMP_J3_REVERSE_TEST
        || c.support.reverse_j3
#endif
    ) { manual_idle(); }
    c.cold=COLD_OFF; c.j3_read=0;
}
BSP_CAN_Status_t FiveControl_Drag(void)
{
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { return BSP_CAN_NOT_READY; }
#endif
    if (__get_IPSR() || !c.init || c.cold || c.j3_read || c.s.fault ||
        c.moving || c.at_target || c.support.impedance ||
        (c.s.phase!=FIVE_GRAVITY && c.s.phase!=FIVE_DRAG) ||
        observe(false) || c.s.enabled!=FIVE_ALL ||
        c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
    /* Idempotent selection preserves an already-running filter/ramp. */
    if (c.drag.active) { return BSP_CAN_OK; }
    c.return_drag=false;
    return FiveDrag_Enter(&c.drag,&c.x,HAL_GetTick()) ? BSP_CAN_OK : BSP_CAN_NOT_READY;
}
BSP_CAN_Status_t FiveControl_Impedance(void)
{
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { return BSP_CAN_NOT_READY; }
#endif
    if (__get_IPSR() || !c.init || c.cold || c.j3_read || c.s.fault ||
        c.moving || c.at_target || observe(false) ||
        c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
    if (c.support.impedance) { return BSP_CAN_OK; } /* Never move a saved center. */
    if (!FiveSupport_EnterImpedance(&c.support,&c.x)) { return BSP_CAN_NOT_READY; }
    FiveDrag_Stop(&c.drag);c.return_drag=false;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveControl_ImpedanceTest(unsigned joint,float offset_deg)
{
    if (joint<1 || joint>FIVE_AXES || !isfinite(offset_deg) || !offset_deg ||
        fabsf(offset_deg)>g_five_impedance_test.max_offset_deg) { return BSP_CAN_INVALID_ARG; }
    if (__get_IPSR() || !c.init || c.cold || c.j3_read || c.s.fault ||
        c.moving || c.at_target || !c.support.impedance || c.support.test_joint ||
        observe(true) || c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) {
        return BSP_CAN_NOT_READY;
    }
    double center[5],endpoint[5];
    for (unsigned i=0;i<5;++i) {
        center[i]=c.x.software_deg[i]+(c.support.anchor_raw_rad[i]-c.x.raw_rad[i])/RAD;
    }
    FiveMIT_Plan checked;
    /* These checks validate the original travel and outward plan. The native
     * return uses a fixed center step; it is NOT a timed/velocity-bounded Move. */
    for (int sign=-1;sign<=1;sign+=2) {
        memcpy(endpoint,center,sizeof endpoint);endpoint[joint-1]+=sign*offset_deg;
        if (!FiveMIT_Prepare(&checked,center,endpoint,g_five_impedance_test.ramp_ms*.001) ||
            !FiveMIT_Prepare(&checked,endpoint,center,g_five_impedance_test.ramp_ms*.001)) {
            return BSP_CAN_INVALID_ARG;
        }
    }
    if (!FiveSupport_StartTest(&c.support,joint,offset_deg,HAL_GetTick())) {
        return BSP_CAN_INVALID_ARG;
    }
    c.s.impedance_test_joint=(uint8_t)joint;
    c.s.impedance_test_started_ms=c.support.test_started_ms;
    c.s.impedance_test_duration_ms=FiveSupport_TestDuration();
    return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveControl_KeyPress(void)
{
    if (__get_IPSR() || !c.init || c.cold || c.s.fault) { return BSP_CAN_NOT_READY; }
    if (!c.known_enabled) { return FiveControl_ColdStart(); }
    if (observe(false) || c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { manual_idle(); return BSP_CAN_OK; }
#endif
    if (c.drag.active) {
        return FiveControl_Impedance();
    }
    if (c.support.impedance || c.moving || c.at_target || !c.return_drag) {
        manual_idle();
        return BSP_CAN_OK;
    }
    return BSP_CAN_NOT_READY;
}
BSP_CAN_Status_t FiveControl_Acknowledge(void)
{
    if (__get_IPSR() || !c.init || c.cold || observe(false) || c.s.enabled!=FIVE_ALL ||
        c.s.submitted!=FIVE_ALL || !still()) { return BSP_CAN_NOT_READY; }
    c.s.fault=0; manual_idle(); return BSP_CAN_OK;
}
static BSP_CAN_Status_t move(const double target[5],uint32_t duration,uint32_t ramp,
                             unsigned joint,float velocity)
{
    if ((!target && !joint) || __get_IPSR() || duration<100U || duration>120000U ||
        (ramp && ramp>=duration*.5)) { return BSP_CAN_INVALID_ARG; }
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { return BSP_CAN_NOT_READY; }
#endif
    /* Motion/waypoint admission has no all-axis instantaneous-rest gate.
     * The user's current commissioning stage does not require every joint
     * velocity to be below the startup/legacy-handshake rest threshold. */
    if (!c.init || c.s.fault || c.moving || c.support.test_joint || c.cold || observe(true) ||
        c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
    const double seconds=duration*.001;
    double endpoint[5];
    if (joint) {
        memcpy(endpoint,c.x.software_deg,sizeof endpoint);
        endpoint[joint-1]+=(double)velocity/RAD*(seconds-ramp*.001);
        target=endpoint;
    }
    FiveMIT_Plan plan;
    if (!(ramp ? FiveMIT_PrepareCruise(&plan,c.x.software_deg,target,seconds,ramp*.001) :
                 FiveMIT_Prepare(&plan,c.x.software_deg,target,seconds))) { return BSP_CAN_INVALID_ARG; }
    FiveSupport_Gravity(&c.support);
    FiveDrag_Stop(&c.drag);
    c.return_drag=false;
    c.plan=plan; c.start=HAL_GetTick(); c.duration=duration; c.moving=true; c.at_target=false;
    return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveControl_Move(const double target[5],uint32_t duration)
{
    return move(target,duration,0,0,0);
}
BSP_CAN_Status_t FiveControl_Speed(unsigned joint,float velocity,uint32_t duration,uint32_t ramp)
{
    if (joint<1 || joint>5 || !isfinite(velocity) || velocity==0 || !ramp) {
        return BSP_CAN_INVALID_ARG;
    }
    return move(NULL,duration,ramp,joint,velocity);
}
void FiveControl_Tick(uint32_t now)
{
    if (!c.init || __get_IPSR() || now-c.last<FIVE_PERIOD_MS) { return; }
    const uint32_t gap=now-c.last;
    c.last=now; c.s.ticks++; c.s.submitted=0;
    unsigned old=c.known_enabled, why=observe(c.moving || c.at_target || c.support.test_joint);
    if (old || c.moving || c.at_target) {
        if (gap>FIVE_MAX_SKEW_MS) { why|=FIVE_TIMING; }
        if ((old & c.s.fresh & ~c.s.enabled)!=0) { why|=FIVE_BAD_STATE; }
        if (why) { fault(why); }
    }
    read_j3_tick(now,why);
    if (c.cold) { cold_tick(now,why); }
    if (c.moving || c.at_target) {
        uint32_t elapsed=now-c.start;
        if (c.at_target || elapsed>=c.duration) { elapsed=c.duration; }
        if (c.s.enabled!=FIVE_ALL) { fault(FIVE_BAD_STATE); }
        if (c.moving || c.at_target) {
            unsigned sent=0;
            BSP_CAN_Status_t result=FiveMIT_SendSample(&c.plan,elapsed*.001,&c.x,
                g_five_mit_hw,c.s.gravity_nm,c.motor,&sent);
            if (result==BSP_CAN_OK && sent==5) {
                c.s.submitted=FIVE_ALL;
                /* POSITION means timed trajectory complete, feedback hold
                 * continues. Actual error is reported, never latched as a
                 * fixed-angle/settling-time fault during commissioning. */
                if (elapsed==c.duration) { c.at_target=true; c.moving=false; }
            } else { fault(result==BSP_CAN_INVALID_ARG ? FIVE_LIMIT : FIVE_TX); }
        }
    }
    if (!c.moving && !c.at_target) {
        if (c.return_drag && !c.cold && !c.j3_read && !c.s.fault && !why &&
            c.s.gravity_valid && c.s.enabled==FIVE_ALL) {
            c.return_drag=false;
            if (!FiveDrag_Enter(&c.drag,&c.x,now)) { fault(FIVE_BAD_FEEDBACK); }
        }
        FiveSupport_TestTick(&c.support,now);
        support(c.known_enabled);
    }
    if (now-c.query>=FIVE_QUERY_MS) {
        c.query=now;
        for (unsigned i=0;i<5;++i) {
            if (i!=2 && !(c.s.submitted & (1U<<i))) { (void)DM_Motor_RequestFeedback(c.motor[i]); }
        }
    }
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { c.s.phase=FIVE_REVERSE_J3; return; }
#endif
    c.s.phase=c.s.fault ? FIVE_DEGRADED : c.moving ? FIVE_MOTION : c.at_target ? FIVE_POSITION :
        c.s.enabled==FIVE_ALL && c.s.submitted==FIVE_ALL ?
            (c.drag.active ? FIVE_DRAG : c.support.impedance ? FIVE_IMPEDANCE : FIVE_GRAVITY) : FIVE_WAIT;
    memcpy(c.s.impedance_target_raw_rad,c.support.anchor_raw_rad,sizeof c.s.impedance_target_raw_rad);
    c.s.impedance_test_joint=(uint8_t)c.support.test_joint;
    c.s.impedance_test_started_ms=c.support.test_started_ms;
    FiveHistory_Run(HAL_GetTick()); /* After motor submission; no erase/program wait. */
}
