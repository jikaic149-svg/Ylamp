#include "five_control.h"
#include "five_support.h"
#include "five_motion.h"
#include "joint_coordinates.h"
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
} c;

static void cancel(void)
{
    c.moving=false; c.at_target=false;
    FiveSupport_Gravity(&c.support);
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
        const float ref=JointCoordinates_Reference(f->position_rad,r->observed_lower_rad,r->observed_upper_rad);
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
static void support(unsigned mask)
{
    if (!c.s.gravity_valid) { return; }
    float torque[5];
    if (!FiveSupport_Sample(&c.support,&c.x,c.s.gravity_nm,torque)) {
        fault(FIVE_BAD_FEEDBACK);
        memcpy(torque,c.s.gravity_nm,sizeof torque);
    }
#ifdef YLAMP_J3_REVERSE_TEST
    FiveSupport_ApplyReverseJ3(&c.support,HAL_GetTick(),torque);
#endif
    /* A full FIFO on CAN2 must not prevent J3/J4/J5 on CAN1 from receiving G. */
    for (unsigned i=0;i<5;++i) {
        const unsigned bit=1U<<i;
        if (!(mask&bit)) { continue; }
        if (DM_Motor_SendTorque(c.motor[i],torque[i])==BSP_CAN_OK) { c.s.submitted|=bit; }
        else { fault(FIVE_TX); memcpy(torque,c.s.gravity_nm,sizeof torque); }
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
        c.support.impedance || c.support.reverse_j3 || observe(false) ||
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
        if (ack==FIVE_ALL && !why) { c.cold=COLD_OFF; }
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
BSP_CAN_Status_t FiveControl_KeyPress(void)
{
    if (__get_IPSR() || !c.init || c.cold || c.s.fault) { return BSP_CAN_NOT_READY; }
    if (!c.known_enabled) { return FiveControl_ColdStart(); }
    if (observe(false) || c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { FiveControl_Gravity(); return BSP_CAN_OK; }
#endif
    if (c.support.impedance || c.moving || c.at_target) {
        FiveControl_Gravity();
        return BSP_CAN_OK;
    }
    return FiveSupport_EnterImpedance(&c.support,&c.x) ? BSP_CAN_OK : BSP_CAN_NOT_READY;
}
BSP_CAN_Status_t FiveControl_Acknowledge(void)
{
    if (__get_IPSR() || !c.init || c.cold || observe(false) || c.s.enabled!=FIVE_ALL ||
        c.s.submitted!=FIVE_ALL || !still()) { return BSP_CAN_NOT_READY; }
    c.s.fault=0; cancel(); return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveControl_Move(const double target[5],uint32_t duration)
{
    if (!target || __get_IPSR() || duration<100U || duration>120000U) { return BSP_CAN_INVALID_ARG; }
#ifdef YLAMP_J3_REVERSE_TEST
    if (c.support.reverse_j3) { return BSP_CAN_NOT_READY; }
#endif
    /* Motion/waypoint admission has no all-axis instantaneous-rest gate.
     * The user's current commissioning stage does not require every joint
     * velocity to be below the startup/legacy-handshake rest threshold. */
    if (!c.init || c.s.fault || c.moving || c.cold || observe(true) ||
        c.s.enabled!=FIVE_ALL || c.s.submitted!=FIVE_ALL) { return BSP_CAN_NOT_READY; }
    const double seconds=duration*.001;
    FiveMIT_Plan plan;
    if (!FiveMIT_Prepare(&plan,c.x.software_deg,target,seconds)) { return BSP_CAN_INVALID_ARG; }
    FiveSupport_Gravity(&c.support);
    c.plan=plan; c.start=HAL_GetTick(); c.duration=duration; c.moving=true; c.at_target=false;
    return BSP_CAN_OK;
}
void FiveControl_Tick(uint32_t now)
{
    if (!c.init || __get_IPSR() || now-c.last<FIVE_PERIOD_MS) { return; }
    const uint32_t gap=now-c.last;
    c.last=now; c.s.ticks++; c.s.submitted=0;
    unsigned old=c.known_enabled, why=observe(c.moving || c.at_target);
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
    if (!c.moving && !c.at_target) { support(c.known_enabled); }
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
            (c.support.impedance ? FIVE_IMPEDANCE : FIVE_GRAVITY) : FIVE_WAIT;
}
