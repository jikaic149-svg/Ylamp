#ifndef YLAMP_J25_TASK_H
#define YLAMP_J25_TASK_H
#include "five_mit.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum {
    J25_OK, J25_HANDLE, J25_NO_FEEDBACK, J25_STALE, J25_SKEW, J25_STATE,
    J25_TEMPERATURE, J25_ANGLE, J25_PLAN, J25_SAMPLE, J25_TRANSPORT, J25_TIMING
} J25_Result;
typedef struct { J25_Result result; unsigned joint, submitted; BSP_CAN_Status_t bus; } J25_Check;
/* Task context, handles J2,J3,J4,J5 only. Feedback is never commanded by
 * these reads. On failure x is unchanged. now is sampled AFTER all reads. */
J25_Check J25_Read(DM_Motor_t *const motors[4],bool require_enabled,FiveMIT_State *x);
typedef enum { J25_READY, J25_WAIT, J25_FAILED } J25_Phase;
typedef struct {
    J25_Phase phase;
    J25_Check check;
    FiveMIT_Plan plan;
    double elapsed_s;
    uint32_t started_ms, last_ms;
    unsigned queried;
    bool saw_busy;
} J25_Step;
/* Exclusive task owner, zero-initialize once, use the same four handles throughout.
 * Begin one sample; while WAIT call RetryStep on control ticks, without advancing
 * trajectory time. Only OK/READY allows the next sample (exclude waiting time).
 * submitted counts FIFO acceptance, not ACKs. FAILED stays visible until the
 * owner explicitly discards the step; clearing it cannot stop physical motion. */
J25_Check J25_SendStep(J25_Step *step,DM_Motor_t *const motors[4],const FiveMIT_Plan *plan,double elapsed_s);
J25_Check J25_RetryStep(J25_Step *step,DM_Motor_t *const motors[4]);
/* Optional board shadow: only samples and publishes results, never submits.
 * Init (before motor power) checks configured boxes; Tick does no table scans. */
void J25Shadow_Init(DM_Motor_t *const motors[4]);
void J25Shadow_Tick(uint32_t now_ms);
typedef struct {
    uint32_t sequence, tick_ms, running, samples, valid;
    J25_Check check;
    float measured_deg[4], target_raw_rad[4], ff_nm[4];
    uint32_t sequence_end;
} J25Shadow_Status;
extern volatile J25Shadow_Status g_j25_shadow_status;
/* Single SWD writer; set target[4] in degrees and duration_ms, then request=1
 * last. Do not edit until consumed (request=0). 2 cancels the preview. */
extern volatile float g_j25_target_deg[4];
extern volatile uint32_t g_j25_duration_ms,g_j25_request;
#ifdef __cplusplus
}
#endif
#endif
