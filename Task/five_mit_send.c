#include "five_mit.h"
#include "motor_config.h"

static bool valid_motors(DM_Motor_t *const motors[5])
{
    if (!motors || __get_IPSR()!=0U) { return false; }
    for (int i=0;i<5;++i) {
        if (!DM_Motor_MatchesConfig(motors[i],&g_motor_can_config[i])) { return false; }
    }
    return true;
}
static BSP_CAN_Status_t send_batch(const FiveMIT_Output *batch,
    DM_Motor_t *const motors[5],unsigned *submitted)
{
    for (int i=0;i<5;++i) {
        BSP_CAN_Status_t status=DM_Motor_SendMIT(motors[i],&batch->motor[i]);
        if (status!=BSP_CAN_OK) { return status; }
        ++*submitted;
    }
    return BSP_CAN_OK;
}
BSP_CAN_Status_t FiveMIT_SendSample(const FiveMIT_Plan *p,double t,
    const FiveMIT_State *x,const FiveMIT_Axis profile[5],
    const float *support_g,DM_Motor_t *const motors[5],unsigned *submitted)
{
    if (!submitted) { return BSP_CAN_INVALID_ARG; }
    *submitted=0;
    if (!valid_motors(motors)) { return BSP_CAN_INVALID_ARG; }
    FiveMIT_Output batch;
    if (support_g ? (profile!=g_five_mit_hw || !FiveMIT_SampleFeedback(p,t,x,support_g,&batch)) :
                   !FiveMIT_Sample(p,t,x,profile,&batch)) { return BSP_CAN_INVALID_ARG; }
    return send_batch(&batch,motors,submitted);
}
BSP_CAN_Status_t FiveMIT_SendReference(const FiveMIT_Reference *r,double t,
    const FiveMIT_State *x,const float support_g[5],
    DM_Motor_t *const motors[5],unsigned *submitted)
{
    if (!submitted) { return BSP_CAN_INVALID_ARG; }
    *submitted=0;
    if (!valid_motors(motors)) { return BSP_CAN_INVALID_ARG; }
    FiveMIT_Output batch;
    if (!FiveMIT_SampleReferenceFeedback(r,t,x,support_g,&batch)) { return BSP_CAN_INVALID_ARG; }
    return send_batch(&batch,motors,submitted);
}
