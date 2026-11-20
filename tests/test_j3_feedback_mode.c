#include "j3_feedback_mode.h"

#include <assert.h>
#include <stdio.h>

static const MotorTask_Config_t config = {
    .joint = {{.temperature_c = 60U}},
    .query_period_ms = 10U, .feedback_max_age_ms = 100U, .enable_timeout_ms = 500U
};
static J3FeedbackMode_t mode;
static DM_Motor_Feedback_t feedback;
static uint32_t now;

static J3Feedback_Action_t update(void)
{
    return J3FeedbackMode_Update(&mode, &config, &feedback, now);
}
static void submit(J3Feedback_Action_t action, BSP_CAN_Status_t result)
{
    assert(update() == action);
    J3FeedbackMode_Submitted(&mode, action, result, feedback.received_count, now);
}
static void reply(uint8_t state)
{
    feedback.state = state;
    feedback.received_at_ms = now;
    ++feedback.received_count;
}
static void begin(uint32_t at)
{
    now = at;
    mode = (J3FeedbackMode_t){0};
    feedback = (DM_Motor_Feedback_t){.position_rad = 0.4f, .received_at_ms = now};
    assert(update() == J3_ACTION_NONE); /* boot never enables */
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    assert(update() == J3_ACTION_NONE); /* cached state=0 is NOT confirmation */
    reply(0U);
    submit(J3_ACTION_MIT, BSP_CAN_OK);
    submit(J3_ACTION_ENABLE, BSP_CAN_OK);
    reply(1U);
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_STREAMING);
}
static void stopped(void)
{
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_STOPPING);
    reply(0U);
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_OFF);
}

int main(void)
{
    begin(1000U);
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_BUSY);
    for (unsigned i = 0U; i < 300U; ++i) {
        now += 9U;
        assert(update() == J3_ACTION_NONE);
        ++now;
        submit(J3_ACTION_MIT, BSP_CAN_OK);
        reply(1U);
    } /* 3 seconds: no hidden 2-second bench deadline. */
    J3FeedbackMode_Stop(&mode);
    stopped();
    assert(update() == J3_ACTION_NONE && mode.reason == J3_FEEDBACK_OK);

    begin(UINT32_MAX - 5U);
    now += 10U;
    submit(J3_ACTION_MIT, BSP_CAN_OK); /* wrap-safe cadence */
    reply(1U);
    now += 100U; /* no burst replay after delayed scheduling */
    submit(J3_ACTION_MIT, BSP_CAN_OK);
    assert(update() == J3_ACTION_NONE);
    reply(1U);
    J3FeedbackMode_Stop(&mode);
    stopped();

    begin(1000U);
    now += 101U;
    submit(J3_ACTION_DISABLE, BSP_CAN_OK); /* lost source feedback */
    assert(mode.reason == J3_FEEDBACK_STALE);
    reply(0U);
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_OFF);
    now += 1000U;
    reply(1U);
    assert(update() == J3_ACTION_NONE); /* no automatic restart */

    begin(1000U);
    now += 10U;
    submit(J3_ACTION_MIT, BSP_CAN_BUSY);
    assert(mode.reason == J3_FEEDBACK_TRANSPORT);
    stopped();

    begin(1000U);
    feedback.mos_temperature_c = 60U;
    stopped();
    assert(mode.reason == J3_FEEDBACK_TEMPERATURE);
    begin(1000U);
    reply(8U);
    stopped();
    assert(mode.reason == J3_FEEDBACK_STATE);

    begin(1000U);
    J3FeedbackMode_Stop(&mode);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    now += 499U;
    J3FeedbackMode_Stop(&mode); /* does not extend an in-flight stop */
    assert(update() == J3_ACTION_NONE);
    ++now;
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_UNCONFIRMED);
    assert(mode.reason == J3_FEEDBACK_DISABLE_TIMEOUT);
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_BUSY);
    J3FeedbackMode_Stop(&mode); /* explicit retry of cleanup, not auto-retry */
    stopped();
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_ERROR);
    assert(mode.phase == J3_FEEDBACK_UNCONFIRMED);

    mode = (J3FeedbackMode_t){0};
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    now += 500U;
    assert(update() == J3_ACTION_NONE && mode.phase == J3_FEEDBACK_UNCONFIRMED);
    assert(mode.reason == J3_FEEDBACK_DISABLE_TIMEOUT); /* no enable without FD reply */

    mode = (J3FeedbackMode_t){0};
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    reply(0U);
    submit(J3_ACTION_MIT, BSP_CAN_ERROR);
    assert(mode.phase == J3_FEEDBACK_OFF && mode.reason == J3_FEEDBACK_TRANSPORT);

    mode = (J3FeedbackMode_t){0};
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    reply(0U);
    submit(J3_ACTION_MIT, BSP_CAN_OK);
    submit(J3_ACTION_ENABLE, BSP_CAN_OK);
    for (unsigned i = 0U; i < 49U; ++i) {
        now += 10U;
        reply(0U);
        submit(J3_ACTION_MIT, BSP_CAN_OK); /* MIT ACKs cannot fake enable=1 */
    }
    now += 10U;
    reply(0U);
    stopped();
    assert(mode.reason == J3_FEEDBACK_ENABLE_TIMEOUT);

    mode = (J3FeedbackMode_t){0};
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    submit(J3_ACTION_DISABLE, BSP_CAN_OK);
    reply(0U);
    submit(J3_ACTION_MIT, BSP_CAN_OK);
    reply(8U); /* Fault appearing between preload and enable: no enable. */
    stopped();
    assert(mode.reason == J3_FEEDBACK_STATE);

    mode = (J3FeedbackMode_t){0};
    assert(J3FeedbackMode_Start(&mode) == BSP_CAN_OK);
    J3FeedbackMode_Stop(&mode); /* Cancelled before the first TX. */
    stopped();
    assert(mode.phase == J3_FEEDBACK_OFF);
    puts("J3 pure lifecycle/start/100Hz/stop/fresh confirmation/wrap/failures: PASS");
    return 0;
}
