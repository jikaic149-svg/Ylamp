#include "j5_test_mode.h"

static float smooth(float s)
{
    return s * s * s * (10.0f + s * (-15.0f + 6.0f * s));
}

static float smooth_velocity(float s)
{
    return 30.0f * s * s * (1.0f - s) * (1.0f - s);
}

void J5TestMode_Begin(float origin_rad, J5TestMode_State_t *state)
{
    *state = (J5TestMode_State_t){
        .origin_rad = origin_rad, .phase = J5_MODE_ENTERING
    };
}

void J5TestMode_Update(const J5Test_Config_t *config, uint32_t elapsed_ms,
                       J5TestMode_State_t *state,
                       DM_Motor_MIT_Command_t *command)
{
    *command = (DM_Motor_MIT_Command_t){
        .position_rad = state->origin_rad, .kp = config->kp, .kd = config->kd
    };
    if (elapsed_ms < config->entry_ms) {
        state->phase = J5_MODE_ENTERING;
        command->kp *= smooth((float)elapsed_ms / (float)config->entry_ms);
        return;
    }
    elapsed_ms -= config->entry_ms;
    if (elapsed_ms < 2U * config->leg_ms) {
        const bool returning = elapsed_ms >= config->leg_ms;
        if (returning) { elapsed_ms -= config->leg_ms; }
        const float s = (float)elapsed_ms / (float)config->leg_ms;
        const float direction = returning ? -1.0f : 1.0f;
        state->phase = returning ? J5_MODE_RETURNING : J5_MODE_OUTBOUND;
        command->position_rad += config->amplitude_rad *
            (returning ? 1.0f - smooth(s) : smooth(s));
        command->velocity_rad_s = direction * config->amplitude_rad *
            smooth_velocity(s) * 1000.0f / (float)config->leg_ms;
        return;
    }
    state->phase = J5_MODE_HOLDING;
}
