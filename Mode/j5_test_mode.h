#ifndef YLAMP_J5_TEST_MODE_H
#define YLAMP_J5_TEST_MODE_H

#include "motor_config.h"

typedef enum {
    J5_MODE_ENTERING = 0, J5_MODE_OUTBOUND, J5_MODE_RETURNING, J5_MODE_HOLDING
} J5TestMode_Phase_t;

typedef struct {
    float origin_rad;
    J5TestMode_Phase_t phase;
} J5TestMode_State_t;

/* Pure reference/lifecycle: ramp stiffness, one rest-to-rest round trip,
 * then hold origin. Task owns feedback, enable/disable and CAN submission. */
void J5TestMode_Begin(float origin_rad, J5TestMode_State_t *state);
void J5TestMode_Update(const J5Test_Config_t *config, uint32_t elapsed_ms,
                       J5TestMode_State_t *state,
                       DM_Motor_MIT_Command_t *command);

#endif
