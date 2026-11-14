#ifndef YLAMP_MOTOR_MODE_H
#define YLAMP_MOTOR_MODE_H

#include "drag_mode.h"
#include "motion_mode.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Independent behaviors; only physical model/coordinates are shared. */
typedef enum { MOTOR_MODE_DRAG = 0, MOTOR_MODE_MOTION = 1 } MotorMode_Id_t;
typedef enum {
    MOTOR_MODE_OFF = 0, MOTOR_MODE_ENTERING, MOTOR_MODE_RUNNING,
    MOTOR_MODE_STOPPING, MOTOR_MODE_HOLDING, MOTOR_MODE_RESUMING
} MotorMode_Phase_t;

typedef struct {
    MotorMode_Id_t id;
    union {
        DragMode_State_t drag;
        MotionMode_State_t motion;
    }; /* Private to the Modes, never interpreted by Task. */
} MotorMode_State_t;

typedef struct {
    DragMode_Status_t drag;
    MotionMode_Status_t motion;
} MotorMode_Diagnostics_t;

/* Task owns scheduling/gates/output; Mode owns trajectory and behavior.
 * Motion does not inherit Drag's measured-acceleration feedback or tuning. */
bool MotorMode_Available(MotorMode_Id_t id);
bool MotorMode_ValidConfig(MotorMode_Id_t id, const MotorTask_Config_t *config);
void MotorMode_Reset(MotorMode_State_t *state);
MotorMode_Phase_t MotorMode_Phase(const MotorMode_State_t *state);
bool MotorMode_TracksPosition(const MotorMode_State_t *state);
bool MotorMode_Enter(const MotorTask_Config_t *config,
                     const DM_Motor_Feedback_t feedback[2], MotorMode_State_t *state);
/* Explicitly capture ONE fixed center, never follow a hand-displaced pose.
 * Previous commands may be NULL when disabled; otherwise retain their damping
 * and support while ramping into Motion. No transport/enable side effects. */
bool MotorMode_CaptureHold(const MotorTask_Config_t *config,
                           const DM_Motor_Feedback_t feedback[2],
                           const DM_Motor_MIT_Command_t previous[2],
                           MotorMode_State_t *state);
bool MotorMode_Stop(const MotorTask_Config_t *config,
                    const DM_Motor_Feedback_t feedback[2], MotorMode_State_t *state,
                    float captured_target[2]);
bool MotorMode_Resume(MotorMode_State_t *state);
bool MotorMode_Move(const MotorTask_Config_t *config,
                    const MotionMode_Request_t *request, MotorMode_State_t *state);
bool MotorMode_Update(const MotorTask_Config_t *config,
                      const DM_Motor_Feedback_t feedback[2], float dt_s,
                      MotorMode_State_t *state, MotorMode_Diagnostics_t *diagnostics,
                      DM_Motor_MIT_Command_t commands[2]);

#ifdef __cplusplus
}
#endif
#endif
