#ifndef YLAMP_MOTION_MODE_H
#define YLAMP_MOTION_MODE_H

#include "motor_config.h"
#include "joint_trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_MODE_OFF = 0, MOTION_MODE_ENTERING, MOTION_MODE_HOLDING,
    MOTION_MODE_MOVING, MOTION_MODE_STOPPING
} MotionMode_Phase_t;

typedef struct {
    /* Reference encoder branch, rad; J2 link-UP is drag_j2_zero_rad, NOT 0.
     * Positive finite limits are specific to this move, not protocol maxima. */
    float target_rad[2];
    float velocity_rad_s[2], acceleration_rad_s2[2], jerk_rad_s3[2];
} MotionMode_Request_t;

typedef struct {
    MotionMode_Phase_t phase;
    float entry_blend;
    float entry_kd[2], entry_ff_nm[2]; /* Preserve support during a live handoff. */
    float encoder_position_rad[2], filtered_velocity_rad_s[2];
    uint32_t encoder_sample_ms[2], encoder_sample_count[2];
    double elapsed_s, duration_s;
    JointTrajectory_Sample_t reference;
    MotionMode_Request_t request;
} MotionMode_State_t;

typedef struct {
    bool valid;
    MotionMode_Phase_t phase;
    float elapsed_s, duration_s;
    float target_acceleration_rad_s2[2];
    float position_error_rad[2], velocity_error_rad_s[2];
    RobotModel_Dynamics_t model; /* Nominal M*ddq_des, c(q,dq), g(q). */
    float friction_velocity_rad_s[2]; /* Filtered encoder derivative, one-count noise removed. */
    float friction_nm[2]; /* Measured-motion FF, not feedback torque or external force. */
} MotionMode_Status_t;

/* One active Motion owns the static trajectory planner. Ordinary task only.
 * Task owns feedback validity, enable/disable, faults and the only CAN output.
 * Enter captures current pose and ramps gains/support; no move is implicit.
 * Move accepts one finite, bounded rest-to-rest job only while HOLDING.
 * Rejection leaves the hold target intact; no fault/disable or old-job replay.
 * Stop brakes the current REFERENCE p/v/a, then holds its resulting endpoint.
 * Stop failure invalidates the planner: Task must use its existing fault exit,
 * not continue sampling the old move. Invalid Move is safe only in HOLDING.
 * Friction uses new encoder positions/sample timestamps, then a low-pass and
 * a resolution-derived soft deadband; a cached sample is not a new zero speed.
 * Reset clears both the plan and estimator, not the actual actuator. */
bool MotionMode_ValidConfig(const MotorTask_Config_t *config);
void MotionMode_Reset(MotionMode_State_t *state);
bool MotionMode_Enter(const MotorTask_Config_t *config,
                      const DM_Motor_Feedback_t feedback[2], MotionMode_State_t *state);
bool MotionMode_Move(const MotorTask_Config_t *config,
                     const MotionMode_Request_t *request, MotionMode_State_t *state);
bool MotionMode_Stop(const MotorTask_Config_t *config, MotionMode_State_t *state);
bool MotionMode_Update(const MotorTask_Config_t *config,
                       const DM_Motor_Feedback_t feedback[2], float dt_s,
                       MotionMode_State_t *state, MotionMode_Status_t *status,
                       DM_Motor_MIT_Command_t commands[2]);

#ifdef __cplusplus
}
#endif
#endif
