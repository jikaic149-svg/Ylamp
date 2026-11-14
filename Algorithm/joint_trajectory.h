#ifndef YLAMP_JOINT_TRAJECTORY_H
#define YLAMP_JOINT_TRAJECTORY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed two-axis local PTP, not a claim that the robot model supports more axes.
 * SI units throughout: rad, rad/s, rad/s^2, rad/s^3, seconds.
 * Input angles must already be on the intended continuous encoder branch.
 * No calibration, hardware, RTOS, MIT gains, torque limits or Mode policy here. */
#define JOINT_TRAJECTORY_AXES 2U

typedef struct {
    double position, velocity, acceleration;
    double target_position, target_velocity, target_acceleration;
    double max_velocity, max_acceleration, max_jerk; /* Finite and > 0. */
} JointTrajectory_Axis_t;

typedef struct {
    JointTrajectory_Axis_t axis[JOINT_TRAJECTORY_AXES];
} JointTrajectory_Input_t;

typedef struct {
    double time_s; /* min(requested time, duration); no extrapolation. */
    double position[JOINT_TRAJECTORY_AXES];
    double velocity[JOINT_TRAJECTORY_AXES];
    double acceleration[JOINT_TRAJECTORY_AXES];
    double jerk[JOINT_TRAJECTORY_AXES];
} JointTrajectory_Sample_t;

/* Preserve native Ruckig error codes; NOT_READY is local to this C adapter. */
typedef int32_t JointTrajectory_Result_t;
enum {
    JOINT_TRAJECTORY_OK = 0,
    JOINT_TRAJECTORY_FINISHED = 1,
    JOINT_TRAJECTORY_ERROR = -1,
    JOINT_TRAJECTORY_INVALID_INPUT = -100,
    JOINT_TRAJECTORY_DURATION_ERROR = -101,
    JOINT_TRAJECTORY_POSITION_ERROR = -102,
    JOINT_TRAJECTORY_ZERO_LIMIT_ERROR = -104,
    JOINT_TRAJECTORY_CALCULATION_ERROR = -110,
    JOINT_TRAJECTORY_SYNCHRONIZATION_ERROR = -111,
    JOINT_TRAJECTORY_NOT_READY = -1000
};

/* One private static planner, owned by one calling task/Mode; NOT reentrant.
 * Explicit Init creates it without heap/global constructors. Re-init/Reset
 * invalidate the trajectory only: neither operation stops an actual motor.
 * period_s must be finite and > 0; Init failure also invalidates any old plan.
 * Do not call from an ISR. There are no waits or implicit background tasks. */
JointTrajectory_Result_t JointTrajectory_Init(double period_s);
void JointTrajectory_Reset(void);

/* Time synchronization, discrete duration in multiples of period_s.
 * All p/v/a and limits must be finite. Current v/a may exceed the new limits:
 * upstream braking restores feasibility; this is not an instantaneous trip.
 * All calculation attempts invalidate the previous plan, including bad inputs.
 * Success writes duration_s; failure leaves it unchanged and disables sampling.
 * Both pointers are required. Result < 0 is an error, never a usable plan.
 * This does NOT enforce position/torque/collision limits or a Cartesian path. */
JointTrajectory_Result_t JointTrajectory_Calculate(
    const JointTrajectory_Input_t *input, double *duration_s);

/* Same lifetime/error contract as Calculate. Velocity interface plans to zero
 * velocity/acceleration; terminal positions are determined by braking, not by
 * target_position. Per-axis shortest braking (not Time stretching); common
 * sampling clock, completed joints hold. Not a hardware emergency stop. */
JointTrajectory_Result_t JointTrajectory_Brake(
    const JointTrajectory_Input_t *input, double *duration_s);
/* Exact upstream position extrema of the current plan, including overshoot.
 * Required arrays contain AXES entries. Failure leaves both unchanged. */
JointTrajectory_Result_t JointTrajectory_PositionBounds(
    double minimum[JOINT_TRAJECTORY_AXES], double maximum[JOINT_TRAJECTORY_AXES]);

/* time_s must be finite and >= 0; output is required and unchanged on failure.
 * Returns FINISHED at/after duration, with the terminal state (not extrapolated).
 * Nonzero terminal v/a is NOT a hold command; Mode must decide what follows.
 * Sampling has no cursor and does not replan: both axes use exactly one time. */
JointTrajectory_Result_t JointTrajectory_Sample(
    double time_s, JointTrajectory_Sample_t *output);

#ifdef __cplusplus
}
#endif
#endif
