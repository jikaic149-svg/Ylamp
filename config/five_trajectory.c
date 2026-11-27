#include "five_trajectory.h"
/* Initial active-path smoothness budgets, SI units. .8 rad/s is within the
 * previous speed-identification range; a/j are planning choices (about .2s
 * acceleration rise), NOT identified actuator ratings or verified tracking.
 * Independent of Move/Speed/drag/KEY gains, G/F/M/C and existing Nm ceilings.
 * The host evaluates the complete retimed path before any explicit commit. */
const FiveTrajectory_Limit g_five_trajectory_limits[5] = {
    {.8,2,10}, {.8,2,10}, {.8,4,20}, {.8,2,10}, {.8,4,20}
};
