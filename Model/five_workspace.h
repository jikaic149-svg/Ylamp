#ifndef YLAMP_FIVE_WORKSPACE_H
#define YLAMP_FIVE_WORKSPACE_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Per-axis travel preflight only, live software degrees, J1/J4 unwrapped.
 * Uses joint_references.def raw bounds with the retained 1-degree inset.
 * No J1=0 restriction, +/-5-degree yaw box or frozen-table angle whitelist.
 * This is NOT a collision/obstacle/cable/clearance certificate. The motion
 * owner must still enforce feedback, protocol, torque and fault handling.
 * Null, nonfinite or out-of-travel inputs return false, leaving band intact.
 * J3Band returns the complete inset J3 travel interval. */
bool FiveWorkspace_J3Band(const double at[5], double band[2]);
/* Ordered per-axis box; endpoints must use the same valid seed J3. */
bool FiveWorkspace_BoxJ3Band(const double lower[5], const double upper[5], double band[2]);
bool FiveWorkspace_Line(const double start[5], const double end[5]);
/* Exact per-axis trajectory extrema. Only an actual start outside the inset
 * may extend the envelope to that start, never farther outward. */
bool FiveWorkspace_TrajectoryBounds(const double start[5],const double end[5],
                                    const double minimum[5],const double maximum[5]);
#ifdef __cplusplus
}
#endif
#endif
