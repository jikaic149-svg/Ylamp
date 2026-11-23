#ifndef YLAMP_FIVE_DYNAMICS_H
#define YLAMP_FIVE_DYNAMICS_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

#define FIVE_AXES 5
/* q: CAD radians, v: measured rad/s, a: planned rad/s^2. Output Nm.
 * With the model's vertical J1, absolute yaw does not change G/M/h; keep
 * measured v1 and planned a1 for centrifugal, Coriolis and inertia coupling.
 * Reflected armature included; no friction, passive damping or contact. */
bool FiveDynamics_Inverse(const float q[FIVE_AXES], const float v[FIVE_AXES],
                          const float a[FIVE_AXES], float tau[FIVE_AXES]);
/* Same nominal static G, plus subtree moment perpendicular to each joint axis
 * [Nm]. These CAD wrench proxies are NOT measured bearing normal forces.
 * All pointers required; both outputs remain unchanged on failure. */
bool FiveDynamics_GravityLoad(const float q[FIVE_AXES], float tau[FIVE_AXES],
                              float bending[FIVE_AXES]);
/* Static potential residual only: delta rows are effective [kg, kg*m X/Y/Z]
 * per link, not identified physical masses or inertia parameters. No division
 * by mass, dynamics, clipping or hardware. Inputs must be finite; all pointers
 * are required. On failure tau is unchanged. */
bool FiveDynamics_GravityResidual(const float q[FIVE_AXES],
    const float delta[FIVE_AXES][4], float tau[FIVE_AXES]);
#ifdef __cplusplus
}
#endif
#endif
