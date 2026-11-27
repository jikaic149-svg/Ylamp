#include "five_workspace.h"
#include "five_motion.h"
#include <math.h>

/* User-requested per-axis admission, not a collision certificate. The old
 * J1=0 table and +/-5-degree yaw box are retained as offline evidence only.
 * Preserve measured raw travel, its 1-degree inset and the actual J4 turn. */
#define JOINT_REFERENCE(joint, zero, lower, upper, kind) {(lower)-(zero)+1.0,(upper)-(zero)-1.0},
static const double travel[5][2] = {
#include "joint_references.def"
};
#undef JOINT_REFERENCE
static bool in_travel(const double q[5])
{
    if (!q) { return false; }
    for (int i=0;i<5;++i) {
        if (!(q[i]>=travel[i][0] && q[i]<=travel[i][1])) { return false; }
    }
    return true;
}
bool FiveWorkspace_BoxJ3Band(const double q[5],const double upper[5],double out[2])
{
    if (!out || !in_travel(q) || !in_travel(upper) || q[2]!=upper[2]) { return false; }
    for (int i=0;i<5;++i) { if (q[i]>upper[i]) { return false; } }
    out[0]=travel[2][0];out[1]=travel[2][1];return true;
}
bool FiveWorkspace_J3Band(const double q[5],double out[2])
{
    return FiveWorkspace_BoxJ3Band(q,q,out);
}
bool FiveWorkspace_Line(const double a[5],const double b[5])
{
    if (!a || !in_travel(b)) { return false; }
    /* Explicit inward departure from a measured endpoint, never an outward
     * target. Monotone progress stays between start and the inset target. */
    for (int i=0;i<5;++i) {
        if (!(a[i]>=travel[i][0]-1.-FIVE_ENCODER_TOL_DEG &&
              a[i]<=travel[i][1]+1.+FIVE_ENCODER_TOL_DEG)) { return false; }
    }
    return true;
}
bool FiveWorkspace_TrajectoryBounds(const double a[5],const double b[5],
                                    const double lo[5],const double hi[5])
{
    if (!lo || !hi || !FiveWorkspace_Line(a,b)) { return false; }
    for (unsigned i=0;i<5;++i) {
        /* Numerical root roundoff only, not a new mechanical margin. */
        if (!isfinite(lo[i]) || !isfinite(hi[i]) || lo[i]>hi[i] ||
            lo[i]<fmin(a[i],travel[i][0])-1e-7 ||
            hi[i]>fmax(a[i],travel[i][1])+1e-7) { return false; }
    }
    return true;
}
