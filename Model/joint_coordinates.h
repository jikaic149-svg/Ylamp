#ifndef YLAMP_JOINT_COORDINATES_H
#define YLAMP_JOINT_COORDINATES_H

#include <math.h>

#define JOINT_TURN_RAD 6.2831853071795864769f

/* Common encoder/model mapping, independent of either Mode.
 * Only select an equivalent 2*pi branch; never clamp feedback or infer polarity.
 * Startup anchor: configured reference-range midpoint (range must be <1 turn).
 * Running anchor: previous reference angle. Raw Device feedback stays untouched.
 * A non-integer-turn encoder reset requires calibration, not this mapping. */
static inline float JointCoordinates_Offset(float raw_rad, float anchor_rad)
{
    if (!isfinite(raw_rad) || !isfinite(anchor_rad)) { return NAN; }
    return JOINT_TURN_RAD * roundf((anchor_rad - raw_rad) / JOINT_TURN_RAD);
}

#endif
