#ifndef YLAMP_JOINT_COORDINATES_H
#define YLAMP_JOINT_COORDINATES_H

#include <math.h>

#define JOINT_TURN_RAD 6.2831853071795864769f

/* Subtract the zero after selecting a reference branch when appropriate. */
static inline float JointCoordinates_Angle(float raw_rad, float zero_rad)
{
    const float angle = raw_rad - zero_rad;
    return isfinite(raw_rad) && isfinite(zero_rad) && isfinite(angle) ? angle : NAN;
}

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

/* Stateless display/recording mapping. A range narrower than one turn gives
 * an unambiguous branch; use its midpoint, never the sign of the raw angle.
 * Wide ranges (J4) must retain the actual multi-turn encoder reading.
 * Out-of-range readings stay out of range; this is NOT a feedback clamp. */
static inline float JointCoordinates_Reference(float raw_rad, float lower_rad,
                                               float upper_rad)
{
    if (!isfinite(raw_rad) || !isfinite(lower_rad) || !isfinite(upper_rad) ||
        lower_rad >= upper_rad) { return NAN; }
    return upper_rad - lower_rad < JOINT_TURN_RAD ?
        raw_rad + JointCoordinates_Offset(raw_rad, (lower_rad + upper_rad) * 0.5f) :
        raw_rad;
}

#endif
