#ifndef YLAMP_FIVE_MOTION_H
#define YLAMP_FIVE_MOTION_H
/* One position-code tolerance, all five configured MIT ranges are +/-12.5 rad.
 * Only the START may use raw travel plus this quantization tolerance; every
 * target retains the original 1-degree inward margin. The extra 0.0001 deg
 * covers float32 roundoff, not mechanical travel. No feedback clamping. */
#define FIVE_ENCODER_TOL_DEG (25.0/65535.0*57.29577951308232+0.0001)
#endif
