#ifndef YLAMP_FIVE_COORDINATES_H
#define YLAMP_FIVE_COORDINATES_H
/* Degrees: qCAD = (raw_reference - shared_zero) + offset, encoder scale +1.
 * Pitch installation offset is provisionally zero. J2 Pitch=0 maps upright.
 * During the J3 sweep J2 raw was 154.75808470469042 deg, not its new zero
 * 146.18099179755507 deg. Subtract that 8.57709290713535 deg upstream angle
 * from the old simultaneous-upright J3 offset. This is a link-axis datum,
 * NOT a fitted subtree-COM equilibrium or a torque-sign switch. */
#define J2_CAD 37.131618232078196
#define J3_CAD (-103.83238843386283)
#define FIVE_CAD_OFFSETS {0.0,J2_CAD,J3_CAD,0.0,0.0}
/* Geometry evidence stays pinned to its historical CAD basis; do not edit
 * these to move/widen a certified region when a calibration changes. */
#define FIVE_WORKSPACE_CAD_OFFSETS {0.0,37.131618232078196,-95.25529552672751,0.0,0.0}
#endif
