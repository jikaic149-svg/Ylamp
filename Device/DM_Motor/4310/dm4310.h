#ifndef DEVICE_DM4310_H
#define DEVICE_DM4310_H

#include "../Common/dm_motor.h"

/* SDK 24V decoding scales, not mechanical or operating limits.
 * Shared by the two configured protocol variants; verify each motor's values. */
#define DM4310_CONFIG_FIELDS \
    .model = DM_MOTOR_DM4310, .pmax = 12.5f, .vmax = 30.0f, .tmax = 10.0f

#endif
