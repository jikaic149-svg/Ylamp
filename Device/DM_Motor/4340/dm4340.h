#ifndef DEVICE_DM4340_H
#define DEVICE_DM4340_H

#include "../Common/dm_motor.h"

/* SDK 24V decoding scales, not mechanical or operating limits.
 * This project uses V13 for DM4340; no additional version is invented. */
#define DM4340_CONFIG_FIELDS \
    .model = DM_MOTOR_DM4340, .pmax = 12.5f, .vmax = 10.0f, .tmax = 28.0f, \
    .protocol = DM_MOTOR_PROTOCOL_V13

#endif
