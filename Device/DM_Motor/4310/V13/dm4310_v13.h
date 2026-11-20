#ifndef DEVICE_DM4310_V13_H
#define DEVICE_DM4310_V13_H

#include "../dm4310.h"

/* Independent 0xCC feedback and 0x33 register queries are supported.
 * Explicit configured protocol, not automatic firmware detection. */
#define DM4310_V13_CONFIG_FIELDS \
    DM4310_CONFIG_FIELDS, .protocol = DM_MOTOR_PROTOCOL_V13

#endif
