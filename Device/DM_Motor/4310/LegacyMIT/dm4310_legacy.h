#ifndef DEVICE_DM4310_LEGACY_H
#define DEVICE_DM4310_LEGACY_H

#include "../dm4310.h"

/* Command-response MIT feedback only; no independent 0xCC/0x33 queries.
 * J3's configured variant; this does not identify its firmware at runtime. */
#define DM4310_LEGACY_CONFIG_FIELDS \
    DM4310_CONFIG_FIELDS, .protocol = DM_MOTOR_PROTOCOL_LEGACY_MIT

#endif
