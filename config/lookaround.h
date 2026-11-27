#ifndef YLAMP_LOOKAROUND_CONFIG_H
#define YLAMP_LOOKAROUND_CONFIG_H
#include "five_trajectory.h"

/* Dedicated 2 x 4KiB slots, before (never overlapping) five-axis history.
 * Nonblank foreign slots are reported, never automatically formatted.
 * No built-in point-table fallback: provisioning is an explicit USB save. */
#define FIVE_LOOKAROUND_BASE 0x007fa000U
#define FIVE_LOOKAROUND_BYTES 8192U
#define FIVE_LOOKAROUND_PAGE_TIMEOUT_MS 100U
#define FIVE_LOOKAROUND_ERASE_TIMEOUT_MS 5000U
#endif
