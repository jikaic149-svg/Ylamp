#ifndef YLAMP_USB_CONFIG_H
#define YLAMP_USB_CONFIG_H

/* Target reporting cadence only; motor acquisition/control remains unchanged.
 * Five * 48 bytes / ms = 240kB/s application payload, FS link 12Mbps.
 * Actual throughput/jitter/CPU budget require a board+host measurement. */
#define USB_TELEMETRY_PERIOD_MS 1U
#define USB_TELEMETRY_MAX_AGE_MS 100U
/* Unified five-encoder + dual-IMU observer; not a motor-control mode. */
#define USB_FOCUS_PERIOD_MS 10U
#define USB_FOCUS_MAX_AGE_MS 40U

#endif
