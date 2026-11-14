#ifndef YLAMP_SERIAL_CONFIG_H
#define YLAMP_SERIAL_CONFIG_H

#define YLAMP_SERIAL_BAUD_RATE 230400U
/* v2: 2 joints = 200 bytes/packet at 50Hz = 10000 bytes/s.
 * MotorSerial also scales the period for larger record counts to keep
 * telemetry below 60% of the 23040 byte/s 8N1 budget.
 * RPC stays v1. BUSY drops telemetry, never blocks control. */
#define YLAMP_SERIAL_PERIOD_MS 20U

#endif
