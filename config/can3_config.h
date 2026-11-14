#ifndef YLAMP_CAN3_CONFIG_H
#define YLAMP_CAN3_CONFIG_H

/* Display/recording only; does not change motor acquisition/control cadence.
 * Run from the existing 1ms task; one FD frame per distinct tick, five per 10ms. */
#define CAN3_TELEMETRY_PERIOD_MS 10U
#define CAN3_TELEMETRY_MAX_AGE_MS 100U
#define CAN3_TELEMETRY_ID 0x600U /* dedicated CAN3, standard ID */

#endif
