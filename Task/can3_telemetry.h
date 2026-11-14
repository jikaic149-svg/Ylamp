#ifndef YLAMP_CAN3_TELEMETRY_H
#define YLAMP_CAN3_TELEMETRY_H

#include "motor_config.h"
#include "motor_snapshot.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN3_TELEMETRY_VERSION MOTOR_SNAPSHOT_VERSION
#define CAN3_TELEMETRY_LENGTH MOTOR_SNAPSHOT_LENGTH
#define CAN3_TELEMETRY_RECORD_SIZE MOTOR_SNAPSHOT_RECORD_SIZE

typedef struct {
    uint32_t snapshots, frames_submitted, snapshots_dropped;
    BSP_CAN_Status_t init_status, last_transport;
} CAN3Telemetry_Status_t;

/* One ordinary task owns Init/Run/GetStatus; no allocation, ISR calls or
 * reentrant use. Copies the five long-lived handles; never queries/commands
 * motors. Init takes the actual BSP_CAN3_InitFD result, sends nothing.
 * Run uses HAL ms (wrap-safe for scheduling gaps <2^31ms); samples latest
 * consistent per-motor feedback, NOT a simultaneous five-encoder capture.
 * Exactly one nonblocking submission per distinct tick; BUSY/error abandons
 * the remaining snapshot, next period starts new data without replay/recovery.
 * Status counts HW acceptance only, never claims ACK/PC delivery. */
BSP_CAN_Status_t CAN3Telemetry_Init(DM_Motor_t *const motors[MOTOR_CAN_JOINT_COUNT],
    uint32_t now_ms, BSP_CAN_Status_t transport_status);
void CAN3Telemetry_Run(uint32_t now_ms);
BSP_CAN_Status_t CAN3Telemetry_GetStatus(CAN3Telemetry_Status_t *status);

#ifdef __cplusplus
}
#endif
#endif
