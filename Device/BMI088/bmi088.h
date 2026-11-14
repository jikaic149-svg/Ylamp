#ifndef YLAMP_BMI088_H
#define YLAMP_BMI088_H

#include "bsp_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float accel_m_s2[3]; /* Sensor axes, not robot/world axes. */
    float gyro_rad_s[3];
    uint32_t received_at_ms;
    uint32_t sequence;
} BMI088_Sample_t;

/* Boot only: chip IDs, soft reset, configure/read back registers, then SUSPEND.
 * No reference-board affine/bias constants, heater, RTOS or auto recovery.
 * Failure leaves the IMU unavailable; does not change any motor output. */
HAL_StatusTypeDef BMI088_Init(void);
/* Explicit cooperative session power transition. Default OFF; no sensor reset.
 * BUSY on an in-flight transfer: finish it before retrying OFF; never abort or
 * reuse borrowed buffers. ON/OK accepts the session, not completed power-on.
 * Run advances six SPI-IT transfers, 30/5/5ms settling and register readback
 * without blocking the control task. OFF returns BUSY until Run confirms all
 * power registers; repeated OFF then returns OK. No samples until completion
 * plus 100ms warmup; a transition failure latches unavailable, never resets. */
HAL_StatusTypeDef BMI088_SetEnabled(bool enabled, uint32_t now_ms);
/* False also means unavailable/transition failed, not proof of ACTIVE. */
bool BMI088_IsSuspended(void);
/* Main loop: bounded nonblocking SPI-IT steps. A timed-out transfer is latched
 * unavailable, its static buffers are not reused even if an IRQ arrives late. */
void BMI088_Run(uint32_t now_ms);
HAL_StatusTypeDef BMI088_GetSample(BMI088_Sample_t *sample);

#ifdef __cplusplus
}
#endif
#endif
