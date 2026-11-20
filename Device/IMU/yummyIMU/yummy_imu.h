#ifndef YLAMP_YUMMY_IMU_H
#define YLAMP_YUMMY_IMU_H

#include "bsp_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { YUMMY_IMU_USE = 0, YUMMY_IMU_BINARY = 1 } YummyIMU_Format_t;

typedef struct {
    float pitch_deg, roll_deg, yaw_deg;
    uint32_t received_at_ms, sequence, rx_bytes, bad_frames, overflows;
    uint32_t uart_error; /* Accumulated UART errors, independent of current RX. */
    YummyIMU_Format_t format;
    bool rx_running; /* Current RX/rearm state; does not imply fresh angles. */
} YummyIMU_Status_t;

/* One UART owner, configured by the board before Init; default YummyIMU is
 * 921600/8N1, USE (Pitch,Roll,Yaw degrees + newline), 50Hz.
 * BINARY is the documented A5 + 3*int16LE/100 + sum8 alternative.
 * No commands, flash writes, automatic format guessing or RX recovery.
 * CUSTOM/DEBUG/INS require an explicit different contract, not guessed angles.
 * Init/Run/GetStatus are serialized in the application task, never in ISR. */
HAL_StatusTypeDef YummyIMU_Init(UART_HandleTypeDef *uart, YummyIMU_Format_t format);
void YummyIMU_Run(void);
void YummyIMU_GetStatus(YummyIMU_Status_t *status);

#ifdef __cplusplus
}
#endif
#endif
