#ifndef YLAMP_MOTOR_USB_H
#define YLAMP_MOTOR_USB_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Ordinary app task only, AFTER MotorTuning_Init; never init in bench builds.
 * Same reliable envelope as UART; allowlisted J1/J2 quick commands only.
 * Run returns false while an RPC reply owns the next USB TX slot. */
void MotorUSB_Init(void);
bool MotorUSB_Run(void);
#ifdef __cplusplus
}
#endif
#endif
