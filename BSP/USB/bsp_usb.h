#ifndef YLAMP_BSP_USB_H
#define YLAMP_BSP_USB_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_USB_TX_CAPACITY 512U

typedef enum {
    BSP_USB_OK = 0, BSP_USB_INVALID_ARG, BSP_USB_NOT_READY, BSP_USB_BUSY, BSP_USB_ERROR
} BSP_USB_Status_t;

typedef struct {
    uint32_t submitted, completed, busy, not_ready, errors, rx_discarded, enumerations;
    BSP_USB_Status_t init_status;
    bool configured, dtr, tx_busy;
} BSP_USB_Diagnostics_t;

/* Board02 Type-C: OTG_HS controller, embedded FULL SPEED PHY (12Mbps), DMA OFF.
 * Init once at boot, before scheduler; no implicit retries. Vendor clock/PCD
 * fatal setup errors retain the SDK Error_Handler path. No cable is not fatal.
 * Send/GetDiagnostics: one ordinary task, not ISR/reentrant. Send copies
 * 1..512 bytes into one owned static buffer; caller may immediately reuse data.
 * Requires USB configured AND host DTR. BUSY/not-ready does not overwrite an
 * in-flight buffer, wait, queue or retry. Completion/disconnect is stack-owned;
 * never force-clear TxState. OUT bytes enter a bounded 256-byte ring; Read is
 * nonblocking, one task consumer. Epoch changes on disconnect/DTR/overflow:
 * discard any partial frame/session; overflow never silently joins commands.
 * OK = CDC stack accepted only (vendor class ignores LL transmit return).
 * completed = device IN completion, NOT proof of host-application processing.
 * No heap, DMA/cache maintenance, motor commands or USB-to-CAN bridge. */
BSP_USB_Status_t BSP_USB_Init(void);
BSP_USB_Status_t BSP_USB_Send(const uint8_t *data, uint16_t length);
BSP_USB_Status_t BSP_USB_Read(uint8_t *byte, uint32_t *at_ms, uint32_t *epoch);
uint32_t BSP_USB_RxEpoch(void);
BSP_USB_Status_t BSP_USB_GetDiagnostics(BSP_USB_Diagnostics_t *out);
void BSP_USB_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif
