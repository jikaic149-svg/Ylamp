#ifndef YLAMP_TEST_USBD_CDC_H
#define YLAMP_TEST_USBD_CDC_H

#include "usbd_core.h"

#define CDC_DATA_FS_OUT_PACKET_SIZE 64U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
#define CDC_SET_CONTROL_LINE_STATE 0x22U
typedef struct {
    volatile uint32_t TxState;
    uint8_t *TxBuffer, *RxBuffer;
    uint32_t TxLength;
} USBD_CDC_HandleTypeDef;
typedef struct {
    int8_t (*Init)(void);
    int8_t (*DeInit)(void);
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
extern USBD_ClassTypeDef USBD_CDC;
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *, USBD_CDC_ItfTypeDef *);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *, uint8_t *, uint32_t);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *, uint8_t *);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *);
#endif
