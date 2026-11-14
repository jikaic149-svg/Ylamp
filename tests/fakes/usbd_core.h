#ifndef YLAMP_TEST_USBD_CORE_H
#define YLAMP_TEST_USBD_CORE_H

#include "main.h"

typedef enum {USBD_OK = 0, USBD_BUSY, USBD_FAIL} USBD_StatusTypeDef;
typedef struct {uint8_t unused;} USBD_ClassTypeDef;
typedef struct {uint8_t unused;} USBD_DescriptorsTypeDef;
typedef struct {
    uint8_t bmRequest, bRequest;
    uint16_t wValue, wIndex, wLength;
} USBD_SetupReqTypedef;
typedef struct {
    void *pClassDataCmsit[1];
    uint8_t classId, dev_state;
} USBD_HandleTypeDef;
typedef struct {uint8_t unused;} PCD_HandleTypeDef;

#define DEVICE_HS 1U
#define USBD_STATE_DEFAULT 1U
#define USBD_STATE_CONFIGURED 3U
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *, USBD_DescriptorsTypeDef *, uint8_t);
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *, USBD_ClassTypeDef *);
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *);
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *);
#endif
