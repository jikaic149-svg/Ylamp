#include "bsp_usb.h"
#include "main.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"

#include <string.h>

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
static USBD_HandleTypeDef device;
static uint8_t tx_buffer[BSP_USB_TX_CAPACITY];
/* The selected PHY is FS, irrespective of the controller/descriptor HS names. */
static uint8_t rx_buffer[CDC_DATA_FS_OUT_PACKET_SIZE];
static volatile BSP_USB_Diagnostics_t diagnostics;
static volatile bool host_dtr;
static bool attempted, started;
static uint8_t rx_ring[256];
static uint32_t rx_time[256];
static volatile uint16_t rx_head, rx_tail;
static volatile uint32_t rx_epoch;

static void reset_rx(void)
{
    rx_tail = rx_head;
    ++rx_epoch;
}

static int8_t cdc_init(void)
{
    host_dtr = false;
    reset_rx();
    ++diagnostics.enumerations;
    if (USBD_CDC_SetTxBuffer(&device, tx_buffer, 0U) != USBD_OK ||
        USBD_CDC_SetRxBuffer(&device, rx_buffer) != USBD_OK) { return USBD_FAIL; }
    return USBD_OK;
}

static int8_t cdc_deinit(void)
{
    host_dtr = false;
    reset_rx();
    /* The stack closes endpoints before returning the in-flight buffer. */
    return USBD_OK;
}

static int8_t cdc_control(uint8_t command, uint8_t *data, uint16_t length)
{
    if (command == CDC_SET_CONTROL_LINE_STATE && length == 0U && data != NULL) {
        /* ST passes the setup request itself for zero-length class requests. */
        const USBD_SetupReqTypedef *request = (const USBD_SetupReqTypedef *)data;
        const bool open = (request->wValue & 1U) != 0U;
        if (host_dtr != open) { reset_rx(); }
        host_dtr = open;
    } else if (command == CDC_GET_LINE_CODING) {
        if (data == NULL || length != 7U) { return USBD_FAIL; }
        /* Nominal 1M / 8N1. CDC baud is metadata, NOT the USB link bitrate. */
        const uint8_t coding[7] = {0x40U, 0x42U, 0x0FU, 0U, 0U, 0U, 8U};
        memcpy(data, coding, sizeof(coding));
    } else if (command == CDC_SET_LINE_CODING && (data == NULL || length != 7U)) {
        return USBD_FAIL;
    }
    return USBD_OK;
}

static int8_t cdc_receive(uint8_t *data, uint32_t *length)
{
    if (data == NULL || length == NULL || *length > sizeof(rx_buffer)) {
        ++diagnostics.errors;
        return USBD_FAIL;
    }
    const uint16_t free_bytes = (uint16_t)((rx_tail - rx_head - 1U) & 255U);
    if (!host_dtr || *length > free_bytes) {
        diagnostics.rx_discarded += *length + ((rx_head - rx_tail) & 255U);
        reset_rx();
    } else {
        const uint32_t at = HAL_GetTick();
        for (uint32_t i = 0U; i < *length; ++i) {
            rx_ring[rx_head] = data[i];
            rx_time[rx_head] = at;
            rx_head = (uint16_t)((rx_head + 1U) & 255U);
        }
    }
    return (int8_t)USBD_CDC_ReceivePacket(&device);
}

BSP_USB_Status_t BSP_USB_Read(uint8_t *byte, uint32_t *at_ms, uint32_t *epoch)
{
    if (byte == NULL || at_ms == NULL || epoch == NULL || __get_IPSR() != 0U) {
        return BSP_USB_INVALID_ARG;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *epoch = rx_epoch;
    const bool ready = host_dtr && rx_tail != rx_head;
    if (ready) {
        *byte = rx_ring[rx_tail];
        *at_ms = rx_time[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & 255U);
    }
    __set_PRIMASK(mask);
    return ready ? BSP_USB_OK : BSP_USB_NOT_READY;
}

uint32_t BSP_USB_RxEpoch(void) { return rx_epoch; }

static int8_t cdc_complete(uint8_t *data, uint32_t *length, uint8_t endpoint)
{
    (void)data; (void)length; (void)endpoint;
    ++diagnostics.completed;
    return USBD_OK;
}

static USBD_CDC_ItfTypeDef interface = {
    .Init = cdc_init, .DeInit = cdc_deinit, .Control = cdc_control,
    .Receive = cdc_receive, .TransmitCplt = cdc_complete
};

BSP_USB_Status_t BSP_USB_Init(void)
{
    if (__get_IPSR() != 0U) { return BSP_USB_INVALID_ARG; }
    if (attempted) { return BSP_USB_BUSY; }
    attempted = true;
    diagnostics.init_status = BSP_USB_ERROR;
    if (USBD_Init(&device, &HS_Desc, DEVICE_HS) != USBD_OK) { return BSP_USB_ERROR; }
    if (USBD_RegisterClass(&device, &USBD_CDC) != USBD_OK ||
        USBD_CDC_RegisterInterface(&device, &interface) != USBD_OK ||
        USBD_Start(&device) != USBD_OK) {
        (void)USBD_DeInit(&device);
        return BSP_USB_ERROR;
    }
    started = true;
    diagnostics.init_status = BSP_USB_OK;
    return BSP_USB_OK;
}

BSP_USB_Status_t BSP_USB_Send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > sizeof(tx_buffer) ||
        __get_IPSR() != 0U) { return BSP_USB_INVALID_ARG; }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    BSP_USB_Status_t result;
    USBD_CDC_HandleTypeDef *cdc = device.pClassDataCmsit[device.classId];
    if (!started || device.dev_state != USBD_STATE_CONFIGURED || cdc == NULL || !host_dtr) {
        ++diagnostics.not_ready;
        result = BSP_USB_NOT_READY;
    } else if (cdc->TxState != 0U) {
        ++diagnostics.busy;
        result = BSP_USB_BUSY;
    } else {
        /* Bounded copy plus register submission. USB deinit/completion cannot
         * race the class pointer or this buffer; no wait/callback in this path. */
        memcpy(tx_buffer, data, length);
        if (USBD_CDC_SetTxBuffer(&device, tx_buffer, length) == USBD_OK &&
            USBD_CDC_TransmitPacket(&device) == USBD_OK) {
            ++diagnostics.submitted;
            result = BSP_USB_OK;
        } else {
            ++diagnostics.errors;
            result = BSP_USB_ERROR;
        }
    }
    __set_PRIMASK(mask);
    return result;
}

BSP_USB_Status_t BSP_USB_GetDiagnostics(BSP_USB_Diagnostics_t *out)
{
    if (out == NULL || __get_IPSR() != 0U) { return BSP_USB_INVALID_ARG; }
    if (!attempted) { return BSP_USB_NOT_READY; }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = diagnostics;
    out->configured = started && device.dev_state == USBD_STATE_CONFIGURED;
    out->dtr = host_dtr;
    const USBD_CDC_HandleTypeDef *cdc = device.pClassDataCmsit[device.classId];
    out->tx_busy = cdc != NULL && cdc->TxState != 0U;
    __set_PRIMASK(mask);
    return BSP_USB_OK;
}

void BSP_USB_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }
