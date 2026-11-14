#include "usb_telemetry.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "motor_usb.h"
#include "motor_tuning.h"
#include "motor_telemetry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t test_ipsr, test_primask, test_tick;
PCD_HandleTypeDef hpcd_USB_OTG_HS;
USBD_ClassTypeDef USBD_CDC;
USBD_DescriptorsTypeDef HS_Desc;
static USBD_HandleTypeDef *usb;
static USBD_CDC_HandleTypeDef cdc;
static USBD_CDC_ItfTypeDef *interface;
static unsigned failure_stage, cleanup, sends, reads, rearms, irqs;
static bool tx_error, set_buffer_error, advance_tick;
static uint8_t tokens[MOTOR_SNAPSHOT_MAX_JOINTS];
static DM_Motor_Feedback_t samples[MOTOR_SNAPSHOT_MAX_JOINTS];
static bool received[MOTOR_SNAPSHOT_MAX_JOINTS];
static unsigned rpc_calls, actions;

BSP_CAN_Status_t MotorTuning_HandleRequest(const uint8_t request[8],
    uint32_t arrived_ms, uint32_t now_ms, uint8_t response[8])
{
    assert(test_ipsr == 0U && test_primask == 0U);
    ++rpc_calls;
    memcpy(response, request, 8U);
    response[1] = now_ms - arrived_ms > 500U ? BSP_CAN_NOT_READY : BSP_CAN_OK;
    if (response[1] == BSP_CAN_OK && (request[1] == 7U ||
        (request[1] == 12U && request[3] == 8U))) { ++actions; }
    return BSP_CAN_OK;
}

uint32_t HAL_GetTick(void) { return test_tick; }
USBD_StatusTypeDef USBD_Init(USBD_HandleTypeDef *dev, USBD_DescriptorsTypeDef *desc, uint8_t id)
{
    assert(desc == &HS_Desc && id == DEVICE_HS);
    usb = dev;
    dev->dev_state = USBD_STATE_DEFAULT;
    return failure_stage == 1U ? USBD_FAIL : USBD_OK;
}
USBD_StatusTypeDef USBD_RegisterClass(USBD_HandleTypeDef *dev, USBD_ClassTypeDef *cls)
{
    assert(dev == usb && cls == &USBD_CDC);
    return failure_stage == 2U ? USBD_FAIL : USBD_OK;
}
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *dev, USBD_CDC_ItfTypeDef *itf)
{
    assert(dev == usb); interface = itf;
    return failure_stage == 3U ? USBD_FAIL : USBD_OK;
}
USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *dev)
{
    assert(dev == usb);
    return failure_stage == 4U ? USBD_FAIL : USBD_OK;
}
USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *dev)
{
    assert(dev == usb); ++cleanup;
    if (dev->pClassDataCmsit[0] != NULL) { assert(interface->DeInit() == USBD_OK); }
    dev->dev_state = USBD_STATE_DEFAULT; dev->pClassDataCmsit[0] = NULL;
    return USBD_OK;
}
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *dev, uint8_t *data, uint32_t length)
{
    assert(dev == usb);
    if (set_buffer_error) { return USBD_FAIL; }
    cdc.TxBuffer = data; cdc.TxLength = length; return USBD_OK;
}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *dev, uint8_t *data)
{
    assert(dev == usb); cdc.RxBuffer = data; return USBD_OK;
}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *dev)
{
    assert(dev == usb && test_primask == 1U && cdc.TxState == 0U);
    if (tx_error) { return USBD_FAIL; }
    cdc.TxState = 1U; ++sends; return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *dev)
{
    assert(dev == usb); ++rearms; return USBD_OK;
}
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *pcd)
{
    assert(pcd == &hpcd_USB_OTG_HS); ++irqs;
}
BSP_CAN_Status_t DM_Motor_GetFeedback(const DM_Motor_t *motor, DM_Motor_Feedback_t *out)
{
    assert(test_ipsr == 0U); ++reads;
    for (unsigned i = 0U; i < MOTOR_SNAPSHOT_MAX_JOINTS; ++i) {
        if (motor == (DM_Motor_t *)&tokens[i]) {
            if (advance_tick) { samples[i].received_at_ms = ++test_tick; }
            if (!received[i]) { return BSP_CAN_NOT_READY; }
            *out = samples[i]; return BSP_CAN_OK;
        }
    }
    assert(false); return BSP_CAN_INVALID_ARG;
}

static void enumerate(void)
{
    memset(&cdc, 0, sizeof(cdc));
    usb->pClassDataCmsit[0] = &cdc; usb->dev_state = USBD_STATE_CONFIGURED;
    test_ipsr = 1U;
    assert(interface->Init() == USBD_OK);
    test_ipsr = 0U;
}
static void dtr(bool open)
{
    USBD_SetupReqTypedef req = {.wValue = open ? 1U : 0U};
    test_ipsr = 1U;
    assert(interface->Control(CDC_SET_CONTROL_LINE_STATE, (uint8_t *)&req, 0U) == USBD_OK);
    test_ipsr = 0U;
}
static void complete(void)
{
    cdc.TxState = 0U;
    test_ipsr = 1U;
    assert(interface->TransmitCplt(cdc.TxBuffer, &cdc.TxLength, 1U) == USBD_OK);
    test_ipsr = 0U;
}
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1]<<8 | (uint32_t)p[2]<<16 | (uint32_t)p[3]<<24;
}
static float get_float(const uint8_t *p)
{
    uint32_t bits = get32(p); float value; memcpy(&value, &bits, sizeof(value)); return value;
}

static unsigned make_frame(uint8_t *out, uint8_t type, const uint8_t *p, unsigned length)
{
    out[0] = 0xA5U; out[1] = 0x5AU; out[2] = 1U; out[3] = type; out[4] = (uint8_t)length;
    memcpy(out + 5, p, length);
    const uint16_t crc = MotorTelemetry_CRC16(out + 2, length + 3U);
    out[length + 5U] = (uint8_t)crc; out[length + 6U] = (uint8_t)(crc >> 8);
    return length + 7U;
}
static void input(const uint8_t *p, unsigned length)
{
    uint32_t n = length;
    test_ipsr = 1U;
    assert(interface->Receive((uint8_t *)p, &n) == USBD_OK);
    test_ipsr = 0U;
}
static void service(void) { for (unsigned i = 0U; i < 4U; ++i) { (void)MotorUSB_Run(); } }
static void usb_control_test(void)
{
    uint8_t frame[31], p[24] = {1,2,3,4,5,6,7,8};
    test_tick = 1000U;
    /* Uninitialized/bench endpoint cannot act even on valid traffic. */
    assert(MotorUSB_Run() && actions == 0U);
    MotorUSB_Init();
    unsigned size = make_frame(frame, 0x12U, p, 8U);
    input(frame, 7U); service(); assert(cdc.TxState == 0U);
    input(frame + 7, size - 7U); service();
    assert(cdc.TxLength == 15U && cdc.TxBuffer[3] == 0x92U && rpc_calls == 0U);
    complete();
    p[8] = 1U; p[12] = 0xE8U; p[13] = 3U; /* id=1, tick=1000 */
    p[16] = 1U; p[17] = 12U; p[19] = 8U; /* capture */
    size = make_frame(frame, 0x11U, p, 24U);
    frame[30] ^= 1U; input(frame, size); service(); assert(rpc_calls == 0U);
    frame[30] ^= 1U; input(frame, size); service();
    assert(actions == 1U && cdc.TxBuffer[18] == BSP_CAN_OK);
    uint8_t cached[27]; memcpy(cached, cdc.TxBuffer, 27U); complete();
    input(frame, size); service(); /* identical request: cached result */
    assert(actions == 1U && memcmp(cached, cdc.TxBuffer, 27U) == 0); complete();
    p[19] = 7U; make_frame(frame, 0x11U, p, 24U); input(frame, size); service();
    assert(rpc_calls == 1U && cdc.TxBuffer[18] == BSP_CAN_INVALID_ARG); complete();
    p[8] = 2U; p[17] = 2U; /* configuration writes are NOT USB quick controls */
    make_frame(frame, 0x11U, p, 24U); input(frame, size); service();
    assert(rpc_calls == 1U && cdc.TxBuffer[18] == BSP_CAN_INVALID_ARG); complete();
    /* Busy telemetry owns its bytes. Execute once, defer only the response. */
    const uint8_t telemetry[3] = {'Y','L',1};
    assert(BSP_USB_Send(telemetry, 3U) == BSP_USB_OK);
    p[8] = 3U; p[17] = 7U; p[19] = 3U;
    make_frame(frame, 0x11U, p, 24U); input(frame, size); service();
    assert(actions == 2U && cdc.TxLength == 3U && !MotorUSB_Run());
    complete(); service(); assert(actions == 2U && cdc.TxLength == 27U); complete();
    /* Intent timestamp stays old across retry; never extend to dispatch time. */
    p[8] = 4U; test_tick = 1600U;
    make_frame(frame, 0x11U, p, 24U); input(frame, size); service();
    assert(actions == 2U && cdc.TxBuffer[18] == BSP_CAN_NOT_READY); complete();
    /* Overflow invalidates session + pending input, no automatic handshake. */
    const uint8_t garbage[64] = {0};
    for (unsigned i = 0; i < 5; ++i) { input(garbage, 64U); }
    service();
    p[8] = 5U; p[12] = 0x40U; p[13] = 6U;
    make_frame(frame, 0x11U, p, 24U); input(frame, size); service();
    assert(actions == 2U && cdc.TxBuffer[18] == BSP_CAN_NOT_READY); complete();
    dtr(false); dtr(true);
    input(frame, size); service();
    assert(actions == 2U && cdc.TxBuffer[18] == BSP_CAN_NOT_READY); complete();
    puts("USB quick-control CRC/fragmentation/allowlist/dedup/busy/expiry/overflow/DTR: PASS");
}

int main(int argc, char **argv)
{
    BSP_USB_Diagnostics_t diag;
    USBTelemetry_Status_t status;
    uint8_t data[BSP_USB_TX_CAPACITY] = {0x59U, 0x4CU, 42U};
    assert(BSP_USB_GetDiagnostics(&diag) == BSP_USB_NOT_READY);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY);
    assert(USBTelemetry_GetStatus(&status) == BSP_USB_NOT_READY);
    if (argc > 1 && strcmp(argv[1], "init-failure") == 0) {
        failure_stage = argc > 2 ? (unsigned)atoi(argv[2]) : 4U;
        assert(BSP_USB_Init() == BSP_USB_ERROR);
        assert(BSP_USB_Init() == BSP_USB_BUSY); /* never silently retry */
        assert(BSP_USB_GetDiagnostics(&diag) == BSP_USB_OK);
        assert(diag.init_status == BSP_USB_ERROR && !diag.configured);
        assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY);
        assert(cleanup == (failure_stage == 1U ? 0U : 1U) && sends == 0U);
        puts("USB partial-init failure: PASS"); return 0;
    }
    test_ipsr = 1U;
    assert(BSP_USB_Init() == BSP_USB_INVALID_ARG);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_INVALID_ARG);
    assert(BSP_USB_GetDiagnostics(&diag) == BSP_USB_INVALID_ARG);
    test_ipsr = 0U;
    assert(BSP_USB_Init() == BSP_USB_OK);
    assert(BSP_USB_Init() == BSP_USB_BUSY);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY);
    enumerate();
    assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY); /* DTR closed */
    dtr(true);
    assert(BSP_USB_Send(NULL, 3U) == BSP_USB_INVALID_ARG);
    assert(BSP_USB_Send(data, 0U) == BSP_USB_INVALID_ARG);
    assert(BSP_USB_Send(data, BSP_USB_TX_CAPACITY + 1U) == BSP_USB_INVALID_ARG);
    test_primask = 1U;
    assert(BSP_USB_Send(data, sizeof(data)) == BSP_USB_OK && test_primask == 1U);
    test_primask = 0U;
    assert(cdc.TxBuffer != data && cdc.TxLength == sizeof(data));
    data[2] = 99U;
    assert(cdc.TxBuffer[2] == 42U);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_BUSY && cdc.TxBuffer[2] == 42U);
    assert(BSP_USB_GetDiagnostics(&diag) == BSP_USB_OK && diag.tx_busy);
    dtr(false);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY && cdc.TxState == 1U);
    dtr(true);
    assert(BSP_USB_Send(data, 3U) == BSP_USB_BUSY); /* open does not release ownership */
    complete();
    set_buffer_error = true;
    assert(BSP_USB_Send(data, 3U) == BSP_USB_ERROR);
    set_buffer_error = false; tx_error = true;
    assert(BSP_USB_Send(data, 3U) == BSP_USB_ERROR);
    tx_error = false;
    uint8_t coding[7];
    assert(interface->Control(CDC_GET_LINE_CODING, coding, 7U) == USBD_OK);
    assert(get32(coding) == 1000000U && coding[6] == 8U);
    assert(interface->Control(CDC_GET_LINE_CODING, coding, 6U) == USBD_FAIL);
    assert(interface->Control(CDC_SET_LINE_CODING, coding, 7U) == USBD_OK);
    uint32_t length = CDC_DATA_FS_OUT_PACKET_SIZE;
    assert(interface->Receive(cdc.RxBuffer, &length) == USBD_OK && rearms == 1U);
    uint8_t rx_byte; uint32_t rx_at, epoch;
    for (unsigned i = 0; i < 64U; ++i) {
        assert(BSP_USB_Read(&rx_byte, &rx_at, &epoch) == BSP_USB_OK && rx_at == test_tick);
    }
    assert(BSP_USB_Read(&rx_byte, &rx_at, &epoch) == BSP_USB_NOT_READY);
    BSP_USB_IRQHandler(); assert(irqs == 1U);

    const uint8_t count = argc > 1 && strcmp(argv[1], "six") == 0 ? 6U : 5U;
    USBTelemetry_Source_t input[MOTOR_SNAPSHOT_MAX_JOINTS];
    test_tick = UINT32_MAX - 4U;
    for (unsigned i = 0U; i < count; ++i) {
        input[i] = (USBTelemetry_Source_t){
            .motor = (DM_Motor_t *)&tokens[i], .bus_number = i < 2U ? 2U : 1U,
            .config = {.can_id = (uint16_t)(i+1U), .master_id = (uint16_t)(0x11U+i),
                .protocol = i == 2U ? DM_MOTOR_PROTOCOL_LEGACY_MIT : DM_MOTOR_PROTOCOL_V13}
        };
        samples[i] = (DM_Motor_Feedback_t){
            .position_rad = 12.5f, .velocity_rad_s = -0.002442002296447754f,
            .torque_nm = 0.006839275360107422f, .received_at_ms = test_tick,
            .received_count = 1U, .state = 0U, .mos_temperature_c = 30U,
            .rotor_temperature_c = 31U
        };
        received[i] = i != 4U;
    }
    assert(USBTelemetry_Init(NULL, count, 1U, 100U, test_tick) == BSP_USB_INVALID_ARG);
    assert(USBTelemetry_Init(input, 7U, 1U, 100U, test_tick) == BSP_USB_INVALID_ARG);
    assert(USBTelemetry_Init(input, 0U, 1U, 100U, test_tick) == BSP_USB_INVALID_ARG);
    assert(USBTelemetry_Init(input, count, 0U, 100U, test_tick) == BSP_USB_INVALID_ARG);
    input[0].bus_number = 3U;
    assert(USBTelemetry_Init(input, count, 1U, 100U, test_tick) == BSP_USB_INVALID_ARG);
    input[0].bus_number = 2U;
    assert(USBTelemetry_Init(input, count, 1U, 100U, test_tick) == BSP_USB_OK);
    assert(USBTelemetry_Init(input, count, 1U, 100U, test_tick) == BSP_USB_BUSY);
    input[0].config.can_id = 15U; /* copied, not borrowed */
    USBTelemetry_Run(test_tick);
    assert(cdc.TxLength == count * 48U && reads == count);
    const uint8_t golden[48] = {
        0x59,0x4c,0x01,0x05,0x01,0,0,0,0xfb,0xff,0xff,0xff,0x64,0,0x20,0,
        1,2,0,3,1,0,0x11,0,0xfb,0xff,0xff,0xff,1,0,0,0,
        0,0,0x48,0x41,0,0x0a,0x20,0xbb,0,0x1c,0xe0,0x3b,0,0x1e,0x1f,0
    };
    if (count == 5U) { assert(memcmp(cdc.TxBuffer, golden, 48U) == 0); }
    assert(cdc.TxBuffer[2*48+18] == 1U); /* J3 remains legacy */
    assert(cdc.TxBuffer[4*48+19] == 0U && isnan(get_float(&cdc.TxBuffer[4*48+32])));
    USBTelemetry_Run(test_tick); assert(reads == count);
    test_ipsr = 1U; USBTelemetry_Run(test_tick + 1U); test_ipsr = 0U;
    assert(reads == count);
    uint8_t saved[288]; memcpy(saved, cdc.TxBuffer, cdc.TxLength);
    ++test_tick; USBTelemetry_Run(test_tick); /* busy, preserve in-flight bytes */
    assert(memcmp(saved, cdc.TxBuffer, cdc.TxLength) == 0);
    complete();
    test_tick = 3U; USBTelemetry_Run(test_tick); /* tick wraps; no catch-up */
    assert(cdc.TxBuffer[19] == 3U && get32(cdc.TxBuffer+4) == 3U);
    complete();
    test_tick = 200U; USBTelemetry_Run(test_tick);
    assert(cdc.TxBuffer[19] == 1U && get_float(cdc.TxBuffer+32) == 12.5f); /* stale */
    complete();
    advance_tick = true; USBTelemetry_Run(++test_tick);
    assert(cdc.TxBuffer[19] == 3U); /* sample time after last simulated RX */
    advance_tick = false;
    assert(BSP_USB_GetDiagnostics(&diag) == BSP_USB_OK);
    assert(diag.submitted == sends && diag.completed == sends - 1U);
    assert(diag.rx_discarded == 0U && diag.errors == 2U && diag.configured && diag.dtr);
    assert(USBTelemetry_GetStatus(&status) == BSP_USB_OK);
    assert(status.snapshots == 5U && status.submitted == 4U && status.dropped == 1U);
    assert(USBD_DeInit(usb) == USBD_OK); /* unplug while active; no forced reuse */
    USBTelemetry_Run(++test_tick);
    assert(USBTelemetry_GetStatus(&status) == BSP_USB_OK && status.dropped == 2U);
    enumerate();
    assert(BSP_USB_Send(data, 3U) == BSP_USB_NOT_READY);
    dtr(true);
    USBTelemetry_Run(++test_tick);
    assert(get32(cdc.TxBuffer+4) == 7U && cdc.TxBuffer[15] == 0U);
    complete();
    usb_control_test();
    puts("USB ownership/DTR/backpressure/disconnect/snapshot/wrap/golden: PASS");
    return 0;
}
