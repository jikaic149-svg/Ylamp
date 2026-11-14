#include "bsp_can.h"

/* Board02: PLL1Q=120MHz. Keep this override in the project, not the external
 * Cube SDK. MSP owns the shared clock reference count, GPIO AF and IRQs.
 * Message RAM: filter 1 + RX16*(2+16) + TX8*(2+16) = 433 words.
 * [1706,2139) is inside the 2560-word RAM and disjoint from CAN1/2. */
BSP_CAN_Status_t BSP_CAN3_InitFD(void)
{
    static uint8_t attempted;
    if (__get_IPSR() != 0U) { return BSP_CAN_INVALID_ARG; }
    if (attempted) { return BSP_CAN_BUSY; }
    attempted = 1U;
    hfdcan3.Instance = FDCAN3;
    hfdcan3.Init = (FDCAN_InitTypeDef){
        .FrameFormat = FDCAN_FRAME_FD_BRS, .Mode = FDCAN_MODE_NORMAL,
        .AutoRetransmission = DISABLE, .TransmitPause = DISABLE,
        .ProtocolException = DISABLE,
        .NominalPrescaler = 3U, .NominalSyncJumpWidth = 10U,
        .NominalTimeSeg1 = 29U, .NominalTimeSeg2 = 10U,
        .DataPrescaler = 3U, .DataSyncJumpWidth = 5U,
        .DataTimeSeg1 = 14U, .DataTimeSeg2 = 5U,
        .MessageRAMOffset = 1706U, .StdFiltersNbr = 1U,
        .RxFifo0ElmtsNbr = 16U, .RxFifo0ElmtSize = FDCAN_DATA_BYTES_64,
        .RxFifo1ElmtSize = FDCAN_DATA_BYTES_64,
        .RxBufferSize = FDCAN_DATA_BYTES_64,
        .TxFifoQueueElmtsNbr = 8U, .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION,
        .TxElmtSize = FDCAN_DATA_BYTES_64
    };
    if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK) { return BSP_CAN_ERROR; }
    return BSP_CAN_Init(&hfdcan3);
}
