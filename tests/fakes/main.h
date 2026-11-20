#ifndef BSP_TEST_MAIN_H
#define BSP_TEST_MAIN_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct {uint32_t ODR, IDR;} GPIO_TypeDef;
extern GPIO_TypeDef test_gpioa, test_gpioc;
#define GPIOA (&test_gpioa)
#define GPIOC (&test_gpioc)
#define DC24_0__OUTPUT_Pin (1U << 14)
#define DC24_1__OUTPUT_Pin (1U << 13)
#define DC5__OUTPUT_Pin (1U << 15)
#define KEY__INPUT_Pin (1U << 15)
#define KEY__INPUT_GPIO_Port GPIOA
#define BMI088_ACCEL__SPI_CS_Pin (1U << 0)
#define BMI088_GYRO__SPI_CS_Pin (1U << 3)
#define BMI088_ACCEL__INTERRUPT_Pin (1U << 10)
#define BMI088_GYRO__INTERRUPT_Pin (1U << 12)

extern uint32_t test_ipsr, test_primask, test_tick;
static inline uint32_t __get_IPSR(void) { return test_ipsr; }
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) { test_primask = mask; }
static inline void __DMB(void) {}
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

typedef struct {uint32_t CR3;} USART_TypeDef;
typedef struct {
    struct {uint32_t Mode, Direction;} Init;
    uint32_t remaining;
} DMA_HandleTypeDef;
#define DMA_CIRCULAR 1U
#define DMA_NORMAL 0U
#define DMA_PERIPH_TO_MEMORY 0U
#define DMA_MEMORY_TO_PERIPH 1U
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
typedef struct {uint32_t WordLength, Parity, Mode, BaudRate, StopBits;} UART_InitTypeDef;
typedef struct {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    uint32_t gState, RxState, ErrorCode;
    DMA_HandleTypeDef *hdmarx, *hdmatx;
} UART_HandleTypeDef;
#define UART_WORDLENGTH_8B 8U
#define UART_WORDLENGTH_9B 9U
#define UART_PARITY_NONE 0U
#define UART_STOPBITS_1 1U
#define UART_PARITY_EVEN 1U
#define UART_MODE_RX 1U
#define UART_MODE_TX 2U
#define UART_MODE_TX_RX 3U
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
#define HAL_UART_STATE_BUSY_RX 2U
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_FE 4U
#define HAL_UART_ERROR_ORE 8U
#define USART_CR3_DMAR (1U << 6)
extern UART_HandleTypeDef huart1, huart2, huart3, huart4, huart5;
extern UART_HandleTypeDef huart6, huart7, huart8, huart9, huart10;
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, const uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

typedef struct {
    uint32_t Mode, Direction, DataSize, FirstBit, CLKPolarity, CLKPhase;
    uint32_t BaudRatePrescaler;
} SPI_InitTypeDef;
typedef struct {void *Instance; SPI_InitTypeDef Init; uint32_t State;} SPI_HandleTypeDef;
extern uint32_t test_spi6_instance, test_spi2_instance;
extern SPI_HandleTypeDef hspi6, hspi2;
#define SPI2 (&test_spi2_instance)
#define SPI6 (&test_spi6_instance)
#define HAL_SPI_STATE_READY 0U
#define HAL_SPI_STATE_BUSY_TX 1U
#define SPI_MODE_MASTER 1U
#define SPI_DIRECTION_2LINES_TXONLY 1U
#define SPI_DIRECTION_2LINES 2U
#define SPI_DATASIZE_8BIT 8U
#define SPI_FIRSTBIT_MSB 1U
#define SPI_POLARITY_LOW 0U
#define SPI_POLARITY_HIGH 1U
#define SPI_PHASE_1EDGE 0U
#define SPI_PHASE_2EDGE 1U
#define SPI_BAUDRATEPRESCALER_8 8U
#define RCC_PERIPHCLK_SPI6 6U
#define RCC_PERIPHCLK_SPI2 2U
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, const uint8_t *,
                                  uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, const uint8_t *,
                                         uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *, const uint8_t *,
                                            uint8_t *, uint16_t);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);

typedef struct {
    uint32_t FrameFormat, Mode, StdFiltersNbr, RxFifo0ElmtsNbr;
    uint32_t TxFifoQueueElmtsNbr;
    uint32_t AutoRetransmission, TransmitPause, ProtocolException;
    uint32_t NominalPrescaler, NominalSyncJumpWidth, NominalTimeSeg1, NominalTimeSeg2;
    uint32_t DataPrescaler, DataSyncJumpWidth, DataTimeSeg1, DataTimeSeg2;
    uint32_t MessageRAMOffset, ExtFiltersNbr, RxFifo0ElmtSize;
    uint32_t RxFifo1ElmtsNbr, RxFifo1ElmtSize, RxBuffersNbr, RxBufferSize;
    uint32_t TxEventsNbr, TxBuffersNbr, TxFifoQueueMode, TxElmtSize;
} FDCAN_InitTypeDef;
typedef struct {uint32_t IR, IE, TXBRP, CCCR;} FDCAN_GlobalTypeDef;
extern FDCAN_GlobalTypeDef test_fdcan3_instance;
#define FDCAN3 (&test_fdcan3_instance)
#define DISABLE 0U
typedef struct {
    FDCAN_GlobalTypeDef *Instance;
    FDCAN_InitTypeDef Init;
    uint32_t State;
} FDCAN_HandleTypeDef;
typedef struct {
    uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2;
} FDCAN_FilterTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat;
} FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t BusOff, LastErrorCode, DataLastErrorCode, ErrorPassive, Warning;
} FDCAN_ProtocolStatusTypeDef;
typedef struct {
    uint32_t TxErrorCnt, RxErrorCnt, RxErrorPassive, ErrorLogging;
} FDCAN_ErrorCountersTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
#define HAL_FDCAN_STATE_READY 1U
#define HAL_FDCAN_STATE_BUSY 2U
#define FDCAN_FRAME_CLASSIC 0U
#define FDCAN_FRAME_FD_BRS 2U
#define FDCAN_DATA_BYTES_64 64U
#define FDCAN_TX_FIFO_OPERATION 0U
#define FDCAN_MODE_NORMAL 0U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID 1U
#define FDCAN_FILTER_MASK 1U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_REJECT 2U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_ERROR_PASSIVE (1U << 23)
#define FDCAN_IT_ERROR_WARNING (1U << 24)
#define FDCAN_IT_BUS_OFF (1U << 25)
#define FDCAN_IT_ARB_PROTOCOL_ERROR (1U << 27)
#define FDCAN_IT_DATA_PROTOCOL_ERROR (1U << 28)
#define FDCAN_CCCR_INIT 1U
#define FDCAN_CCCR_CCE (1U << 1)
#define FDCAN_CCCR_CSA (1U << 3)
#define FDCAN_CCCR_CSR (1U << 4)
#define FDCAN_DATA_FRAME 0U
#define FDCAN_REMOTE_FRAME 1U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_BRS_ON 1U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_RX_FIFO0 0U
/* 与当前 STM32H7 HAL 一致：DLC 宏是原始码值，并非 <<16 后的寄存器字段。 */
#define FDCAN_DLC_BYTES_0 0U
#define FDCAN_DLC_BYTES_1 1U
#define FDCAN_DLC_BYTES_2 2U
#define FDCAN_DLC_BYTES_3 3U
#define FDCAN_DLC_BYTES_4 4U
#define FDCAN_DLC_BYTES_5 5U
#define FDCAN_DLC_BYTES_6 6U
#define FDCAN_DLC_BYTES_7 7U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_DLC_BYTES_12 9U
#define FDCAN_DLC_BYTES_16 10U
#define FDCAN_DLC_BYTES_20 11U
#define FDCAN_DLC_BYTES_24 12U
#define FDCAN_DLC_BYTES_32 13U
#define FDCAN_DLC_BYTES_48 14U
#define FDCAN_DLC_BYTES_64 15U
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, const FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t,
                                              uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_DeactivateNotification(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *,
                                            FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(const FDCAN_HandleTypeDef *,
                                             FDCAN_ErrorCountersTypeDef *);
void HAL_FDCAN_IRQHandler(FDCAN_HandleTypeDef *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *,
                                              const FDCAN_TxHeaderTypeDef *,
                                              const uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(const FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t,
                                       FDCAN_RxHeaderTypeDef *, uint8_t *);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);

#ifdef __cplusplus
}
#endif
#endif
