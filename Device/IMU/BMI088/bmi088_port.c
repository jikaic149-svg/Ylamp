#include "bmi088_port.h"

static bool initialized;
static volatile bool pending, complete;
static volatile HAL_StatusTypeDef result;
static volatile uint32_t ready_counts[2];
static BMI088_PortChip_t selected;

static void select_chip(BMI088_PortChip_t chip, GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, chip == BMI088_PORT_ACCEL ?
        BMI088_ACCEL__SPI_CS_Pin : BMI088_GYRO__SPI_CS_Pin, state);
}

HAL_StatusTypeDef BMI088_Port_Init(void)
{
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U) { return HAL_ERROR; }
    if (initialized) { return HAL_BUSY; }
    if (hspi2.Instance != SPI2 || hspi2.State != HAL_SPI_STATE_READY ||
        hspi2.Init.Mode != SPI_MODE_MASTER ||
        hspi2.Init.Direction != SPI_DIRECTION_2LINES ||
        hspi2.Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi2.Init.FirstBit != SPI_FIRSTBIT_MSB ||
        hspi2.Init.CLKPolarity != SPI_POLARITY_HIGH ||
        hspi2.Init.CLKPhase != SPI_PHASE_2EDGE ||
        hspi2.Init.BaudRatePrescaler != SPI_BAUDRATEPRESCALER_8) { return HAL_ERROR; }
    const uint32_t clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
    if (clock == 0U || clock / 8U > 10000000U) { return HAL_ERROR; }
    select_chip(BMI088_PORT_ACCEL, GPIO_PIN_SET);
    select_chip(BMI088_PORT_GYRO, GPIO_PIN_SET);
    initialized = true;
    return HAL_OK;
}

static bool valid(BMI088_PortChip_t chip, const uint8_t *tx,
                  uint8_t *rx, uint16_t length)
{
    return initialized && __get_IPSR() == 0U && __get_PRIMASK() == 0U &&
        (chip == BMI088_PORT_ACCEL || chip == BMI088_PORT_GYRO) &&
        tx != NULL && rx != NULL && length >= 2U && length <= 16U;
}

HAL_StatusTypeDef BMI088_Port_Exchange(BMI088_PortChip_t chip, const uint8_t *tx,
                                  uint8_t *rx, uint16_t length)
{
    if (!valid(chip, tx, rx, length)) { return HAL_ERROR; }
    if (pending || hspi2.State != HAL_SPI_STATE_READY) { return HAL_BUSY; }
    select_chip(chip, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi2, tx, rx, length, 2U);
    select_chip(chip, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef BMI088_Port_Start(BMI088_PortChip_t chip, const uint8_t *tx,
                               uint8_t *rx, uint16_t length)
{
    if (!valid(chip, tx, rx, length)) { return HAL_ERROR; }
    if (pending || hspi2.State != HAL_SPI_STATE_READY) { return HAL_BUSY; }
    selected = chip;
    complete = false;
    pending = true;
    result = HAL_BUSY;
    select_chip(chip, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_IT(&hspi2, tx, rx, length);
    if (status != HAL_OK) {
        select_chip(chip, GPIO_PIN_SET);
        pending = false;
    }
    return status;
}

HAL_StatusTypeDef BMI088_Port_TakeResult(void)
{
    if (!initialized || __get_IPSR() != 0U) { return HAL_ERROR; }
    if (!pending) { return HAL_ERROR; }
    if (!complete) { return HAL_BUSY; }
    __DMB();
    pending = false;
    return result;
}

void BMI088_Port_GetReadyCounts(uint32_t counts[2])
{
    if (counts == NULL) { return; }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    counts[0] = ready_counts[0];
    counts[1] = ready_counts[1];
    __set_PRIMASK(mask);
}

void BMI088_Port_NotifyReady(uint16_t pin)
{
    if (pin == BMI088_ACCEL__INTERRUPT_Pin) { ++ready_counts[0]; }
    if (pin == BMI088_GYRO__INTERRUPT_Pin) { ++ready_counts[1]; }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi != &hspi2 || !pending) { return; }
    select_chip(selected, GPIO_PIN_SET);
    result = HAL_OK;
    __DMB();
    complete = true;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi != &hspi2 || !pending) { return; }
    select_chip(selected, GPIO_PIN_SET);
    result = HAL_ERROR;
    __DMB();
    complete = true;
}
