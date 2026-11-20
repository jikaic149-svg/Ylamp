#include "bmi088.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef test_gpioa, test_gpioc;
uint32_t test_ipsr, test_primask, test_tick, test_spi2_instance;
SPI_HandleTypeDef hspi2;
static uint8_t registers[2][128];
static const uint8_t *borrowed_tx;
static uint8_t *borrowed_rx;
static uint16_t borrowed_size;
static unsigned transfers, starts, delays;
static bool pending;
static uint32_t last_power_write[2], power_settle_ms[2];

uint32_t HAL_GetTick(void) { return test_tick; }
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t peripheral)
{
    assert(peripheral == RCC_PERIPHCLK_SPI2);
    return 80000000U;
}
void HAL_Delay(uint32_t ms) { ++delays; test_tick += ms; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) { port->ODR |= pin; }
    else { port->ODR &= ~(uint32_t)pin; }
}
static void exchange(const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    const bool accel = !(test_gpioc.ODR & BMI088_ACCEL__SPI_CS_Pin);
    const bool gyro = !(test_gpioc.ODR & BMI088_GYRO__SPI_CS_Pin);
    assert(accel != gyro);
    const unsigned chip = accel ? 0U : 1U;
    assert(test_tick - last_power_write[chip] >= power_settle_ms[chip]);
    ++transfers;
    memset(rx, 0, size);
    if (tx[0] & 0x80U) {
        const unsigned dummy = accel ? 2U : 1U;
        assert(size > dummy);
        for (unsigned i = dummy; i < size; ++i) {
            rx[i] = registers[chip][(tx[0] & 0x7FU) + i - dummy];
        }
    } else {
        assert(size == 2U);
        registers[chip][tx[0]] = tx[1];
        if ((accel && (tx[0] == 0x7CU || tx[0] == 0x7DU)) ||
            (gyro && tx[0] == 0x11U)) {
            last_power_write[chip] = test_tick;
            power_settle_ms[chip] = accel ? 5U : 30U;
        }
    }
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout)
{
    assert(spi == &hspi2 && timeout == 2U && !pending);
    exchange(tx, rx, size);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t size)
{
    assert(spi == &hspi2 && !pending);
    borrowed_tx = tx; borrowed_rx = rx; borrowed_size = size;
    pending = true;
    ++starts;
    return HAL_OK;
}
static void finish(void)
{
    assert(pending);
    exchange(borrowed_tx, borrowed_rx, borrowed_size);
    pending = false;
    test_ipsr = 16U;
    HAL_SPI_TxRxCpltCallback(&hspi2);
    test_ipsr = 0U;
}
static void ready(void)
{
    BMI088_Port_NotifyReady(BMI088_ACCEL__INTERRUPT_Pin);
    BMI088_Port_NotifyReady(BMI088_GYRO__INTERRUPT_Pin);
}
static void run(uint32_t now)
{
    test_tick = now;
    BMI088_Run(now);
}
static void power_change(uint32_t now, bool enabled)
{
    const unsigned before_starts = starts, before_delays = delays;
    test_tick = now;
    assert(BMI088_SetEnabled(enabled, now) == (enabled ? HAL_OK : HAL_BUSY));
    assert(test_tick == now && BMI088_GetSample(&(BMI088_Sample_t){0}) == HAL_ERROR);
    for (unsigned i = 0U; i < 60U; ++i) {
        run(now + i);
        if (pending) { finish(); }
    }
    assert(delays == before_delays && starts == before_starts + 6U);
    assert(BMI088_IsSuspended() == !enabled);
    assert(BMI088_SetEnabled(enabled, test_tick) == HAL_OK);
    assert(registers[0][0x7D] == (enabled ? 4U : 0U));
    assert(registers[1][0x11] == (enabled ? 0U : 0x80U));
}
static void acquire(uint32_t now)
{
    ready();
    run(now);
    finish();
    run(now + 1U);
    finish();
    run(now + 2U);
}

int main(int argc, char **argv)
{
    hspi2 = (SPI_HandleTypeDef){.Instance = SPI2, .State = HAL_SPI_STATE_READY,
        .Init = {.Mode = SPI_MODE_MASTER, .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_8BIT, .FirstBit = SPI_FIRSTBIT_MSB,
        .CLKPolarity = SPI_POLARITY_HIGH, .CLKPhase = SPI_PHASE_2EDGE,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8}};
    registers[0][0] = 0x1E; registers[1][0] = 0x0F;
    assert(BMI088_Port_Init() == HAL_OK);
    assert(BMI088_Init() == HAL_OK);
    assert(BMI088_IsSuspended());
    assert(registers[0][0x7D] == 0 && registers[1][0x11] == 0x80);
    if (argc == 2) {
        const unsigned before_delays = delays;
        test_tick = 1000U;
        assert(BMI088_SetEnabled(true, test_tick) == HAL_OK);
        run(1000U);
        assert(pending);
        if (strcmp(argv[1], "power-timeout") == 0) {
            const uint8_t *const held_tx = borrowed_tx;
            run(1006U);
            assert(BMI088_SetEnabled(false, test_tick) == HAL_ERROR);
            finish(); run(1007U);
            assert(borrowed_tx == held_tx && starts == 1U);
        } else {
            assert(strcmp(argv[1], "power-readback") == 0);
            finish();
            registers[1][0x11] = 0x01U; /* rejected hardware readback */
            for (unsigned i = 1U; i < 60U; ++i) {
                run(1000U + i);
                if (pending) { finish(); }
            }
            assert(starts == 2U); /* no later writes/retry after failed verify */
        }
        assert(delays == before_delays && !BMI088_IsSuspended());
        assert(BMI088_GetSample(&(BMI088_Sample_t){0}) == HAL_ERROR);
        assert(BMI088_SetEnabled(true, test_tick) == HAL_ERROR);
        puts("PASS: asynchronous power failure stays latched, buffers not reused.");
        return 0;
    }
    BMI088_Sample_t sample;
    assert(BMI088_GetSample(&sample) == HAL_ERROR);
    const unsigned initialized_transfers = transfers;
    ready();
    run(1000U);
    assert(transfers == initialized_transfers && starts == 0U);
    power_change(1000U, true);
    registers[0][0x12] = 0; registers[0][0x13] = 0x80; /* -32768 */
    registers[0][0x14] = 0xAB; registers[0][0x15] = 0x2A; /* 10923, ~1g */
    registers[0][0x18] = 1;
    registers[1][0x02] = 0; registers[1][0x03] = 0x40; /* 16384, 1000dps */
    const unsigned power_starts = starts;
    ready(); run(1100U);
    assert(starts == power_starts); /* warmup after power readback, not request */
    acquire(1200U);
    assert(BMI088_GetSample(&sample) == HAL_OK && sample.sequence == 1U);
    assert(fabsf(sample.accel_m_s2[0] + 3.0f * 9.80665f) < 1e-5f);
    assert(fabsf(sample.accel_m_s2[1] - 9.80665f) < 0.001f);
    assert(fabsf(sample.gyro_rad_s[0] - 17.4532925f) < 1e-5f);
    acquire(1210U); /* frozen sensor time cannot publish a fake fresh sample */
    assert(BMI088_GetSample(&sample) == HAL_OK && sample.sequence == 1U);
    registers[0][0x18] = 2;
    acquire(1220U);
    assert(BMI088_GetSample(&sample) == HAL_OK && sample.sequence == 2U);
    ready(); run(1230U);
    assert(BMI088_SetEnabled(false, 1231U) == HAL_BUSY);
    finish(); run(1231U);
    finish(); run(1232U);
    power_change(1233U, false);
    const unsigned off_starts = starts;
    ready(); run(2000U);
    assert(starts == off_starts && BMI088_GetSample(&sample) == HAL_ERROR);
    power_change(2100U, true);
    ready(); run(2300U);
    run(2306U); /* timeout never aborts/reuses a still-borrowed buffer */
    const unsigned failed_starts = starts;
    finish(); run(2310U);
    assert(BMI088_GetSample(&sample) == HAL_ERROR && starts == failed_starts);
    assert(BMI088_SetEnabled(true, 2400U) == HAL_ERROR);
    puts("PASS: BMI088 IDs/dummy bytes/scales, default suspend, explicit session,"
         " nonblocking power-write settling/readback, fresh sensor time, drain guard,"
         " timeout latch and no auto-recovery.");
}
