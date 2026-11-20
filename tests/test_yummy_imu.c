#include "yummy_imu.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t test_ipsr, test_primask, test_tick;
static UART_HandleTypeDef port;
static BSP_UART_RxCallback_t rx;
static void *rx_context;
static BSP_UART_Status_t uart_status = {.rx_running = true, .rx_rearm = HAL_OK};
static unsigned init_calls;

uint32_t HAL_GetTick(void) { return test_tick; }
HAL_StatusTypeDef BSP_UART_Init(UART_HandleTypeDef *uart,
                                BSP_UART_RxCallback_t callback, void *context)
{
    assert(uart == &port && test_ipsr == 0U);
    ++init_calls; rx = callback; rx_context = context;
    return HAL_OK;
}
HAL_StatusTypeDef BSP_UART_GetStatus(UART_HandleTypeDef *uart, BSP_UART_Status_t *status)
{
    assert(uart == &port && test_ipsr == 0U);
    *status = uart_status;
    return HAL_OK;
}
static void feed(const void *data, unsigned length)
{
    assert(length <= BSP_UART_RX_CAPACITY);
    uint8_t temporary[BSP_UART_RX_CAPACITY];
    memcpy(temporary, data, length);
    test_ipsr = 16U;
    rx(&port, temporary, (uint16_t)length, rx_context);
    test_ipsr = 0U;
    memset(temporary, 0xCC, sizeof(temporary)); /* receiver must copy, not borrow */
}
static YummyIMU_Status_t status(void)
{
    YummyIMU_Status_t s = {0};
    YummyIMU_GetStatus(&s);
    return s;
}
static void binary(uint8_t frame[8], int16_t pitch, int16_t roll, int16_t yaw)
{
    const int16_t values[3] = {pitch, roll, yaw};
    frame[0] = 0xA5U; frame[7] = 0U;
    for (unsigned i = 0U; i < 3U; ++i) {
        frame[1U + 2U*i] = (uint8_t)values[i];
        frame[2U + 2U*i] = (uint8_t)((uint16_t)values[i] >> 8);
    }
    for (unsigned i = 0U; i < 7U; ++i) { frame[7] = (uint8_t)(frame[7] + frame[i]); }
}
int main(int argc, char **argv)
{
    (void)argv;
    const bool use_binary = argc > 1;
    assert(!status().rx_running && status().sequence == 0U);
    assert(YummyIMU_Init(NULL, YUMMY_IMU_USE) == HAL_ERROR);
    assert(YummyIMU_Init(&port, (YummyIMU_Format_t)2) == HAL_ERROR);
    assert(YummyIMU_Init(&port, use_binary ? YUMMY_IMU_BINARY : YUMMY_IMU_USE) == HAL_OK);
    assert(YummyIMU_Init(&port, YUMMY_IMU_USE) == HAL_BUSY && init_calls == 1U);
    assert(status().rx_running && status().sequence == 0U);
    test_tick = UINT32_MAX - 2U;
    if (use_binary) {
        uint8_t frame[8];
        binary(frame, 1234, -4567, 18000);
        feed("noise", 5U);
        feed(frame, 3U); YummyIMU_Run();
        assert(status().sequence == 0U);
        feed(frame + 3U, 5U);
        test_ipsr = 16U; YummyIMU_Run(); test_ipsr = 0U;
        assert(status().sequence == 0U);
        YummyIMU_Run();
        assert(status().sequence == 1U && fabsf(status().roll_deg + 45.67f) < 0.001f);
        assert(status().yaw_deg == 180.0f);
        ++frame[7]; feed(frame, 8U); YummyIMU_Run();
        assert(status().sequence == 1U && status().bad_frames == 1U);
        binary(frame, 20000, 0, 0); feed(frame, 8U); YummyIMU_Run();
        assert(status().sequence == 1U && status().bad_frames == 2U);
        binary(frame, -18000, 0, 17999); feed(frame, 8U); YummyIMU_Run();
        assert(status().sequence == 2U && status().pitch_deg == -180.0f);
    } else {
        feed("12.340,-45", 10U); YummyIMU_Run();
        assert(status().sequence == 0U);
        feed(".670,180.000\r\n", 14U); YummyIMU_Run();
        assert(status().sequence == 1U && fabsf(status().roll_deg + 45.67f) < 0.001f);
        assert(status().yaw_deg == 180.0f && status().received_at_ms == test_tick);
        const char *invalid[] = {"nan,0,0\n", "inf,0,0\n", "181,0,0\n",
            "1,2\n", "1,2,3,4\n", "1,2,3,4,5,6,7,8,9\n", "OK\n", "1,2,3x\n",
            "1e2,0,0\n", ",0,0\n", "1,2,\n", "999999999999999999,0,0\n"};
        for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
            feed(invalid[i], (unsigned)strlen(invalid[i])); YummyIMU_Run();
            assert(status().sequence == 1U);
        }
        uint8_t huge[BSP_UART_RX_CAPACITY]; memset(huge, '1', sizeof(huge));
        feed(huge, sizeof(huge)); YummyIMU_Run();
        feed(",2,3\n", 5U); YummyIMU_Run();
        assert(status().sequence == 1U);
        uint8_t nul[] = {'1', ',', '2', ',', '3', 0, '\n'};
        feed(nul, sizeof(nul)); YummyIMU_Run();
        assert(status().sequence == 1U);
        feed(" +.50, -0.25,179.999\n", 21U); YummyIMU_Run();
        assert(status().sequence == 2U && status().pitch_deg == 0.5f);
    }
    assert(status().sequence == 2U && status().received_at_ms == test_tick);
    uint8_t noise[BSP_UART_RX_CAPACITY]; memset(noise, 0xCC, sizeof(noise));
    for (unsigned i = 0U; i < 9U; ++i) { feed(noise, sizeof(noise)); }
    YummyIMU_Run();
    assert(status().overflows == 1U && status().sequence == 2U);
    test_tick = 3U; /* rollover is a normal timestamp, not a new sensor sequence */
    if (use_binary) {
        uint8_t frame[8]; binary(frame, 1, 2, 3); feed(frame, sizeof(frame));
    } else { feed("\n1,2,3\n", 7U); }
    YummyIMU_Run();
    assert(status().sequence == 3U && status().received_at_ms == 3U);
    uart_status.rx_error = HAL_UART_ERROR_FE;
    assert(status().rx_running && status().uart_error == HAL_UART_ERROR_FE);
    assert(status().sequence == 3U && status().received_at_ms == 3U);
    uart_status.rx_rearm = HAL_ERROR;
    assert(!status().rx_running && status().uart_error == HAL_UART_ERROR_FE);
    uart_status.rx_rearm = HAL_OK;
    assert(status().rx_running); /* Resumed RX does not erase historical errors. */
    uart_status.rx_running = false; uart_status.rx_error = HAL_UART_ERROR_ORE;
    assert(!status().rx_running && status().uart_error == HAL_UART_ERROR_ORE);
    for (unsigned i = 0U; i < 20U; ++i) { YummyIMU_Run(); }
    assert(init_calls == 1U && !status().rx_running); /* no automatic restart */
    assert(test_primask == 0U && status().rx_bytes > 1024U);
    puts("YummyIMU UART copy/fragment/range/checksum/overflow/resync/wrap/error: PASS");
    return 0;
}
