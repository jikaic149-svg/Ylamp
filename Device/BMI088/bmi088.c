#include "bmi088.h"

#include <string.h>

static struct {
    bool initialized, failed, enabled;
    bool power_transition, power_target;
    uint8_t power_step, power_phase;
    uint32_t power_at_ms;
    uint8_t phase;
    uint8_t tx[16], rx[16], accel[9];
    uint32_t started_at, cycle_at, enabled_at, ready[2], used_ready[2], sensor_time;
    bool have_sensor_time;
    BMI088_Sample_t sample;
} imu;

static HAL_StatusTypeDef read_register(BSP_IMU_Chip_t chip, uint8_t address,
                                      uint8_t *value)
{
    uint8_t tx[3] = {(uint8_t)(address | 0x80U), 0U, 0U}, rx[3] = {0};
    const uint16_t dummy = chip == BSP_IMU_ACCEL ? 2U : 1U;
    const HAL_StatusTypeDef status = BSP_IMU_Exchange(chip, tx, rx, dummy + 1U);
    if (status == HAL_OK) { *value = rx[dummy]; }
    return status;
}

static HAL_StatusTypeDef write_register(BSP_IMU_Chip_t chip, uint8_t address,
                                       uint8_t value)
{
    const uint8_t tx[2] = {address, value};
    uint8_t rx[2];
    return BSP_IMU_Exchange(chip, tx, rx, 2U);
}

static bool configure(BSP_IMU_Chip_t chip, uint8_t address, uint8_t value)
{
    uint8_t readback = (uint8_t)~value;
    if (write_register(chip, address, value) != HAL_OK) { return false; }
    /* No access to a chip while its power mode is settling (DS 4.1.1/4.1.2).
     * In particular, waiting after both ACC power writes is too late. */
    HAL_Delay(chip == BSP_IMU_GYRO && address == 0x11U ? 30U : 5U);
    return read_register(chip, address, &readback) == HAL_OK && readback == value;
}

HAL_StatusTypeDef BMI088_Init(void)
{
    if (__get_IPSR() != 0U) { return HAL_ERROR; }
    if (imu.initialized || imu.failed) { return HAL_BUSY; }
    imu.failed = true;
    uint8_t id = 0U;
    /* The first accelerometer read switches its interface to SPI. */
    if (read_register(BSP_IMU_ACCEL, 0x00U, &id) != HAL_OK) { return HAL_ERROR; }
    HAL_Delay(5U);
    if (read_register(BSP_IMU_ACCEL, 0x00U, &id) != HAL_OK || id != 0x1EU ||
        read_register(BSP_IMU_GYRO, 0x00U, &id) != HAL_OK || id != 0x0FU ||
        write_register(BSP_IMU_ACCEL, 0x7EU, 0xB6U) != HAL_OK ||
        write_register(BSP_IMU_GYRO, 0x14U, 0xB6U) != HAL_OK) { return HAL_ERROR; }
    HAL_Delay(100U);
    if (read_register(BSP_IMU_ACCEL, 0x00U, &id) != HAL_OK) { return HAL_ERROR; }
    HAL_Delay(5U);
    if (read_register(BSP_IMU_ACCEL, 0x00U, &id) != HAL_OK || id != 0x1EU ||
        read_register(BSP_IMU_GYRO, 0x00U, &id) != HAL_OK || id != 0x0FU) {
        return HAL_ERROR;
    }
    /* BMI088: normal filtered 100Hz, +/-3g; gyro 100Hz/32Hz, +/-2000dps.
     * Data-ready mapped to PE10/PE12, active high. No FIFO/DMA/heater.
     * Addresses/dummy-byte handling verified against the supplied BMI088 driver. */
    const uint8_t accel[][2] = {
        {0x7DU, 0x04U}, {0x7CU, 0x00U}, {0x40U, 0xA8U},
        {0x41U, 0x00U}, {0x53U, 0x0AU}, {0x58U, 0x04U}
    };
    const uint8_t gyro[][2] = {
        {0x0FU, 0x00U}, {0x10U, 0x87U}, {0x11U, 0x00U},
        {0x16U, 0x0DU}, {0x18U, 0x01U}, {0x15U, 0x80U}
    };
    for (unsigned i = 0; i < sizeof(accel) / sizeof(accel[0]); ++i) {
        if (!configure(BSP_IMU_ACCEL, accel[i][0], accel[i][1])) { return HAL_ERROR; }
    }
    for (unsigned i = 0; i < sizeof(gyro) / sizeof(gyro[0]); ++i) {
        if (!configure(BSP_IMU_GYRO, gyro[i][0], gyro[i][1])) { return HAL_ERROR; }
    }
    BSP_IMU_GetReadyCounts(imu.used_ready);
    imu.initialized = true;
    imu.failed = false;
    /* Boot only, before motor power: synchronous configuration is permitted. */
    if (!configure(BSP_IMU_GYRO, 0x11U, 0x80U) ||
        !configure(BSP_IMU_ACCEL, 0x7CU, 0x03U) ||
        !configure(BSP_IMU_ACCEL, 0x7DU, 0x00U)) {
        imu.failed = true;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef BMI088_SetEnabled(bool enabled, uint32_t now_ms)
{
    if (__get_IPSR() != 0U || !imu.initialized || imu.failed) { return HAL_ERROR; }
    if (imu.power_transition) { return HAL_BUSY; }
    if (enabled == imu.enabled) { return HAL_OK; }
    if (imu.phase != 0U) { return HAL_BUSY; }
    (void)now_ms;
    imu.power_target = enabled;
    imu.power_transition = true;
    imu.power_step = imu.power_phase = 0U;
    imu.have_sensor_time = false;
    if (enabled) { imu.sample.sequence = 0U; }
    /* ON accepts the session, but samples stay unavailable until readback and
     * warmup. OFF reports BUSY until hardware suspension is actually confirmed. */
    return enabled ? HAL_OK : HAL_BUSY;
}

bool BMI088_IsSuspended(void)
{
    return imu.initialized && !imu.failed && !imu.enabled && !imu.power_transition;
}

static bool start_transfer(BSP_IMU_Chip_t chip, uint16_t length, uint32_t now_ms)
{
    if (BSP_IMU_Start(chip, imu.tx, imu.rx, length) != HAL_OK) {
        imu.failed = true;
        return false;
    }
    imu.started_at = now_ms;
    return true;
}

static bool begin(BSP_IMU_Chip_t chip, uint8_t address, uint16_t length,
                  uint32_t now_ms)
{
    memset(imu.tx, 0, sizeof(imu.tx));
    imu.tx[0] = address | 0x80U;
    return start_transfer(chip, length, now_ms);
}

/* Runtime power writes/readbacks use the same borrowed IT buffers as sampling.
 * Never hold the motor-control thread in HAL_Delay for the 30/5/5ms settling. */
static void update_power(uint32_t now_ms)
{
    /* Bosch Sensor API also writes ACC_PWR_CONF before ACC_PWR_CTRL in both
     * directions. Keep the >=5ms separation recommended by the datasheet;
     * a different boot setup order alone is not evidence of a runtime fault. */
    static const uint8_t address[3] = {0x11U, 0x7CU, 0x7DU};
    static const uint8_t value[2][3] = {{0x80U, 0x03U, 0x00U},
                                      {0x00U, 0x00U, 0x04U}};
    const unsigned step = imu.power_step;
    const BSP_IMU_Chip_t chip = step == 0U ? BSP_IMU_GYRO : BSP_IMU_ACCEL;
    if (imu.power_phase == 0U) {
        memset(imu.tx, 0, sizeof(imu.tx));
        imu.tx[0] = address[step];
        imu.tx[1] = value[imu.power_target][step];
        if (start_transfer(chip, 2U, now_ms)) { imu.power_phase = 1U; }
        return;
    }
    if (imu.power_phase == 2U) {
        if (now_ms - imu.power_at_ms < (step == 0U ? 30U : 5U)) { return; }
        if (begin(chip, address[step], step == 0U ? 2U : 3U, now_ms)) {
            imu.power_phase = 3U;
        }
        return;
    }
    const HAL_StatusTypeDef result = BSP_IMU_TakeResult();
    if (result == HAL_BUSY && now_ms - imu.started_at <= 5U) { return; }
    if (result != HAL_OK || now_ms - imu.started_at > 5U) {
        imu.failed = true; /* Keep a timed-out transfer's buffers borrowed. */
        return;
    }
    if (imu.power_phase == 1U) {
        imu.power_at_ms = now_ms;
        imu.power_phase = 2U;
        return;
    }
    if (imu.rx[step == 0U ? 1U : 2U] != value[imu.power_target][step]) {
        imu.failed = true;
        return;
    }
    imu.power_phase = 0U;
    if (++imu.power_step == 3U) {
        imu.enabled = imu.power_target;
        imu.power_transition = false;
        imu.enabled_at = now_ms;
        BSP_IMU_GetReadyCounts(imu.used_ready);
    }
}

static int16_t signed_le(const uint8_t *data)
{
    const uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return value >= 0x8000U ? (int16_t)((int32_t)value - 65536) : (int16_t)value;
}

void BMI088_Run(uint32_t now_ms)
{
    if (!imu.initialized || imu.failed || __get_IPSR() != 0U) { return; }
    if (imu.power_transition) { update_power(now_ms); return; }
    if (!imu.enabled || now_ms - imu.enabled_at < 100U) { return; }
    if (imu.phase == 0U) {
        BSP_IMU_GetReadyCounts(imu.ready);
        if (now_ms - imu.cycle_at < 10U ||
            imu.ready[0] == imu.used_ready[0] || imu.ready[1] == imu.used_ready[1]) {
            return;
        }
        imu.cycle_at = now_ms;
        if (begin(BSP_IMU_ACCEL, 0x12U, 11U, now_ms)) { imu.phase = 1U; }
        return;
    }
    const HAL_StatusTypeDef result = BSP_IMU_TakeResult();
    if (result == HAL_BUSY && now_ms - imu.started_at <= 5U) { return; }
    if (result != HAL_OK || now_ms - imu.started_at > 5U) {
        imu.failed = true; /* No buffer reuse or automatic bus reset after timeout. */
        return;
    }
    if (imu.phase == 1U) {
        memcpy(imu.accel, &imu.rx[2], 9U);
        if (begin(BSP_IMU_GYRO, 0x02U, 7U, now_ms)) { imu.phase = 2U; }
        return;
    }
    const uint32_t sensor_time = (uint32_t)imu.accel[6] |
        ((uint32_t)imu.accel[7] << 8) | ((uint32_t)imu.accel[8] << 16);
    if (!imu.have_sensor_time || sensor_time != imu.sensor_time) {
        for (unsigned i = 0U; i < 3U; ++i) {
            imu.sample.accel_m_s2[i] = signed_le(&imu.accel[i * 2U]) *
                (3.0f * 9.80665f / 32768.0f);
            imu.sample.gyro_rad_s[i] = signed_le(&imu.rx[1U + i * 2U]) *
                (2000.0f * 0.01745329251994329577f / 32768.0f);
        }
        imu.sample.received_at_ms = now_ms;
        ++imu.sample.sequence;
        imu.have_sensor_time = true;
        imu.sensor_time = sensor_time;
    }
    imu.used_ready[0] = imu.ready[0];
    imu.used_ready[1] = imu.ready[1];
    imu.phase = 0U;
}

HAL_StatusTypeDef BMI088_GetSample(BMI088_Sample_t *sample)
{
    if (sample == NULL || __get_IPSR() != 0U) { return HAL_ERROR; }
    if (!imu.initialized || imu.failed || !imu.enabled || imu.power_transition ||
        imu.sample.sequence == 0U) {
        return HAL_ERROR;
    }
    *sample = imu.sample;
    return HAL_OK;
}
