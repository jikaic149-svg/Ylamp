#include "bsp_power.h"

#include <stddef.h>

static bool initialized;
static const uint16_t pins[3] = {
    DC24_0__OUTPUT_Pin, DC24_1__OUTPUT_Pin, DC5__OUTPUT_Pin
};

static bool valid_branch(BSP_POWER_Branch_t branch)
{
    return (unsigned)branch < 3U;
}

HAL_StatusTypeDef BSP_POWER_Init(void)
{
    if (__get_IPSR() != 0U) { return HAL_ERROR; }
    if (initialized) { return HAL_BUSY; }
    /* 同一个 GPIOC BSRR 写入，三路一起置低；不先置高再配置。 */
    HAL_GPIO_WritePin(GPIOC, pins[0] | pins[1] | pins[2], GPIO_PIN_RESET);
    initialized = true;
    return HAL_OK;
}

HAL_StatusTypeDef BSP_POWER_Set(BSP_POWER_Branch_t branch, bool enabled)
{
    if (!initialized || !valid_branch(branch) || __get_IPSR() != 0U) {
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(GPIOC, pins[branch],
                       enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_POWER_SetMotorPower(bool enabled)
{
    if (!initialized || __get_IPSR() != 0U) { return HAL_ERROR; }
    HAL_GPIO_WritePin(GPIOC, pins[0] | pins[1],
                       enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_POWER_AllOff(void)
{
    if (!initialized || __get_IPSR() != 0U) { return HAL_ERROR; }
    HAL_GPIO_WritePin(GPIOC, pins[0] | pins[1] | pins[2], GPIO_PIN_RESET);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_POWER_GetEnabled(BSP_POWER_Branch_t branch, bool *enabled)
{
    if (!initialized || !valid_branch(branch) || enabled == NULL ||
        __get_IPSR() != 0U) {
        return HAL_ERROR;
    }
    *enabled = (GPIOC->ODR & pins[branch]) != 0U;
    return HAL_OK;
}
