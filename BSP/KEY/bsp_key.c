#include "bsp_key.h"

#include <stddef.h>

static bool initialized;
static bool stable_pressed;
static bool candidate_pressed;
static uint32_t candidate_since;
static uint32_t debounce_time;

static bool read_pressed(void)
{
    return HAL_GPIO_ReadPin(KEY__INPUT_GPIO_Port, KEY__INPUT_Pin) == GPIO_PIN_RESET;
}

HAL_StatusTypeDef BSP_KEY_Init(uint32_t debounce_ms)
{
    if (__get_IPSR() != 0U || debounce_ms == 0U || debounce_ms > 1000U) {
        return HAL_ERROR;
    }
    if (initialized) { return HAL_BUSY; }
    stable_pressed = read_pressed();
    candidate_pressed = stable_pressed;
    candidate_since = HAL_GetTick();
    debounce_time = debounce_ms;
    initialized = true;
    return HAL_OK;
}

BSP_KEY_Event_t BSP_KEY_Scan(uint32_t now_ms)
{
    if (!initialized || __get_IPSR() != 0U) { return BSP_KEY_EVENT_NONE; }
    const bool pressed = read_pressed();
    if (pressed != candidate_pressed) {
        candidate_pressed = pressed;
        candidate_since = now_ms;
    }
    if (candidate_pressed != stable_pressed &&
        (uint32_t)(now_ms - candidate_since) >= debounce_time) {
        stable_pressed = candidate_pressed;
        return stable_pressed ? BSP_KEY_EVENT_PRESSED : BSP_KEY_EVENT_RELEASED;
    }
    return BSP_KEY_EVENT_NONE;
}

HAL_StatusTypeDef BSP_KEY_GetPressed(bool *pressed)
{
    if (!initialized || pressed == NULL || __get_IPSR() != 0U) {
        return HAL_ERROR;
    }
    *pressed = stable_pressed;
    return HAL_OK;
}
