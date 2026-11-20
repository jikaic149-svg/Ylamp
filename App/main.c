/* DM-MC-Board02 / STM32H723：FreeRTOS 应用入口，不调用参考工程旧业务。
 * 时钟/MPU 参数沿用 H7_BSP-HEAD 的 CubeMX main.c。
 * 参考生成代码 Copyright (c) 2026 STMicroelectronics，许可见原工程。
 * 冷启动不自动使能；同一任务负责五轴重力、轨迹和 USB 请求。 */
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "bsp_rgb.h"
#include "bsp_uart.h"
#include "five_control.h"
#include "bmi088_port.h"
#include "motor_usb.h"
#include "can3_telemetry.h"
#include "usb_telemetry.h"
#include "usb_config.h"
#include "joint_calibration.h"
#include "yummy_imu.h"
#include "serial_config.h"
#include "fault.h"
extern TIM_HandleTypeDef htim2;
void Ylamp_StartScheduler(void);
HAL_StatusTypeDef YlampSerialDMA_Init(void);

/* Same order throughout: J1 and J2 on CAN2, J3..J5 on CAN1. */
static DM_Motor_t *motors[MOTOR_CAN_JOINT_COUNT];

static void configure_mpu(void)
{
    const MPU_Region_InitTypeDef region = {
        .Enable = MPU_REGION_ENABLE, .Number = MPU_REGION_NUMBER0,
        .BaseAddress = 0x24000000U, .Size = MPU_REGION_SIZE_64KB,
        .SubRegionDisable = 0U, .TypeExtField = MPU_TEX_LEVEL1,
        .AccessPermission = MPU_REGION_FULL_ACCESS,
        .DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE,
        .IsShareable = MPU_ACCESS_SHAREABLE,
        .IsCacheable = MPU_ACCESS_NOT_CACHEABLE,
        .IsBufferable = MPU_ACCESS_NOT_BUFFERABLE
    };
    HAL_MPU_Disable();
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void configure_clock(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) { Error_Handler(); }
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSI48State = RCC_HSI48_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 2U;
    oscillator.PLL.PLLN = 40U;
    oscillator.PLL.PLLP = 1U;
    oscillator.PLL.PLLQ = 4U;
    oscillator.PLL.PLLR = 2U;
    oscillator.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = 0U;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) { Error_Handler(); }
    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
        RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clock.AHBCLKDivider = RCC_HCLK_DIV2;
    clock.APB3CLKDivider = RCC_APB3_DIV2;
    clock.APB1CLKDivider = RCC_APB1_DIV2;
    clock.APB2CLKDivider = RCC_APB2_DIV2;
    clock.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }

    RCC_PeriphCLKInitTypeDef peripheral = {0};
    peripheral.PeriphClockSelection = RCC_PERIPHCLK_SPI6 | RCC_PERIPHCLK_ADC;
    peripheral.PLL2.PLL2M = 12U;
    peripheral.PLL2.PLL2N = 100U;
    peripheral.PLL2.PLL2P = 2U;
    peripheral.PLL2.PLL2Q = 4U;
    peripheral.PLL2.PLL2R = 2U;
    peripheral.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
    peripheral.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    peripheral.PLL2.PLL2FRACN = 0U;
    peripheral.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    peripheral.Spi6ClockSelection = RCC_SPI6CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK) { Error_Handler(); }
}

void Ylamp_ControlStep(uint32_t now)
{
    static uint32_t report;
    g_fault_stage=YLAMP_STAGE_MOTOR;
    FiveControl_Tick(now);
    if (BSP_KEY_Scan(now)==BSP_KEY_EVENT_PRESSED) { (void)FiveControl_KeyPress(); }
    g_fault_stage=YLAMP_STAGE_IMU;
    BMI088_Run(now);
    YummyIMU_Run();
    g_fault_stage=YLAMP_STAGE_CALIBRATION;
    JointCalibration_Run(now);
    if (MotorUSB_Run()) { USBTelemetry_Run(HAL_GetTick()); }
#ifdef YLAMP_CAN3_TELEMETRY
    CAN3Telemetry_Run(HAL_GetTick());
#endif
    if (now-report>=500U) {
        report=now;
        const FiveControl_Status *s=FiveControl_StatusNow();
        (void)BSP_RGB_Set(s->fault?8U:s->phase==FIVE_IMPEDANCE || s->phase==FIVE_REVERSE_J3?5U:0U,
                          s->phase==FIVE_GRAVITY || s->phase==FIVE_POSITION || s->phase==FIVE_IMPEDANCE?5U:0U,
                          s->phase==FIVE_MOTION || s->phase==FIVE_WAIT?5U:0U);
    }
    g_fault_stage=YLAMP_STAGE_WAIT;
}

int main(void)
{
    YlampFault_Boot();
    configure_mpu();
    SCB_EnableICache();
    SCB_EnableDCache();
    if (HAL_Init() != HAL_OK) { Error_Handler(); }
    g_fault_stage = YLAMP_STAGE_CLOCK;
    configure_clock();
    MX_GPIO_Init();
    if (BSP_POWER_Init() != HAL_OK ||
        BSP_KEY_Init(20U) != HAL_OK) {
        Error_Handler();
    }
    /* 用户确认：UART10/YummyIMU所接共享对外5V开机常开，独立于电机电源。 */
    if (BSP_POWER_Set(BSP_POWER_EXT_5V, true) != HAL_OK) { Error_Handler(); }
    MX_SPI6_Init();
    MX_USART1_UART_Init();
    /* Apply project baud before registering UART RX/TX; keep the SDK unchanged. */
    huart1.Init.BaudRate = YLAMP_SERIAL_BAUD_RATE;
    if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
    if (YlampSerialDMA_Init() != HAL_OK) { Error_Handler(); }
    if (BSP_RGB_Init() != HAL_OK) { Error_Handler(); }
    /* SPI2 generated MSP initializes DMA handles; enable its clock, but runtime
     * IMU acquisition uses SPI interrupts only, no DMA buffers/cache changes.
     * All blocking sensor setup happens while motor power is still OFF. */
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* UART10: PE2 RX / PE3 TX, YummyIMU factory 921600/8N1 USE output.
     * Receive only via BSP IT; no commands or changes to the module's flash. */
    MX_USART10_UART_Init();
    huart10.Init.BaudRate = 921600U;
    if (HAL_UART_Init(&huart10) == HAL_OK) {
        (void)YummyIMU_Init(&huart10, YUMMY_IMU_USE);
    }
    g_fault_stage = YLAMP_STAGE_IMU_INIT;
    MX_SPI2_Init();
    /* IMU is optional for the saved-reference drag mode. Failure leaves it
     * unavailable (calibration status/op10), not a successful zero capture. */
    if (BMI088_Port_Init() == HAL_OK) { (void)BMI088_Init(); }
    g_fault_stage = YLAMP_STAGE_CAN_INIT;
    MX_FDCAN1_Init();
    if (BSP_CAN_Init(&hfdcan1) != BSP_CAN_OK) { Error_Handler(); }
    /* V1.2 needs two independent buses. The generated RAM offsets are 0/853
     * words. CAN initialization does not switch power or enable motors. */
    MX_FDCAN2_Init();
    if (BSP_CAN_Init(&hfdcan2) != BSP_CAN_OK) { Error_Handler(); }
    /* Optional PC telemetry must not prevent motor control from booting.
     * Own configuration overrides the SDK's unused Classic-CAN3 initializer. */
#ifdef YLAMP_CAN3_TELEMETRY
    const BSP_CAN_Status_t can3_status = BSP_CAN3_InitFD();
#endif
    /* Existing SDK port selects HSI48, IRQ priority 5, embedded FS PHY, no DMA.
     * No generated usb_device.c/cdc_if.c or reference C++ application is used. */
    (void)BSP_USB_Init();
    g_fault_stage = YLAMP_STAGE_MOTOR_INIT;
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        if (DM_Motor_Init(&g_motor_can_config[i], &motors[i]) != BSP_CAN_OK) {
            Error_Handler();
        }
    }
#ifdef YLAMP_CAN3_TELEMETRY
    (void)CAN3Telemetry_Init(motors, HAL_GetTick(), can3_status);
#endif
    USBTelemetry_Source_t usb_sources[MOTOR_CAN_JOINT_COUNT];
    _Static_assert(MOTOR_CAN_JOINT_COUNT <= MOTOR_SNAPSHOT_MAX_JOINTS, "USB joint capacity");
    _Static_assert(USB_TELEMETRY_PERIOD_MS > 0U && USB_TELEMETRY_PERIOD_MS <= UINT16_MAX,
                   "USB period is u16 milliseconds");
    _Static_assert(USB_TELEMETRY_MAX_AGE_MS > 0U && USB_TELEMETRY_MAX_AGE_MS <= UINT16_MAX,
                   "USB feedback age is u16 milliseconds");
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        usb_sources[i] = (USBTelemetry_Source_t){
            .motor = motors[i], .config = g_motor_can_config[i],
            .bus_number = g_motor_can_config[i].bus == &hfdcan1 ? 1U : 2U
        };
    }
    (void)USBTelemetry_Init(usb_sources, MOTOR_CAN_JOINT_COUNT,
        USB_TELEMETRY_PERIOD_MS, USB_TELEMETRY_MAX_AGE_MS, HAL_GetTick());
    /* 用户确认：CAN1/PC14（VCC_OUT0）、CAN2/PC13（VCC_OUT1）电源联动。
     * 两路 EN 同次置高，不改变前面已开启的对外5V/PC15。
     * 上电先只读角度，Task 不因收到反馈就使能，不自动拉回区间外的关节。 */
    if (BSP_POWER_SetMotorPower(true) != HAL_OK) { Error_Handler(); }
    const uint32_t powered_at_ms = HAL_GetTick();
    while (HAL_GetTick() - powered_at_ms < 1000U) {
        (void)BSP_KEY_Scan(HAL_GetTick()); /* 丢弃供电稳定阶段的按键动作。 */
        HAL_Delay(1U);
    }
    if (!FiveControl_Init(motors,HAL_GetTick()) || JointCalibration_Init()!=BSP_CAN_OK) { Error_Handler(); }
    MotorUSB_Init();
    g_fault_stage = YLAMP_STAGE_SCHEDULER;
    Ylamp_StartScheduler();
    Error_Handler();
}

/* 仅硬件入口转发，按键策略与电机控制不在 ISR 中运行。
 * 完整应用不链接参考 stm32h7xx_it.c；覆盖 BSP 支持的端口，避免以后按既有
 * 接口启用其他端口时落入弱 Default_Handler。这里不初始化/使能额外外设。 */
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void FDCAN1_IT0_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan1); }
void FDCAN1_IT1_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan1); }
void FDCAN2_IT0_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan2); }
void FDCAN2_IT1_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan2); }
void FDCAN3_IT0_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan3); }
void FDCAN3_IT1_IRQHandler(void) { BSP_CAN_IRQHandler(&hfdcan3); }
void OTG_HS_IRQHandler(void) { BSP_USB_IRQHandler(); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
void UART5_IRQHandler(void) { HAL_UART_IRQHandler(&huart5); }
void USART6_IRQHandler(void) { HAL_UART_IRQHandler(&huart6); }
void UART7_IRQHandler(void) { HAL_UART_IRQHandler(&huart7); }
void USART10_IRQHandler(void) { HAL_UART_IRQHandler(&huart10); }
void SPI2_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi2); }
void HAL_GPIO_EXTI_Callback(uint16_t pin) { BMI088_Port_NotifyReady(pin); }
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BMI088_ACCEL__INTERRUPT_Pin);
    HAL_GPIO_EXTI_IRQHandler(BMI088_GYRO__INTERRUPT_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM2) { HAL_IncTick(); }
}
