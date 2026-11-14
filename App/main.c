/* DM-MC-Board02 / STM32H723：FreeRTOS 应用入口，不调用参考工程旧业务。
 * 时钟/MPU 参数沿用 H7_BSP-HEAD 的 CubeMX main.c。
 * 参考生成代码 Copyright (c) 2026 STMicroelectronics，许可见原工程。
 * 启动只查询；策略获准后也须 KEY 和启动条件通过才会使能。 */
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "bsp_key.h"
#include "bsp_power.h"
#include "bsp_rgb.h"
#include "bsp_uart.h"
#include "motor_task.h"
#include "motor_tuning.h"
#include "motor_serial.h"
#include "motor_usb.h"
#include "can3_telemetry.h"
#include "usb_telemetry.h"
#include "usb_config.h"
#include "joint_calibration.h"
#include "serial_config.h"
#include "fault.h"
#ifdef YLAMP_J5_TEST_ONLY
#include "j5_test.h"
#endif
#ifdef YLAMP_CAN_CHECK_ONLY
#include "can_check.h"
#endif

extern TIM_HandleTypeDef htim2;
void Ylamp_StartScheduler(void);
HAL_StatusTypeDef YlampSerialDMA_Init(void);

/* J1..J5通信句柄；仅前两台交给双轴控制，其余接收反馈/支持时只读查询。 */
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

#if !defined(YLAMP_J5_TEST_ONLY) && !defined(YLAMP_CAN_CHECK_ONLY)
static void report_status(void)
{
    MotorTask_Status_t status;
    if (MotorTask_GetStatus(&status) != BSP_CAN_OK) { return; }
    /* USART1 is exclusively binary telemetry/RPC; do not interleave printf. */
    if (status.state == MOTOR_TASK_FAULT) {
        (void)BSP_RGB_Set(8U, 0U, 0U);
    } else if (!status.policy_ready) {
        (void)BSP_RGB_Set(4U, 0U, 4U);
    } else if (status.state == MOTOR_TASK_RUNNING) {
        (void)BSP_RGB_Set(0U, 6U, 0U);
    } else {
        (void)BSP_RGB_Set(0U, 0U, 6U);
    }
}
#endif

/* One application task owns all mutable Task/Device state. ISR callbacks only
 * deliver bounded feedback/transport events; no extra unsynchronized readers. */
void Ylamp_ControlStep(uint32_t now_ms)
{
#ifdef YLAMP_CAN_CHECK_ONLY
    g_fault_stage = YLAMP_STAGE_MOTOR;
    CanCheck_Run(now_ms);
    g_fault_stage = YLAMP_STAGE_WAIT;
#elif defined(YLAMP_J5_TEST_ONLY)
    /* Exclusive bench owner: no KEY, dual-axis Task/RPC or CAN2 TX. */
    g_fault_stage = YLAMP_STAGE_MOTOR;
    J5Test_Run(now_ms);
    g_fault_stage = YLAMP_STAGE_WAIT;
#else
    static uint32_t last_report_ms;
    static uint32_t last_monitor_query_ms;
    g_fault_stage = YLAMP_STAGE_MOTOR;
    (void)MotorTask_Run(now_ms);
    /* Its CAN snapshot may include an IRQ newer than the loop's initial tick. */
    now_ms = HAL_GetTick();
    /* J1/J2 已由 Task 查询；J4/J5 仅只读查询，不发 MIT/使能/写参。
     * J3旧协议由Device拒绝独立查询，只接收显式控制命令的应答；
     * 不以反复失能/零MIT伪装只读。每轮最多两帧，FIFO忙时不补发。 */
    if (now_ms - last_monitor_query_ms >= g_motor_config.query_period_ms) {
        last_monitor_query_ms = now_ms;
        for (unsigned i = MOTOR_TASK_JOINT_COUNT; i < MOTOR_CAN_JOINT_COUNT; ++i) {
            (void)DM_Motor_RequestFeedback(motors[i]);
        }
    }
    g_fault_stage = YLAMP_STAGE_IMU;
    BMI088_Run(now_ms);
    g_fault_stage = YLAMP_STAGE_CALIBRATION;
    JointCalibration_Run(now_ms);
    g_fault_stage = YLAMP_STAGE_TUNING;
    (void)MotorTuning_Run(now_ms);
    g_fault_stage = YLAMP_STAGE_SERIAL;
    (void)MotorSerial_Run(now_ms);
    if (now_ms - last_report_ms >= g_motor_config.status_period_ms) {
        last_report_ms = now_ms;
        g_fault_stage = YLAMP_STAGE_STATUS;
        report_status();
    }
    g_fault_stage = YLAMP_STAGE_WAIT;
#endif
    /* Independent read-only output in normal AND exclusive bench builds.
     * Does not change motor querying, enable policy or SWD test requests. */
#ifdef YLAMP_CAN3_TELEMETRY
    CAN3Telemetry_Run(HAL_GetTick());
#endif
#if !defined(YLAMP_J5_TEST_ONLY) && !defined(YLAMP_CAN_CHECK_ONLY)
    if (MotorUSB_Run()) { USBTelemetry_Run(HAL_GetTick()); }
#else
    USBTelemetry_Run(HAL_GetTick()); /* No control dispatcher linked into bench. */
#endif
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
        BSP_KEY_Init(g_motor_config.key_debounce_ms) != HAL_OK) {
        Error_Handler();
    }
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
    g_fault_stage = YLAMP_STAGE_IMU_INIT;
    MX_SPI2_Init();
    /* IMU is optional for the saved-reference drag mode. Failure leaves it
     * unavailable (calibration status/op10), not a successful zero capture. */
    if (BSP_IMU_Init() == HAL_OK) { (void)BMI088_Init(); }
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
     * 两路 EN 同次置高，对外 5V/PC15 保持关闭。
     * 上电先只读角度，Task 不因收到反馈就使能，不自动拉回区间外的关节。 */
    if (BSP_POWER_SetMotorPower(true) != HAL_OK) { Error_Handler(); }
    const uint32_t powered_at_ms = HAL_GetTick();
    while (HAL_GetTick() - powered_at_ms < g_motor_config.power_settle_ms) {
        (void)BSP_KEY_Scan(HAL_GetTick()); /* 丢弃供电稳定阶段的按键动作。 */
        HAL_Delay(1U);
    }
#ifdef YLAMP_CAN_CHECK_ONLY
    CanCheck_Init(motors, HAL_GetTick());
    (void)BSP_RGB_Set(4U, 0U, 4U);
#elif defined(YLAMP_J5_TEST_ONLY)
    J5Test_Init(motors[4], HAL_GetTick());
    (void)BSP_RGB_Set(4U, 0U, 4U);
#else
    if (MotorTask_Init(&g_motor_config, motors, HAL_GetTick()) != BSP_CAN_OK) {
        Error_Handler();
    }
    /* Native USB quick control, CAN/UART full editors dispatch in one task.
     * Only one host/transport may own
     * a configuration transaction. Neither host queries the motors directly.
     * The CAN endpoint follows J1/J2 on CAN2; attach the existing PC tool there. */
    if (MotorTuning_Init(g_motor_config.joint[0].device.bus) != BSP_CAN_OK) {
        Error_Handler();
    }
    if (JointCalibration_Init() != BSP_CAN_OK) { Error_Handler(); }
    if (MotorSerial_Init(&huart1) != HAL_OK) { Error_Handler(); }
    MotorUSB_Init(); /* USB controls J1/J2 only; bench builds stay telemetry-only. */
#endif
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
void HAL_GPIO_EXTI_Callback(uint16_t pin) { BSP_IMU_NotifyReady(pin); }
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BMI088_ACCEL__INTERRUPT_Pin);
    HAL_GPIO_EXTI_IRQHandler(BMI088_GYRO__INTERRUPT_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM2) { HAL_IncTick(); }
}
