#include "bsp_ospi.h"
#include "main.h"

static OSPI_HandleTypeDef port;
static bool initialized;

bool BSP_OSPI_Init(void)
{
    if (__get_IPSR()) { return false; }
    if (initialized) { return true; }
    RCC_PeriphCLKInitTypeDef clock={0};
    clock.PeriphClockSelection=RCC_PERIPHCLK_OSPI;
    clock.OspiClockSelection=RCC_OSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK) { return false; }
    __HAL_RCC_OCTOSPIM_CLK_ENABLE();
    __HAL_RCC_OSPI2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
    /* Original Board02 routing. In 1-1-1 SPI, HOLD/WP must remain high;
     * no QE/status-register change is required. */
    GPIO_InitTypeDef pin={.Mode=GPIO_MODE_OUTPUT_PP,.Pull=GPIO_NOPULL,
        .Speed=GPIO_SPEED_FREQ_VERY_HIGH};
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1|GPIO_PIN_3,GPIO_PIN_SET);
    pin.Pin=GPIO_PIN_1|GPIO_PIN_3;HAL_GPIO_Init(GPIOA,&pin);
    pin.Mode=GPIO_MODE_AF_PP;
    pin.Pin=GPIO_PIN_0;pin.Alternate=GPIO_AF4_OCTOSPIM_P1;HAL_GPIO_Init(GPIOB,&pin);
    pin.Pin=GPIO_PIN_2;pin.Alternate=GPIO_AF9_OCTOSPIM_P1;HAL_GPIO_Init(GPIOB,&pin);
    pin.Pin=GPIO_PIN_11;pin.Alternate=GPIO_AF9_OCTOSPIM_P1;HAL_GPIO_Init(GPIOD,&pin);
    pin.Pin=GPIO_PIN_11;pin.Alternate=GPIO_AF11_OCTOSPIM_P1;HAL_GPIO_Init(GPIOE,&pin);
    port.Instance=OCTOSPI2;
    port.Init=(OSPI_InitTypeDef){
        .FifoThreshold=1,.DualQuad=HAL_OSPI_DUALQUAD_DISABLE,
        .MemoryType=HAL_OSPI_MEMTYPE_MICRON,.DeviceSize=23,
        .ChipSelectHighTime=2,.ClockMode=HAL_OSPI_CLOCK_MODE_3,
        .ClockPrescaler=8,.SampleShifting=HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE,
        .DelayBlockBypass=HAL_OSPI_DELAY_BLOCK_BYPASSED};
    if (HAL_OSPI_Init(&port)!=HAL_OK) { return false; }
    OSPIM_CfgTypeDef manager={.ClkPort=1,.NCSPort=1,.IOLowPort=HAL_OSPIM_IOPORT_1_LOW};
    initialized=HAL_OSPIM_Config(&port,&manager,1U)==HAL_OK;
    return initialized;
}

bool BSP_OSPI_Command(uint8_t instruction,uint32_t address,bool addressed,
                      const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    if (!initialized || __get_IPSR() || length>256U ||
        (length && ((!tx && !rx) || (tx && rx))) ||
        (!length && (tx || rx))) { return false; }
    OSPI_RegularCmdTypeDef command={
        .OperationType=HAL_OSPI_OPTYPE_COMMON_CFG,.FlashId=HAL_OSPI_FLASH_ID_1,
        .Instruction=instruction,.InstructionMode=HAL_OSPI_INSTRUCTION_1_LINE,
        .InstructionSize=HAL_OSPI_INSTRUCTION_8_BITS,.Address=address,
        .AddressMode=addressed ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_NONE,
        .AddressSize=HAL_OSPI_ADDRESS_24_BITS,
        .DataMode=length ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_NONE,
        .NbData=length,.SIOOMode=HAL_OSPI_SIOO_INST_EVERY_CMD};
    /* Only the short command/data transfer is polled (30MHz at HCLK240).
     * Never poll the flash's millisecond program/erase cycle here. */
    if (HAL_OSPI_Command(&port,&command,1U)!=HAL_OK) { return false; }
    if (rx) { return HAL_OSPI_Receive(&port,rx,1U)==HAL_OK; }
    if (tx) { return HAL_OSPI_Transmit(&port,(uint8_t *)tx,1U)==HAL_OK; }
    return true;
}
