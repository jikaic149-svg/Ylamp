#include "w25q64.h"
#include "bsp_ospi.h"

bool W25Q64_Status(uint8_t *sr1,uint8_t *sr2)
{
    return sr1 && sr2 &&
        BSP_OSPI_Command(0x05,0,false,0,sr1,1) &&
        BSP_OSPI_Command(0x35,0,false,0,sr2,1);
}
bool W25Q64_Init(void)
{
    uint8_t id[3]={0},sr1=0,sr2=0;
    return BSP_OSPI_Init() && BSP_OSPI_Command(0x9f,0,false,0,id,3) &&
        id[0]==0xef && id[1]==0x40 && id[2]==0x17 &&
        W25Q64_Status(&sr1,&sr2) && !(sr1&1U) && !(sr2&0x80U);
}
bool W25Q64_Read(uint32_t address,uint8_t *data,uint16_t length)
{
    return data && length && length<=256U && address<W25Q64_SIZE &&
        length<=W25Q64_SIZE-address &&
        BSP_OSPI_Command(0x03,address,true,0,data,length);
}
static bool write_enable(void)
{
    uint8_t status=0;
    return BSP_OSPI_Command(0x06,0,false,0,0,0) &&
        BSP_OSPI_Command(0x05,0,false,0,&status,1) && (status&2U);
}
bool W25Q64_Program(uint32_t address,const uint8_t *data,uint16_t length)
{
    return data && length && length<=W25Q64_PAGE && address<W25Q64_SIZE &&
        length<=W25Q64_SIZE-address && (address%W25Q64_PAGE)+length<=W25Q64_PAGE &&
        write_enable() && BSP_OSPI_Command(0x02,address,true,data,0,length);
}
bool W25Q64_Erase(uint32_t address)
{
    return address<W25Q64_SIZE && address%W25Q64_SECTOR==0 &&
        write_enable() && BSP_OSPI_Command(0x20,address,true,0,0,0);
}
bool W25Q64_Suspend(void) { return BSP_OSPI_Command(0x75,0,false,0,0,0); }
bool W25Q64_Resume(void) { return BSP_OSPI_Command(0x7a,0,false,0,0,0); }
