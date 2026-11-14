/* 仅用于完整链接检查，不是可烧录的应用：无时钟配置、外设启动或业务循环。 */
#include "main.h"

void Error_Handler(void)
{
    __disable_irq();
    for (;;) {}
}

int main(void)
{
    for (;;) {}
}
