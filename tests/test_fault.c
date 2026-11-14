#include "fault.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const uint32_t msp = 0x2001F000U, psp = 0x20004000U;
    const uint32_t returns[] = {
        0xFFFFFFF1U, 0xFFFFFFF9U, 0xFFFFFFFDU,
        0xFFFFFFE1U, 0xFFFFFFE9U, 0xFFFFFFEDU
    };
    for (unsigned i = 0U; i < sizeof(returns) / sizeof(returns[0]); ++i) {
        const uint32_t exc = returns[i];
        const uint32_t sp = (exc & 4U) != 0U ? psp : msp;
        assert(YlampFault_FrameAddress(exc, msp, psp, 0U) == sp);
        /* FP extension must not move the basic r0..xPSR frame by 18 words. */
        assert(YlampFault_FrameAddress(exc, msp, psp, 0x02008200U) == sp);
        for (unsigned bit = 0U; bit < 32U; ++bit) {
            if ((0x3838U & (1U << bit)) != 0U) {
                assert(YlampFault_FrameAddress(exc, msp, psp, 1U << bit) == 0U);
            }
        }
    }
    assert(YlampFault_FrameAddress(0xFFFFFFFDU, msp, 0x2001FFE0U, 0U) == 0x2001FFE0U);
    assert(YlampFault_FrameAddress(0xFFFFFFEDU, msp, 0x2001FF98U, 0U) == 0x2001FF98U);
    assert(YlampFault_FrameAddress(0xFFFFFFEDU, msp, 0x2001FFA0U, 0U) == 0U);
    const uint32_t invalid[] = {
        0U, 0x1FFFFFF8U, 0x20000004U, 0x2001FFE8U,
        0x20020000U, 0x24000000U, 0xFFFFFFFFU
    };
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        assert(YlampFault_FrameAddress(0xFFFFFFFDU, msp, invalid[i], 0U) == 0U);
    }
    assert(YlampFault_FrameAddress(0xFFFFFFFDU, msp, 0x20000000U, 0U) == 0x20000000U);
    assert(YlampFault_FrameAddress(0xFFFFFFF5U, msp, psp, 0U) == 0U);
    assert(YlampFault_FrameAddress(0U, msp, psp, 0U) == 0U);
    puts("PASS: exception return whitelist, MSP/PSP selection, basic/FP frame"
         " bounds, alignment, corrupt stacks and stacking-error rejection.");
}
