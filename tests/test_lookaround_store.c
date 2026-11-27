#include "lookaround_store.h"
#include "lookaround.h"
#include "five_history.h"
#include "five_history_config.h"
#include "bsp_ospi.h"
#include "w25q64.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1,hfdcan2;
uint32_t test_ipsr,test_primask,test_tick;
static uint8_t nor[4*4096];
static uint32_t program_until,erase_until,erase_at;
static unsigned programs,erases;
static bool wel,fail;
uint32_t HAL_GetTick(void) { return test_tick; }
bool BSP_OSPI_Init(void) { return true; }
bool BSP_OSPI_Command(uint8_t op,uint32_t at,bool addressed,
                      const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    if (erase_until && test_tick>=erase_until) {
        memset(nor+erase_at-FIVE_LOOKAROUND_BASE,255,4096);erase_until=0;
    }
    if (fail) { return false; }
    const bool busy=test_tick<program_until || erase_until;
    if (op==0x9f) { rx[0]=0xef;rx[1]=0x40;rx[2]=0x17;return true; }
    if (op==0x05 || op==0x35) { *rx=op==0x05 ? (uint8_t)(busy|(wel ? 2 : 0)) : 0;return true; }
    if (op==0x06) { assert(!busy);wel=true;return true; }
    assert(addressed && at>=FIVE_LOOKAROUND_BASE &&
           at+length<=FIVE_HISTORY_BASE+FIVE_HISTORY_BYTES);
    assert(!busy);
    if (op==0x03) { memcpy(rx,nor+at-FIVE_LOOKAROUND_BASE,length);return true; }
    assert(wel);wel=false;
    if (op==0x20) { assert(at%4096==0);erase_at=at;erase_until=test_tick+40;++erases;return true; }
    assert(op==0x02 && length && length<=256 && at%256+length<=256);
    for (unsigned i=0;i<length;++i) {
        uint8_t *b=&nor[at-FIVE_LOOKAROUND_BASE+i];assert((*b&tx[i])==tx[i]);*b&=tx[i];
    }
    ++programs;program_until=test_tick+2;return true;
}
static void reboot(void)
{
    program_until=erase_until=0;fail=wel=false;
    FiveHistory_ResetForTest();LookaroundStore_ResetForTest();LookaroundStore_Init();
}
static void empty(void)
{
    memset(nor,255,sizeof nor);test_tick=0;programs=erases=0;reboot();
    assert(LookaroundStore_StatusNow()->flags==LOOKAROUND_READY);
}
static uint32_t upload(float shift)
{
    assert(LookaroundStore_Begin(32));
    const float anchor[]={131,-1,19,116,14};
    uint32_t checksum=2166136261U;
    for (unsigned i=0;i<32;++i) {
        FiveTrajectory_Point point={.at_ms=500*(i+1)};
        memcpy(point.position_deg,anchor,sizeof anchor);point.position_deg[0]+=shift;
        uint8_t bytes[64];FiveTrajectory_EncodePoint(bytes,&point);
        if (!i) {
            bytes[4]=0;bytes[5]=0;bytes[6]=0xc0;bytes[7]=0x7f; /* NaN q, rejected transactionally. */
            assert(!LookaroundStore_Append(0,bytes));
            FiveTrajectory_EncodePoint(bytes,&point);
            assert(!LookaroundStore_Append(1,bytes));
        }
        for (unsigned k=0;k<64;++k) { checksum=(checksum^bytes[k])*16777619U; }
        assert(LookaroundStore_Append(i,bytes));
    }
    assert(!LookaroundStore_Save(checksum^1U));
    assert(LookaroundStore_Save(checksum));
    assert(!LookaroundStore_Begin(1));
    return checksum;
}
static void complete(unsigned sequence)
{
    for (unsigned i=0;i<1000 && (LookaroundStore_StatusNow()->flags&LOOKAROUND_SAVING);++i) {
        LookaroundStore_Run(test_tick+=2);
    }
    const LookaroundStore_Status *s=LookaroundStore_StatusNow();
    assert(s->sequence==sequence && s->count==32 && s->flags&LOOKAROUND_LOADED &&
           !(s->flags&(LOOKAROUND_SAVING|LOOKAROUND_IO_ERROR)) && !s->errors);
    for (unsigned i=8192;i<sizeof nor;++i) { assert(nor[i]==255); }
}
static void persistence(void)
{
    empty();
    assert(!LookaroundStore_Points(NULL) && !LookaroundStore_Begin(33));
    const uint32_t checksum=upload(0);
    assert(W25Q64_TryLock(W25Q64_HISTORY));
    for (unsigned i=0;i<20;++i) { LookaroundStore_Run(test_tick+=2); }
    assert(!programs && !erases);
    W25Q64_Unlock(W25Q64_HISTORY);complete(1);
    assert(LookaroundStore_StatusNow()->checksum==checksum);
    reboot();
    unsigned count=0;const FiveTrajectory_Point *points=LookaroundStore_Points(&count);
    assert(points && count==32 && points[0].position_deg[0]==131 && points[31].at_ms==16000);
    upload(.25f);complete(2);reboot();
    assert(LookaroundStore_StatusNow()->sequence==2 && LookaroundStore_Points(NULL)[0].position_deg[0]==131.25f);
    nor[4096+64+4]^=1;reboot(); /* New body CRC failure selects the old committed slot. */
    assert(LookaroundStore_StatusNow()->sequence==1 && LookaroundStore_Points(NULL)[0].position_deg[0]==131);
    upload(.5f);complete(2); /* Rewrite only the inactive, recognisably owned slot. */
    assert(erases==1);
}
static void interrupted(void)
{
    empty();upload(0);complete(1);
    upload(.25f);
    LookaroundStore_Run(test_tick+=2); /* Header/body page only, no commit marker. */
    fail=true;LookaroundStore_Run(test_tick+=2);
    assert(LookaroundStore_StatusNow()->flags&LOOKAROUND_IO_ERROR);
    fail=false;reboot();
    assert(LookaroundStore_StatusNow()->sequence==1 && LookaroundStore_Points(NULL)[0].position_deg[0]==131);
    upload(.5f);complete(2);
    /* Power loss at every transaction phase retains a valid old or new commit. */
    for (unsigned cut=0;cut<45;++cut) {
        uint8_t saved[sizeof nor];memcpy(saved,nor,sizeof nor);
        const uint32_t old=LookaroundStore_StatusNow()->sequence;
        upload(.75f);
        for (unsigned i=0;i<cut;++i) { LookaroundStore_Run(test_tick+=2); }
        reboot();
        assert(LookaroundStore_StatusNow()->flags&LOOKAROUND_LOADED);
        assert(LookaroundStore_StatusNow()->sequence==old ||
               LookaroundStore_StatusNow()->sequence==old+1);
        memcpy(nor,saved,sizeof nor);reboot();
    }
}
static void foreign(void)
{
    empty();nor[0]=0x12;reboot();
    assert(LookaroundStore_StatusNow()->flags&LOOKAROUND_FOREIGN);
    upload(0);complete(1);
    assert(nor[0]==0x12 && !LookaroundStore_Begin(32) && !erases);
    empty();nor[0]=0x12;nor[4096]=0x34;reboot();
    assert(!LookaroundStore_Begin(32) && !programs && !erases);
}
int main(void)
{
    persistence();interrupted();foreign();
    puts("PASS: W25Q64 32-point CRC/commit/readback/reboot, foreign partition, torn writes, chip ownership");
}
