#include "five_history.h"
#include "five_history_config.h"
#include "joint_coordinates.h"
#include "bsp_ospi.h"
#include "w25q64.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1,hfdcan2;
static uint8_t nor[FIVE_HISTORY_BYTES];
static bool present=true,wel,suspended,io_failure;
static uint32_t clock_ms,program_until,erase_until,erase_address,count[5];
static int cut_after=-1;
static unsigned programs,erases,suspends,resumes;
static float pose[5];
#define RAD .01745329251994329577f

bool BSP_OSPI_Init(void) { return present; }
static void finish(void)
{
    if (erase_until && !suspended && clock_ms>=erase_until) {
        memset(nor+erase_address-FIVE_HISTORY_BASE,0xff,W25Q64_SECTOR);
        erase_until=0;
    }
}
bool BSP_OSPI_Command(uint8_t op,uint32_t at,bool addressed,
                      const uint8_t *tx,uint8_t *rx,uint16_t length)
{
    finish();
    if (!present || io_failure) { return false; }
    bool busy=clock_ms<program_until || (erase_until && !suspended);
    if (op==0x9f) {
        assert(!addressed && rx && length==3);rx[0]=0xef;rx[1]=0x40;rx[2]=0x17;return true;
    }
    if (op==0x05 || op==0x35) {
        assert(!addressed && rx && length==1);
        *rx=op==0x05 ? (uint8_t)(busy|(wel?2:0)) : (suspended?0x80:0);return true;
    }
    if (op==0x06) { assert(!busy);wel=true;return true; }
    if (op==0x75) {
        assert(erase_until && !suspended && clock_ms>=program_until);
        suspended=true;++suspends;return true;
    }
    if (op==0x7a) {
        assert(erase_until && suspended && clock_ms>=program_until);
        suspended=false;erase_until=clock_ms+40;++resumes;return true;
    }
    assert(addressed && at>=FIVE_HISTORY_BASE && at<FIVE_HISTORY_BASE+FIVE_HISTORY_BYTES);
    assert(at+length<=FIVE_HISTORY_BASE+FIVE_HISTORY_BYTES); /* IMU region untouched. */
    const unsigned offset=at-FIVE_HISTORY_BASE;
    if (op==0x03) {
        assert(!busy && rx && length);memcpy(rx,nor+offset,length);return true;
    }
    if (op==0x02) {
        assert(!busy && wel && tx && length && at%256+length<=256);
        assert(!suspended || at/W25Q64_SECTOR!=erase_address/W25Q64_SECTOR);
        ++programs;wel=false;
        unsigned n=length;
        if (cut_after>=0 && (unsigned)cut_after<n) { n=(unsigned)cut_after; }
        for (unsigned i=0;i<n;++i) {
            assert((nor[offset+i]&tx[i])==tx[i]);nor[offset+i]&=tx[i];
        }
        if (cut_after>=0) { io_failure=true;return false; }
        program_until=clock_ms+2;return true;
    }
    assert(op==0x20 && !busy && wel && !suspended && at%W25Q64_SECTOR==0);
    wel=false;erase_address=at;erase_until=clock_ms+40;++erases;return true;
}
static void power_cycle(void)
{
    wel=false;suspended=false;program_until=erase_until=0;io_failure=false;cut_after=-1;
    memset(count,0,sizeof count);
    FiveHistory_ResetForTest();FiveHistory_Init();
}
static void empty(void)
{
    memset(nor,0xff,sizeof nor);clock_ms=0;present=true;
    programs=erases=suspends=resumes=0;
    for (unsigned i=0;i<5;++i) { pose[i]=g_joint_reference_config[i].zero_rad; }
    power_cycle();
    assert(FiveHistory_StatusNow()->flags==FIVE_HISTORY_READY);
}
static void feed(unsigned mask)
{
    for (unsigned i=0;i<5;++i) {
        if (!(mask&(1U<<i))) { continue; }
        const DM_Motor_Feedback_t f={.position_rad=pose[i],.velocity_rad_s=0,
            .received_at_ms=clock_ms,.received_count=++count[i],.state=1};
        float ref=FiveHistory_Reference(i,&f);
        assert(isfinite(ref));
    }
}
static void step(unsigned mask)
{
    clock_ms+=2;feed(mask);FiveHistory_Run(clock_ms);
}
static void committed(unsigned target,unsigned mask)
{
    for (unsigned i=0;i<1000 && FiveHistory_StatusNow()->writes<target;++i) { step(mask); }
    assert(FiveHistory_StatusNow()->writes==target);
    assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_READY);
    assert(!FiveHistory_StatusNow()->errors);
}
static void initial(void) { empty();committed(1,31); }

static void thresholds(void)
{
    initial();
    assert(FiveHistory_StatusNow()->saved_mask==31);
    for (unsigned axis=0;axis<5;++axis) {
        const unsigned target=FiveHistory_StatusNow()->writes+1;
        const float start=pose[axis];
        /* Whole single-turn detail need not be recorded every 30 degrees. */
        for (unsigned n=1;n<=149;++n) {
            pose[axis]=start+n*RAD;step(31);
            assert(FiveHistory_StatusNow()->writes==target-1);
        }
        pose[axis]=start+151*RAD;committed(target,31);
        assert(fabsf(FiveHistory_StatusNow()->saved_rad[axis]-pose[axis])<2e-6f);
    }
}
static void restore_and_wrap(void)
{
    initial();
    float saved[5];memcpy(saved,pose,sizeof saved);
    power_cycle();
    assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_LOADED);
    for (unsigned i=0;i<5;++i) {
        const DM_Motor_Feedback_t f={.position_rad=saved[i]+JOINT_TURN_RAD,
            .received_count=1,.received_at_ms=clock_ms,.state=1};
        assert(fabsf(FiveHistory_Reference(i,&f)-saved[i])<2e-6f);
        DM_Motor_Feedback_t next=f;next.received_count=2;next.received_at_ms+=2;
        next.position_rad-=JOINT_TURN_RAD;
        assert(fabsf(FiveHistory_Reference(i,&next)-saved[i])<2e-6f);
        assert(FiveHistory_StatusNow()->corrections[i]==1);
        /* Repeated snapshots cannot add another turn. */
        assert(fabsf(FiveHistory_Reference(i,&next)-saved[i])<2e-6f);
        assert(FiveHistory_StatusNow()->corrections[i]==1);
    }
    assert(FiveHistory_StatusNow()->restored_mask==31);
}
static void partial_axes_and_stale(void)
{
    empty();committed(1,27);assert(FiveHistory_StatusNow()->saved_mask==27);
    committed(2,31);assert(FiveHistory_StatusNow()->saved_mask==31);
    unsigned writes=FiveHistory_StatusNow()->writes;
    pose[0]+=160*RAD;feed(1);clock_ms+=102;
    for (unsigned i=0;i<30;++i) { FiveHistory_Run(clock_ms+=2); }
    assert(FiveHistory_StatusNow()->writes==writes);
    committed(writes+1,31);
}
static void torn_writes(void)
{
    initial();
    uint8_t base[sizeof nor];memcpy(base,nor,sizeof base);
    for (int cut=0;cut<=60;++cut) {
        memcpy(nor,base,sizeof nor);power_cycle();
        for (unsigned i=0;i<5;++i) { pose[i]=g_joint_reference_config[i].zero_rad; }
        pose[0]+=160*RAD;cut_after=cut;step(31);
        assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_IO_ERROR);
        power_cycle();
        assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_LOADED);
        assert(FiveHistory_StatusNow()->sequence==1);
        assert(fabsf(FiveHistory_StatusNow()->saved_rad[0]-g_joint_reference_config[0].zero_rad)<1e-6f);
    }
    for (int cut=0;cut<=4;++cut) {
        memcpy(nor,base,sizeof nor);power_cycle();
        pose[0]=g_joint_reference_config[0].zero_rad+160*RAD;
        step(31); /* Complete body, then interrupt commit. */
        cut_after=cut;step(31);power_cycle();
        assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_LOADED);
        assert(FiveHistory_StatusNow()->sequence==(cut==4 ? 2U : 1U));
    }
}
static void rollover_and_erase_loss(void)
{
    initial();
    for (unsigned n=2;n<=130;++n) {
        pose[0]=g_joint_reference_config[0].zero_rad+(n%2 ? 0 : 160*RAD);
        committed(n,31);
        step(31); /* Allow proactive reclaim; next checkpoint preempts it. */
    }
    assert(erases && suspends && resumes);
    unsigned seq=FiveHistory_StatusNow()->sequence;
    float saved=FiveHistory_StatusNow()->saved_rad[0];
    power_cycle();
    assert(FiveHistory_StatusNow()->sequence==seq);
    assert(fabsf(FiveHistory_StatusNow()->saved_rad[0]-saved)<1e-6f);
    /* The most recent sector survives a partly erased OLD sector. */
    while (!erase_until) { step(31); }
    unsigned offset=erase_address-FIVE_HISTORY_BASE;
    memset(nor+offset,0xff,W25Q64_SECTOR/2);
    power_cycle();
    assert(FiveHistory_StatusNow()->sequence==seq);
}
static void foreign_and_missing(void)
{
    empty();nor[0]=0x42;power_cycle();
    assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_FOREIGN);
    for (unsigned i=0;i<100;++i) { step(31); }
    assert(nor[0]==0x42 && !programs && !erases);
    empty();present=false;power_cycle();
    assert(FiveHistory_StatusNow()->flags&FIVE_HISTORY_NO_CHIP);
    step(31);assert(!programs && !erases);
}
static void initial_legal_branch(void)
{
    empty();
    DM_Motor_Feedback_t f={.received_count=1,.received_at_ms=clock_ms,.state=1};
    f.position_rad=192.85f*RAD;
    assert(fabsf(FiveHistory_Reference(3,&f)-(-167.15f*RAD))<2e-6f);
    assert(floorf(FiveHistory_Reference(3,&f)/JOINT_TURN_RAD)==-1.f);
    /* Same current pose returned on the negative raw branch, no double -360. */
    f.position_rad=-167.15f*RAD;f.received_count++;f.received_at_ms+=2;
    assert(fabsf(FiveHistory_Reference(3,&f)-(-167.15f*RAD))<2e-6f);
    empty();f.received_count=1;f.received_at_ms=clock_ms;
    f.position_rad=-520*RAD;
    assert(fabsf(FiveHistory_Reference(3,&f)-(-160*RAD))<2e-6f);
    /* Actual movement beyond a boundary must NOT be folded into the window. */
    f.position_rad=-539*RAD;f.received_count++;f.received_at_ms+=2;
    assert(fabsf(FiveHistory_Reference(3,&f)-(-179*RAD))<2e-6f);
    f.position_rad=-542*RAD;f.received_count++;f.received_at_ms+=2;
    assert(fabsf(FiveHistory_Reference(3,&f)-(-182*RAD))<2e-6f);
    f.position_rad=100*RAD;
    assert(FiveHistory_Reference(1,&f)>g_joint_reference_config[1].observed_upper_rad ||
           FiveHistory_Reference(1,&f)<g_joint_reference_config[1].observed_lower_rad);
}
int main(void)
{
    thresholds();restore_and_wrap();partial_axes_and_stale();
    torn_writes();rollover_and_erase_loss();foreign_and_missing();initial_legal_branch();
    puts("PASS: five-axis 150deg thresholds, initial/partial checkpoints, turn restore,"
         " live wrap, stale data, all torn-body/commit cuts, rollover/suspend,"
         " erase power loss, foreign-data preservation and absent chip.");
    return 0;
}
