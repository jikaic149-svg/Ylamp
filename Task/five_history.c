#include "five_history.h"
#include "five_history_config.h"
#include "joint_coordinates.h"
#include "w25q64.h"
#include <math.h>
#include <string.h>

#define RAD .01745329251994329577f
#define MAGIC 0x594c4831U
#define COMMIT 0x434f4d54U
enum { RECORD=64, SLOTS=W25Q64_SECTOR/RECORD };
_Static_assert(FIVE_HISTORY_BYTES==2*W25Q64_SECTOR,"two-sector journal");
_Static_assert(FIVE_HISTORY_BASE+FIVE_HISTORY_BYTES==0x7fe000U,"preserve IMU bias");
static struct {
    FiveHistory_Status s;
    struct { float reference,velocity,offset;uint32_t count,at;bool seen; } axis[5];
    uint32_t signature,operation_at,erase_at,suspend_at;
    unsigned active,cursor[2],write_phase,erase_phase,erase_sector;
    bool blank[2],initialized;
    uint8_t image[RECORD];
} h;

static uint32_t get32(const uint8_t *p)
{ return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static void put32(uint8_t *p,uint32_t v)
{ for (unsigned i=0;i<4;++i) { p[i]=(uint8_t)(v>>(8*i)); } }
static float getfloat(const uint8_t *p)
{ float v;uint32_t bits=get32(p);memcpy(&v,&bits,4);return v; }
static void putfloat(uint8_t *p,float v)
{ uint32_t bits;memcpy(&bits,&v,4);put32(p,bits); }
static uint32_t crc(const uint8_t *p,unsigned n)
{
    uint32_t value=0xffffffffU;
    for (unsigned i=0;i<n;++i) {
        value^=p[i];
        for (unsigned j=0;j<8;++j) {
            value=(value>>1)^(0xedb88320U & (0U-(value&1U)));
        }
    }
    return ~value;
}
static uint32_t configuration(void)
{
    uint8_t bytes[4+5*36]={0};
    const int32_t initial[5]=FIVE_HISTORY_INITIAL_TURNS;
    put32(bytes,FIVE_HISTORY_INITIAL_TURN_MASK);
    for (unsigned i=0;i<5;++i) {
        uint8_t *p=bytes+4+i*36;
        const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
        const DM_Motor_Config_t *c=&g_motor_can_config[i];
        putfloat(p,r->zero_rad);putfloat(p+4,r->observed_lower_rad);
        putfloat(p+8,r->observed_upper_rad);putfloat(p+12,c->pmax);
        putfloat(p+16,c->vmax);putfloat(p+20,c->tmax);
        put32(p+24,c->can_id|((uint32_t)c->master_id<<8));
        put32(p+28,(uint32_t)c->protocol|((uint32_t)c->model<<8));
        put32(p+32,(uint32_t)initial[i]);
    }
    return crc(bytes,sizeof bytes);
}
static bool erased(const uint8_t *p)
{
    for (unsigned i=0;i<RECORD;++i) { if (p[i]!=0xffU) { return false; } }
    return true;
}
static bool valid(const uint8_t *p)
{
    if (get32(p)!=MAGIC || get32(p+4)!=1U || get32(p+60)!=COMMIT ||
        get32(p+56)!=crc(p,56) || !get32(p+16) || (get32(p+16)&~31U)) { return false; }
    for (unsigned i=0;i<5;++i) {
        if ((get32(p+16)&(1U<<i)) && !isfinite(getfloat(p+20+4*i))) { return false; }
    }
    return true;
}
static uint32_t address(unsigned sector,unsigned slot)
{ return FIVE_HISTORY_BASE+sector*W25Q64_SECTOR+slot*RECORD; }
static void failure(uint32_t reason)
{
    h.s.flags=(h.s.flags&~(FIVE_HISTORY_READY|FIVE_HISTORY_WRITING|FIVE_HISTORY_ERASING))|reason;
    ++h.s.errors;
}
const FiveHistory_Status *FiveHistory_StatusNow(void) { return &h.s; }

void FiveHistory_Init(void)
{
    if (h.initialized) { return; }
    memset(&h,0,sizeof h);h.initialized=true;
    const float step[5]=FIVE_HISTORY_STEP_DEG;
    memcpy(h.s.step_deg,step,sizeof step);
    for (unsigned i=0;i<5;++i) {
        h.s.saved_rad[i]=NAN;
        if (!isfinite(step[i]) || step[i]<=0 || step[i]>=180) {
            failure(FIVE_HISTORY_IO_ERROR);return;
        }
    }
    h.signature=configuration();
    if (!W25Q64_Init()) { failure(FIVE_HISTORY_NO_CHIP);return; }
    bool found=false,nonempty=false;
    uint8_t latest[RECORD]={0};
    for (unsigned sector=0;sector<2;++sector) {
        h.blank[sector]=true;
        for (unsigned slot=0;slot<SLOTS;++slot) {
            uint8_t image[RECORD];
            if (!W25Q64_Read(address(sector,slot),image,RECORD)) {
                failure(FIVE_HISTORY_IO_ERROR);return;
            }
            if (!erased(image)) {
                h.blank[sector]=false;nonempty=true;h.cursor[sector]=slot+1;
            }
            if (valid(image) && (!found || (int32_t)(get32(image+8)-h.s.sequence)>0)) {
                found=true;h.s.sequence=get32(image+8);h.active=sector;
                memcpy(latest,image,RECORD);
            }
        }
    }
    /* A valid own commit witnesses this partition's prior enrollment.
     * With no own commit, nonblank bytes are never automatically erased. */
    if (!found && nonempty) { failure(FIVE_HISTORY_FOREIGN);return; }
    if (found && get32(latest+12)==h.signature) {
        h.s.saved_mask=(uint8_t)get32(latest+16);
        for (unsigned i=0;i<5;++i) { h.s.saved_rad[i]=getfloat(latest+20+4*i); }
        h.s.flags|=FIVE_HISTORY_LOADED;
    }
    h.s.flags|=FIVE_HISTORY_READY;
}
#ifdef YLAMP_HOST_FIXTURES
void FiveHistory_ResetForTest(void) { memset(&h,0,sizeof h); }
#endif

float FiveHistory_Reference(unsigned i,const DM_Motor_Feedback_t *f)
{
    if (i>=5 || !f || !f->received_count || !isfinite(f->position_rad) ||
        !isfinite(f->velocity_rad_s)) { return NAN; }
    const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
    if (!h.initialized) {
        return JointCoordinates_Reference(f->position_rad,r->observed_lower_rad,r->observed_upper_rad);
    }
    if (h.axis[i].seen && h.axis[i].count==f->received_count) { return h.axis[i].reference; }
    float reference=f->position_rad;
    if (!h.axis[i].seen) {
        if (h.s.saved_mask&(1U<<i)) {
            reference+=JointCoordinates_Offset(reference,h.s.saved_rad[i]);
            h.s.restored_mask|=(uint8_t)(1U<<i);
        } else {
            reference=JointCoordinates_Reference(reference,r->observed_lower_rad,r->observed_upper_rad);
            if (FIVE_HISTORY_INITIAL_TURN_MASK&(1U<<i)) {
                const int32_t turns[5]=FIVE_HISTORY_INITIAL_TURNS;
                const float phase=f->position_rad-
                    floorf(f->position_rad/JOINT_TURN_RAD)*JOINT_TURN_RAD;
                const float seeded=phase+turns[i]*JOINT_TURN_RAD;
                if (seeded>=r->observed_lower_rad && seeded<=r->observed_upper_rad) {
                    reference=seeded;
                }
            }
        }
        /* User-requested INITIAL branch selection, not a running clamp:
         * keep an already legal branch; otherwise choose a legal WHOLE-turn
         * equivalent if one exists. No solution leaves a genuine violation
         * visible. Once running, never fold a real travel-end crossing back. */
        if (reference<r->observed_lower_rad || reference>r->observed_upper_rad) {
            const float low=ceilf((r->observed_lower_rad-f->position_rad)/JOINT_TURN_RAD);
            const float high=floorf((r->observed_upper_rad-f->position_rad)/JOINT_TURN_RAD);
            if (low<=high) {
                const float turns=roundf((reference-f->position_rad)/JOINT_TURN_RAD);
                reference=f->position_rad+fminf(high,fmaxf(low,turns))*JOINT_TURN_RAD;
            }
        }
    } else {
        reference+=h.axis[i].offset;
        const uint32_t elapsed=f->received_at_ms-h.axis[i].at;
        if (elapsed && elapsed<=100U) {
            const float dt=elapsed*.001f;
            const float predicted=h.axis[i].reference+h.axis[i].velocity*dt;
            const float correction=JointCoordinates_Offset(reference,predicted);
            /* Only a near-whole-turn renumbering may change the offset.
             * This is not a position-error limit or an all-axis rest gate. */
            const float tolerance=g_motor_can_config[i].vmax*dt+
                4.f*g_motor_can_config[i].pmax/65535.f;
            if (correction!=0 && fabsf(reference+correction-predicted)<=tolerance) {
                reference+=correction;++h.s.corrections[i];
            }
        }
    }
    h.axis[i].offset=reference-f->position_rad;
    h.axis[i].reference=reference;h.axis[i].velocity=f->velocity_rad_s;
    h.axis[i].at=f->received_at_ms;h.axis[i].count=f->received_count;h.axis[i].seen=true;
    return reference;
}
static unsigned fresh(uint32_t now)
{
    unsigned mask=0;
    for (unsigned i=0;i<5;++i) {
        if (h.axis[i].seen && now-h.axis[i].at<=100U) { mask|=1U<<i; }
    }
    return mask;
}
static bool due(unsigned mask)
{
    for (unsigned i=0;i<5;++i) {
        if ((mask&(1U<<i)) && (!(h.s.saved_mask&(1U<<i)) ||
            fabsf(h.axis[i].reference-h.s.saved_rad[i])>=h.s.step_deg[i]*RAD)) { return true; }
    }
    return false;
}
static void make_record(uint32_t now,unsigned mask)
{
    memset(h.image,0xff,sizeof h.image);
    put32(h.image,MAGIC);put32(h.image+4,1);put32(h.image+8,h.s.sequence+1);
    put32(h.image+12,h.signature);put32(h.image+16,mask|h.s.saved_mask);
    for (unsigned i=0;i<5;++i) {
        putfloat(h.image+20+4*i,(mask&(1U<<i)) ? h.axis[i].reference : h.s.saved_rad[i]);
    }
    put32(h.image+40,now);put32(h.image+56,crc(h.image,56));
    put32(h.image+60,COMMIT);
}
void FiveHistory_Run(uint32_t now)
{
    if (!(h.s.flags&FIVE_HISTORY_READY)) { return; }
    uint8_t sr1=0,sr2=0;
    if (!W25Q64_Status(&sr1,&sr2)) { failure(FIVE_HISTORY_IO_ERROR);return; }
    const unsigned mask=fresh(now);
    const bool pending=due(mask);
    if (h.write_phase) {
        if (now-h.operation_at>FIVE_HISTORY_PAGE_TIMEOUT_MS) { failure(FIVE_HISTORY_IO_ERROR);return; }
        if (sr1&1U) { return; }
        const uint32_t at=address(h.active,h.cursor[h.active]);
        if (h.write_phase==1) {
            if (!W25Q64_Program(at+60,h.image+60,4)) { failure(FIVE_HISTORY_IO_ERROR);return; }
            h.write_phase=2;h.operation_at=now;return;
        }
        uint8_t check[RECORD];
        if (!W25Q64_Read(at,check,RECORD) || memcmp(check,h.image,RECORD) || !valid(check)) {
            failure(FIVE_HISTORY_IO_ERROR);return;
        }
        h.s.sequence=get32(check+8);h.s.saved_mask=(uint8_t)get32(check+16);
        for (unsigned i=0;i<5;++i) { h.s.saved_rad[i]=getfloat(check+20+4*i); }
        h.s.committed_at_ms=now;++h.s.writes;
        h.blank[h.active]=false;++h.cursor[h.active];h.write_phase=0;
        h.s.flags&=~FIVE_HISTORY_WRITING;
        if (h.erase_phase==2) {
            if (!W25Q64_Resume()) { failure(FIVE_HISTORY_IO_ERROR);return; }
            h.erase_phase=1;
        }
        return;
    }
    if (h.erase_phase) {
        if (now-h.erase_at>FIVE_HISTORY_ERASE_TIMEOUT_MS) { failure(FIVE_HISTORY_IO_ERROR);return; }
        if (!(sr1&1U) && !(sr2&0x80U)) {
            h.cursor[h.erase_sector]=0;h.blank[h.erase_sector]=true;h.erase_phase=0;
            h.s.flags&=~FIVE_HISTORY_ERASING;
        } else if (h.erase_phase==1) {
            if (pending && h.cursor[h.active]<SLOTS) {
                if (!W25Q64_Suspend()) { failure(FIVE_HISTORY_IO_ERROR);return; }
                h.erase_phase=3;h.suspend_at=now;
            }
            return;
        } else if (h.erase_phase==3) {
            if ((sr2&0x80U) && !(sr1&1U)) { h.erase_phase=2; }
            else {
                if (now-h.suspend_at>10U) { failure(FIVE_HISTORY_IO_ERROR); }
                return;
            }
        }
    } else if ((sr1&1U) || (sr2&0x80U)) {
        failure(FIVE_HISTORY_IO_ERROR);return;
    }
    if (pending) {
        if (h.cursor[h.active]>=SLOTS) {
            const unsigned next=1U-h.active;
            if (!h.blank[next]) {
                if (!h.erase_phase) {
                    if (!W25Q64_Erase(address(next,0))) { failure(FIVE_HISTORY_IO_ERROR);return; }
                    h.erase_sector=next;h.erase_phase=1;h.erase_at=now;
                    h.s.flags|=FIVE_HISTORY_ERASING;
                }
                return;
            }
            h.active=next;
        }
        make_record(now,mask);
        if (!W25Q64_Program(address(h.active,h.cursor[h.active]),h.image,60)) {
            failure(FIVE_HISTORY_IO_ERROR);return;
        }
        h.write_phase=1;h.operation_at=now;h.s.flags|=FIVE_HISTORY_WRITING;
    } else if (!h.erase_phase && !h.blank[1U-h.active] && h.cursor[h.active]>0) {
        /* Reclaim the OLDER sector only after a verified new commit.
         * New checkpoints suspend this background erase, then resume it. */
        h.erase_sector=1U-h.active;
        if (!W25Q64_Erase(address(h.erase_sector,0))) { failure(FIVE_HISTORY_IO_ERROR);return; }
        h.erase_phase=1;h.erase_at=now;h.s.flags|=FIVE_HISTORY_ERASING;
    }
}
