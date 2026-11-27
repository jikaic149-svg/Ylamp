#include "lookaround_store.h"
#include "lookaround.h"
#include "five_history_config.h"
#include "motor_config.h"
#include "w25q64.h"
#include <string.h>

#define MAGIC 0x594c5031U
#define COMMIT 0x434f4d54U
enum { HEADER=64, IMAGE=HEADER+64*FIVE_TRAJECTORY_CAPACITY };
enum { IDLE, ACQUIRE, ERASE_WAIT, PROGRAM, VERIFY_BODY, COMMIT_WAIT, VERIFY_COMMIT, FAILED };
_Static_assert((unsigned)IMAGE<=(unsigned)W25Q64_SECTOR,"preset fits one sector");
_Static_assert(FIVE_LOOKAROUND_BYTES==2*W25Q64_SECTOR &&
    FIVE_LOOKAROUND_BASE+FIVE_LOOKAROUND_BYTES<=FIVE_HISTORY_BASE,"separate preset/history partitions");
static struct {
    LookaroundStore_Status s;
    FiveTrajectory_Point points[FIVE_TRAJECTORY_CAPACITY];
    uint8_t image[IMAGE];
    uint32_t signature,operation_at,previous_ms;
    unsigned active,destination,phase,cursor;
    bool initialized,blank[2],owned[2],locked;
} p;

static uint32_t get32(const uint8_t *b)
{ return (uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24; }
static void put32(uint8_t *b,uint32_t n)
{ for (unsigned i=0;i<4;++i) { b[i]=(uint8_t)(n>>(8*i)); } }
static uint32_t hash(uint32_t h,const uint8_t *b,unsigned n)
{ for (unsigned i=0;i<n;++i) { h=(h^b[i])*16777619U; }return h; }
static uint32_t crc(const uint8_t *b,unsigned n)
{
    uint32_t c=0xffffffffU;
    for (unsigned i=0;i<n;++i) {
        c^=b[i];
        for (unsigned j=0;j<8;++j) { c=(c>>1)^(0xedb88320U&(0U-(c&1U))); }
    }
    return ~c;
}
static uint32_t signature(void)
{
    uint32_t value=2166136261U;
    for (unsigned i=0;i<5;++i) {
        const MotorJointReference_Config_t *r=&g_joint_reference_config[i];
        const float fields[]={r->zero_rad,r->observed_lower_rad,r->observed_upper_rad,
                              (float)g_five_mit_hw[i].cad_offset_deg};
        for (unsigned j=0;j<4;++j) {
            uint32_t bits;uint8_t b[4];memcpy(&bits,&fields[j],4);put32(b,bits);value=hash(value,b,4);
        }
    }
    return value;
}
static uint32_t base(unsigned slot) { return FIVE_LOOKAROUND_BASE+slot*W25Q64_SECTOR; }
static unsigned image_size(void) { return HEADER+64*p.s.expected; }
static void failure(void)
{
    p.s.flags=(p.s.flags&~(LOOKAROUND_SAVING|LOOKAROUND_UPLOADING))|LOOKAROUND_IO_ERROR;
    ++p.s.errors;p.phase=FAILED;
}
static bool header_valid(const uint8_t *b)
{
    return get32(b)==MAGIC && get32(b+4)==1 && get32(b+12)>0 &&
        get32(b+12)<=FIVE_TRAJECTORY_CAPACITY && get32(b+16)==64*get32(b+12) &&
        get32(b+28)==p.signature && get32(b+56)==crc(b,56);
}
static bool body_valid(const uint8_t *b)
{
    const unsigned count=get32(b+12);
    uint32_t previous=0;
    for (unsigned i=0;i<count;++i) {
        FiveTrajectory_Point point;FiveTrajectory_DecodePoint(&point,b+HEADER+64*i);
        if (!FiveTrajectory_ValidatePoint(&point,previous,i+1==count)) { return false; }
        previous=point.at_ms;
    }
    return get32(b+20)==crc(b+HEADER,64*count) &&
        get32(b+24)==hash(2166136261U,b+HEADER,64*count);
}
static void publish(void)
{
    p.s.count=get32(p.image+12);p.s.sequence=get32(p.image+8);p.s.checksum=get32(p.image+24);
    for (unsigned i=0;i<p.s.count;++i) {
        FiveTrajectory_DecodePoint(&p.points[i],p.image+HEADER+64*i);
    }
    p.s.flags|=LOOKAROUND_LOADED;
}
const LookaroundStore_Status *LookaroundStore_StatusNow(void) { return &p.s; }
const FiveTrajectory_Point *LookaroundStore_Points(unsigned *count)
{
    if (count) { *count=p.s.count; }
    return (p.s.flags&LOOKAROUND_LOADED) && !(p.s.flags&LOOKAROUND_SAVING) ? p.points : NULL;
}
void LookaroundStore_Init(void)
{
    if (p.initialized) { return; }
    memset(&p,0,sizeof p);p.initialized=true;p.signature=signature();
    if (!W25Q64_Init()) { p.s.flags=LOOKAROUND_NO_CHIP;return; }
    bool found=false;
    for (unsigned slot=0;slot<2;++slot) {
        uint8_t block[256],header[HEADER];
        bool blank=true;
        for (unsigned at=0;at<W25Q64_SECTOR;at+=sizeof block) {
            if (!W25Q64_Read(base(slot)+at,block,sizeof block)) { failure();return; }
            if (!at) { memcpy(header,block,HEADER); }
            for (unsigned i=0;i<sizeof block;++i) { if (block[i]!=0xffU) { blank=false; } }
        }
        p.blank[slot]=blank;
        /* A fully formed own header, even before commit, witnesses ownership.
         * A torn/unrecognised initial header is not permission to format it. */
        p.owned[slot]=header_valid(header);
        if (!blank && !p.owned[slot]) { p.s.flags|=LOOKAROUND_FOREIGN; }
        if (!p.owned[slot] || get32(header+60)!=COMMIT) { continue; }
        memcpy(p.image,header,HEADER);
        const unsigned size=HEADER+get32(header+16);
        for (unsigned at=HEADER;at<size;) {
            const unsigned n=size-at>256 ? 256 : size-at;
            if (!W25Q64_Read(base(slot)+at,p.image+at,(uint16_t)n)) { failure();return; }
            at+=n;
        }
        if (body_valid(p.image) && (!found || (int32_t)(get32(header+8)-p.s.sequence)>0)) {
            found=true;p.active=slot;publish();
        }
    }
    p.s.flags|=LOOKAROUND_READY;
}
#ifdef YLAMP_HOST_FIXTURES
void LookaroundStore_ResetForTest(void) { memset(&p,0,sizeof p);W25Q64_ResetLockForTest(); }
#endif
bool LookaroundStore_Begin(unsigned count)
{
    if (!(p.s.flags&LOOKAROUND_READY) || (p.s.flags&LOOKAROUND_IO_ERROR) ||
        p.phase!=IDLE || !count || count>FIVE_TRAJECTORY_CAPACITY) { return false; }
    if (p.s.flags&LOOKAROUND_LOADED) { p.destination=1U-p.active; }
    else { p.destination=p.blank[0] || p.owned[0] ? 0 : 1; }
    if (!p.blank[p.destination] && !p.owned[p.destination]) { return false; }
    p.s.expected=count;p.s.staged_count=0;p.s.staged_checksum=2166136261U;p.previous_ms=0;
    p.s.flags|=LOOKAROUND_UPLOADING;memset(p.image,0xff,sizeof p.image);return true;
}
bool LookaroundStore_Append(unsigned index,const uint8_t bytes[64])
{
    if (!(p.s.flags&LOOKAROUND_UPLOADING) || index!=p.s.staged_count ||
        index>=p.s.expected || !bytes) { return false; }
    FiveTrajectory_Point point;FiveTrajectory_DecodePoint(&point,bytes);
    if (!FiveTrajectory_ValidatePoint(&point,p.previous_ms,index+1==p.s.expected)) { return false; }
    memcpy(p.image+HEADER+64*index,bytes,64);p.previous_ms=point.at_ms;
    p.s.staged_checksum=hash(p.s.staged_checksum,bytes,64);++p.s.staged_count;return true;
}
void LookaroundStore_CancelUpload(void)
{
    if (p.s.flags&LOOKAROUND_UPLOADING) {
        p.s.flags&=~LOOKAROUND_UPLOADING;p.s.expected=p.s.staged_count=0;
    }
}
bool LookaroundStore_Save(uint32_t checksum)
{
    if (!(p.s.flags&LOOKAROUND_UPLOADING) || p.s.staged_count!=p.s.expected ||
        checksum!=p.s.staged_checksum) { return false; }
    put32(p.image,MAGIC);put32(p.image+4,1);put32(p.image+8,p.s.sequence+1);
    put32(p.image+12,p.s.expected);put32(p.image+16,64*p.s.expected);
    put32(p.image+20,crc(p.image+HEADER,64*p.s.expected));
    put32(p.image+24,checksum);put32(p.image+28,p.signature);put32(p.image+56,crc(p.image,56));
    p.s.flags=(p.s.flags&~LOOKAROUND_UPLOADING)|LOOKAROUND_SAVING;
    p.phase=ACQUIRE;p.cursor=0;return true;
}
bool LookaroundStore_Read(unsigned offset,uint8_t *bytes,unsigned length)
{
    if (!bytes || !length || length>224 || offset>FIVE_LOOKAROUND_BYTES ||
        length>FIVE_LOOKAROUND_BYTES-offset || p.locked ||
        !W25Q64_TryLock(W25Q64_LOOKAROUND)) { return false; }
    uint8_t sr1=0,sr2=0;
    const bool ok=W25Q64_Status(&sr1,&sr2) && !(sr1&1U) && !(sr2&0x80U) &&
        W25Q64_Read(FIVE_LOOKAROUND_BASE+offset,bytes,(uint16_t)length);
    W25Q64_Unlock(W25Q64_LOOKAROUND);return ok;
}
void LookaroundStore_Run(uint32_t now)
{
    if (p.phase==IDLE) { return; }
    if (!p.locked) {
        if (p.phase!=ACQUIRE || !W25Q64_TryLock(W25Q64_LOOKAROUND)) { return; }
        p.locked=true;
    }
    uint8_t sr1=0,sr2=0;
    if (!W25Q64_Status(&sr1,&sr2)) { failure();return; }
    if (p.phase==FAILED) {
        /* Finish ownership cleanup only; no retry/re-enable/erase recovery. */
        if (!(sr1&1U) && !(sr2&0x80U)) { W25Q64_Unlock(W25Q64_LOOKAROUND);p.locked=false; }
        return;
    }
    if ((sr1&1U) || (sr2&0x80U)) {
        const unsigned timeout=p.phase==ERASE_WAIT ? FIVE_LOOKAROUND_ERASE_TIMEOUT_MS :
                                                                  FIVE_LOOKAROUND_PAGE_TIMEOUT_MS;
        if (p.phase==ACQUIRE || now-p.operation_at>timeout) { failure(); }
        return;
    }
    if (p.phase==ACQUIRE) {
        if (!p.blank[p.destination]) {
            if (!W25Q64_Erase(base(p.destination))) { failure();return; }
            p.operation_at=now;p.phase=ERASE_WAIT;return;
        }
        p.phase=PROGRAM;
    } else if (p.phase==ERASE_WAIT) { p.phase=PROGRAM; }
    if (p.phase==PROGRAM) {
        if (p.cursor<image_size()) {
            unsigned n=image_size()-p.cursor;if (n>256) { n=256; }
            if (!W25Q64_Program(base(p.destination)+p.cursor,p.image+p.cursor,(uint16_t)n)) {
                failure();return;
            }
            p.cursor+=n;p.operation_at=now;return;
        }
        p.cursor=0;p.phase=VERIFY_BODY;
    } else if (p.phase==COMMIT_WAIT) { p.cursor=0;p.phase=VERIFY_COMMIT; }
    if (p.phase==VERIFY_BODY || p.phase==VERIFY_COMMIT) {
        uint8_t check[256];
        unsigned n=image_size()-p.cursor;if (n>256) { n=256; }
        if (!W25Q64_Read(base(p.destination)+p.cursor,check,(uint16_t)n) ||
            memcmp(check,p.image+p.cursor,n)) { failure();return; }
        p.cursor+=n;
        if (p.cursor!=image_size()) { return; }
        if (p.phase==VERIFY_BODY) {
            put32(p.image+60,COMMIT);
            if (!W25Q64_Program(base(p.destination)+60,p.image+60,4)) { failure();return; }
            p.phase=COMMIT_WAIT;p.operation_at=now;return;
        }
        if (!header_valid(p.image) || !body_valid(p.image)) { failure();return; }
        publish();p.active=p.destination;p.owned[p.active]=true;p.blank[p.active]=false;
        p.s.flags&=~LOOKAROUND_SAVING;p.phase=IDLE;
        W25Q64_Unlock(W25Q64_LOOKAROUND);p.locked=false;
    }
}
