#include "five_trajectory.h"
#include "five_workspace.h"
#include "motor_config.h"
#include <array>
#include <cmath>
#include <cstring>
#include <new>
#include <ruckig/ruckig.hpp>

namespace {
constexpr double rad=.01745329251994329577;
constexpr unsigned all_components=(1U<<15)-1U;
using Profile=ruckig::Trajectory<5>;
struct Engine {
    ruckig::Ruckig<5> planner{.002};
    ruckig::InputParameter<5> input;
    std::array<Profile,FIVE_TRAJECTORY_CAPACITY> segments;
};
static_assert(sizeof(Engine)<=sizeof(FiveTrajectory_State::engine));
static_assert(alignof(Engine)<=alignof(FiveTrajectory_State));
Engine& engine(FiveTrajectory_State *s)
{
    return *std::launder(reinterpret_cast<Engine *>(s->engine.bytes));
}
const Engine& engine(const FiveTrajectory_State *s)
{
    return *std::launder(reinterpret_cast<const Engine *>(s->engine.bytes));
}
uint32_t hash_word(uint32_t hash,uint32_t word)
{
    for (unsigned i=0;i<4;++i) { hash=(hash^((word>>(8*i))&255U))*16777619U; }
    return hash;
}
uint32_t point_hash(uint32_t hash,const FiveTrajectory_Point& point)
{
    hash=hash_word(hash,point.at_ms);
    for (unsigned d=0;d<3;++d) {
        const float *values=d==0 ? point.position_deg :
            d==1 ? point.velocity_rad_s : point.acceleration_rad_s2;
        for (unsigned i=0;i<5;++i) {
            uint32_t bits;std::memcpy(&bits,&values[i],4);hash=hash_word(hash,bits);
        }
    }
    return hash;
}
bool valid_point(const FiveTrajectory_Point& p)
{
    double q[5];
    for (unsigned i=0;i<5;++i) {
        const auto& limit=g_five_trajectory_limits[i];
        q[i]=p.position_deg[i];
        if (!std::isfinite(q[i]) || !std::isfinite(p.velocity_rad_s[i]) ||
            !std::isfinite(p.acceleration_rad_s2[i]) ||
            std::abs(p.velocity_rad_s[i])>limit.velocity_rad_s ||
            std::abs(p.velocity_rad_s[i])>g_motor_can_config[i].vmax ||
            std::abs(p.acceleration_rad_s2[i])>limit.acceleration_rad_s2) { return false; }
    }
    return FiveWorkspace_Line(q,q);
}
bool calculate(FiveTrajectory_State *s,const FiveTrajectory_Point& a,
               const FiveTrajectory_Point& b,unsigned index)
{
    auto& e=engine(s);
    auto& in=e.input;
    in.control_interface=ruckig::ControlInterface::Position;
    in.synchronization=ruckig::Synchronization::Time;
    in.duration_discretization=ruckig::DurationDiscretization::Discrete;
    in.minimum_duration=(b.at_ms-a.at_ms)*.001;
    for (unsigned i=0;i<5;++i) {
        const auto& limit=g_five_trajectory_limits[i];
        if (!std::isfinite(limit.velocity_rad_s) || limit.velocity_rad_s<=0 ||
            limit.velocity_rad_s>g_motor_can_config[i].vmax ||
            !std::isfinite(limit.acceleration_rad_s2) || limit.acceleration_rad_s2<=0 ||
            !std::isfinite(limit.jerk_rad_s3) || limit.jerk_rad_s3<=0) { return false; }
        in.current_position[i]=a.position_deg[i]*rad;
        in.current_velocity[i]=a.velocity_rad_s[i];
        in.current_acceleration[i]=a.acceleration_rad_s2[i];
        in.target_position[i]=b.position_deg[i]*rad;
        in.target_velocity[i]=b.velocity_rad_s[i];
        in.target_acceleration[i]=b.acceleration_rad_s2[i];
        in.max_velocity[i]=limit.velocity_rad_s;
        in.max_acceleration[i]=limit.acceleration_rad_s2;
        in.max_jerk[i]=limit.jerk_rad_s3;
    }
    /* Ruckig can brake an already-moving start back inside planning limits.
     * Do not reintroduce the removed instantaneous-rest admission gate. */
    if (!in.validate<false>(false,true) ||
        e.planner.calculate(in,e.segments[index])!=ruckig::Result::Working) { return false; }
    const double milliseconds=e.segments[index].get_duration()*1000;
    if (!std::isfinite(milliseconds) || milliseconds<1 || milliseconds>120001) { return false; }
    const auto duration=static_cast<uint32_t>(std::llround(milliseconds));
    if (duration<2 || duration>120000 || std::abs(milliseconds-duration)>1e-5 || duration%2) {
        return false;
    }
    std::array<ruckig::Bound,5> bounds;
    e.segments[index].get_position_extrema(bounds);
    double from[5],to[5],lo[5],hi[5];
    for (unsigned i=0;i<5;++i) {
        from[i]=a.position_deg[i];to[i]=b.position_deg[i];
        lo[i]=bounds[i].min/rad;hi[i]=bounds[i].max/rad;
    }
    if (!FiveWorkspace_TrajectoryBounds(from,to,lo,hi)) { return false; }
    s->segment_ms[index]=duration;
    return true;
}
}

void FiveTrajectory_EncodePoint(uint8_t bytes[64],const FiveTrajectory_Point *p)
{
    uint32_t words[16];words[0]=p->at_ms;
    for (unsigned i=0;i<15;++i) {
        const float *v=i<5 ? p->position_deg : i<10 ? p->velocity_rad_s : p->acceleration_rad_s2;
        std::memcpy(&words[i+1],&v[i%5],4);
    }
    for (unsigned i=0;i<64;++i) { bytes[i]=static_cast<uint8_t>(words[i/4]>>(8*(i%4))); }
}
void FiveTrajectory_DecodePoint(FiveTrajectory_Point *p,const uint8_t bytes[64])
{
    uint32_t words[16]{};
    for (unsigned i=0;i<64;++i) { words[i/4]|=static_cast<uint32_t>(bytes[i])<<(8*(i%4)); }
    p->at_ms=words[0];
    for (unsigned i=0;i<15;++i) {
        float *v=i<5 ? p->position_deg : i<10 ? p->velocity_rad_s : p->acceleration_rad_s2;
        std::memcpy(&v[i%5],&words[i+1],4);
    }
}
bool FiveTrajectory_ValidatePoint(const FiveTrajectory_Point *p,uint32_t previous,bool last)
{
    if (!p || p->at_ms>120000U || p->at_ms<previous+2U || !valid_point(*p)) { return false; }
    if (last) {
        if (p->at_ms<100U) { return false; }
        for (unsigned i=0;i<5;++i) {
            if (p->velocity_rad_s[i]!=0 || p->acceleration_rad_s2[i]!=0) { return false; }
        }
    }
    return true;
}
bool FiveTrajectory_RelativePoint(FiveTrajectory_Point *out,const FiveTrajectory_Point *point,
                                  const FiveTrajectory_Point *first,const double origin[5])
{
    if (!out || !point || !first || !origin) { return false; }
    FiveTrajectory_Point next=*point;
    for (unsigned i=0;i<5;++i) {
        const double q=origin[i]+(static_cast<double>(point->position_deg[i])-first->position_deg[i]);
        if (!std::isfinite(origin[i]) || !std::isfinite(q)) { return false; }
        next.position_deg[i]=static_cast<float>(q);
        if (!std::isfinite(next.position_deg[i])) { return false; }
    }
    *out=next;return true;
}
void FiveTrajectory_Cancel(FiveTrajectory_State *s)
{
    if (!s) { return; }
    s->uploading=s->active=s->prepared=false;s->expected=s->count=s->pending_mask=0;
    s->start_ms=s->checksum=s->duration_ms=0;
    std::memset(s->segment_ms,0,sizeof s->segment_ms);
}
bool FiveTrajectory_Begin(FiveTrajectory_State *s,unsigned count)
{
    if (!s || s->active || !count || count>FIVE_TRAJECTORY_CAPACITY) { return false; }
    FiveTrajectory_Cancel(s);
    if (!s->initialized) {
        ::new (static_cast<void *>(s->engine.bytes)) Engine{};
        s->initialized=true;
    }
    s->expected=count;s->uploading=true;s->checksum=2166136261U;
    return true;
}
bool FiveTrajectory_Set(FiveTrajectory_State *s,unsigned axis,unsigned derivative,float value)
{
    if (!s || !s->uploading || s->count>=s->expected || axis>=5 ||
        derivative>=3 || !std::isfinite(value)) { return false; }
    float *values=derivative==0 ? s->pending.position_deg :
        derivative==1 ? s->pending.velocity_rad_s : s->pending.acceleration_rad_s2;
    values[axis]=value;s->pending_mask|=1U<<(5*derivative+axis);
    return true;
}
bool FiveTrajectory_Append(FiveTrajectory_State *s,uint32_t at)
{
    if (!s || s->pending_mask!=all_components) { return false; }
    FiveTrajectory_Point next=s->pending;next.at_ms=at;
    return FiveTrajectory_AppendPoint(s,&next);
}
bool FiveTrajectory_AppendPoint(FiveTrajectory_State *s,const FiveTrajectory_Point *point)
{
    if (!s || !point || !s->uploading || s->count>=s->expected ||
        !FiveTrajectory_ValidatePoint(point,s->count ? s->points[s->count-1].at_ms : 0,
                                      s->count+1==s->expected)) { return false; }
    const FiveTrajectory_Point& next=*point;
    if (s->count && !calculate(s,s->points[s->count-1],next,s->count)) { return false; }
    s->points[s->count++]=next;s->checksum=point_hash(s->checksum,next);
    s->pending_mask=0;s->prepared=false;
    return true;
}
bool FiveTrajectory_PrepareStart(FiveTrajectory_State *s,const FiveMIT_State *x)
{
    if (!s || !x || !s->uploading || s->count!=s->expected || s->pending_mask ||
        !s->count || s->points[s->count-1].at_ms<100U) { return false; }
    s->prepared=false;
    FiveTrajectory_Point origin{};
    for (unsigned i=0;i<5;++i) {
        if (!std::isfinite(x->software_deg[i]) || !std::isfinite(x->velocity_rad_s[i])) {
            return false;
        }
        origin.position_deg[i]=static_cast<float>(x->software_deg[i]);
        origin.velocity_rad_s[i]=x->velocity_rad_s[i];
    }
    if (!calculate(s,origin,s->points[0],0)) { return false; }
    uint32_t total=0;
    for (unsigned i=0;i<s->count;++i) { total+=s->segment_ms[i]; }
    if (total>120000U) { return false; }
    s->origin=origin;s->duration_ms=total;s->prepared=true;
    return true;
}
bool FiveTrajectory_Start(FiveTrajectory_State *s,uint32_t now)
{
    if (!s || !s->uploading || !s->prepared || s->active) { return false; }
    s->start_ms=now;s->active=true;s->uploading=false;
    return true;
}
bool FiveTrajectory_Active(const FiveTrajectory_State *s) { return s && s->active; }
FiveTrajectory_Status FiveTrajectory_StatusNow(const FiveTrajectory_State *s)
{
    FiveTrajectory_Status out{};
    if (s) {
        out.expected=s->expected;out.count=s->count;out.start_ms=s->start_ms;
        out.duration_ms=s->duration_ms;out.checksum=s->checksum;
        out.uploading=s->uploading;out.active=s->active;
        uint32_t elapsed=0;
        for (unsigned i=0;i<s->count;++i) { elapsed+=s->segment_ms[i];out.knot_ms[i]=elapsed; }
        if (s->active) {
            std::memcpy(out.origin_deg,s->origin.position_deg,sizeof out.origin_deg);
            std::memcpy(out.origin_velocity_rad_s,s->origin.velocity_rad_s,
                        sizeof out.origin_velocity_rad_s);
        }
    }
    return out;
}
bool FiveTrajectory_Sample(const FiveTrajectory_State *s,uint32_t elapsed,FiveMIT_Reference *out)
{
    if (!s || !out || !s->active || !s->count) { return false; }
    unsigned index=0;uint32_t start=0;
    while (index+1<s->count && elapsed>start+s->segment_ms[index]) {
        start+=s->segment_ms[index++];
    }
    const auto& target=s->points[index];
    const FiveTrajectory_Point& from=index ? s->points[index-1] : s->origin;
    FiveMIT_Reference next{};
    if (elapsed==start || elapsed>=start+s->segment_ms[index]) {
        const auto& p=elapsed==start ? from : target;
        for (unsigned i=0;i<5;++i) {
            next.position_deg[i]=p.position_deg[i];next.velocity_rad_s[i]=p.velocity_rad_s[i];
            next.acceleration_rad_s2[i]=p.acceleration_rad_s2[i];
        }
    } else {
        std::array<double,5> q,v,a;
        engine(s).segments[index].at_time((elapsed-start)*.001,q,v,a);
        for (unsigned i=0;i<5;++i) {
            if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || !std::isfinite(a[i])) { return false; }
            next.position_deg[i]=q[i]/rad;next.velocity_rad_s[i]=static_cast<float>(v[i]);
            next.acceleration_rad_s2[i]=static_cast<float>(a[i]);
        }
    }
    *out=next;return true;
}
