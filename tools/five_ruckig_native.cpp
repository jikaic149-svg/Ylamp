/* Host-only bridge: same five-axis Mode/config/Ruckig as the MCU, no device IO.
 * Caller owns aligned zero-initialized state memory; never copies a live state. */
#include "five_trajectory.h"
#include "motor_config.h"
#include <cstddef>
#include <cstring>
#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C" __attribute__((visibility("default")))
#endif
FDCAN_HandleTypeDef hfdcan1{},hfdcan2{};
EXPORT std::size_t FiveRuckig_StateBytes() { return sizeof(FiveTrajectory_State); }
EXPORT uint32_t FiveRuckig_Version() { return FIVE_TRAJECTORY_VERSION; }
EXPORT bool FiveRuckig_Prepare(void *memory,const double origin[5],const double velocity[5],
    const FiveTrajectory_Point *points,unsigned count,uint32_t *duration,uint32_t *knots)
{
    if (!memory || !origin || !velocity || !points || !duration || !knots) { return false; }
    auto *s=static_cast<FiveTrajectory_State *>(memory);
    if (!FiveTrajectory_Begin(s,count)) { return false; }
    for (unsigned k=0;k<count;++k) {
        for (unsigned d=0;d<3;++d) {
            const float *v=d==0 ? points[k].position_deg :
                d==1 ? points[k].velocity_rad_s : points[k].acceleration_rad_s2;
            for (unsigned i=0;i<5;++i) { if (!FiveTrajectory_Set(s,i,d,v[i])) { return false; } }
        }
        if (!FiveTrajectory_Append(s,points[k].at_ms)) { return false; }
    }
    FiveMIT_State x{};std::memcpy(x.software_deg,origin,sizeof x.software_deg);
    for (unsigned i=0;i<5;++i) { x.velocity_rad_s[i]=static_cast<float>(velocity[i]); }
    if (!FiveTrajectory_PrepareStart(s,&x) || !FiveTrajectory_Start(s,0)) { return false; }
    const auto status=FiveTrajectory_StatusNow(s);
    *duration=status.duration_ms;std::memcpy(knots,status.knot_ms,count*sizeof(uint32_t));
    return true;
}
EXPORT bool FiveRuckig_SampleBatch(const void *memory,const uint32_t *ms,std::size_t count,
                                  double *q,double *v,double *a)
{
    if (!memory || !ms || !q || !v || !a) { return false; }
    for (std::size_t k=0;k<count;++k) {
        FiveMIT_Reference sample{};
        if (!FiveTrajectory_Sample(static_cast<const FiveTrajectory_State *>(memory),ms[k],&sample)) {
            return false;
        }
        for (unsigned i=0;i<5;++i) {
            q[5*k+i]=sample.position_deg[i];v[5*k+i]=sample.velocity_rad_s[i];
            a[5*k+i]=sample.acceleration_rad_s2[i];
        }
    }
    return true;
}
EXPORT void FiveRuckig_Limits(double *limits)
{
    for (unsigned i=0;i<5;++i) {
        limits[3*i]=g_five_trajectory_limits[i].velocity_rad_s;
        limits[3*i+1]=g_five_trajectory_limits[i].acceleration_rad_s2;
        limits[3*i+2]=g_five_trajectory_limits[i].jerk_rad_s3;
    }
}
