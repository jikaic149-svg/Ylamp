#include "joint_trajectory.h"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <new>
#include <ruckig/ruckig.hpp>

namespace {
bool tracking;
unsigned heap_calls;
void record_heap_call() { if (tracking) { ++heap_calls; } }
}

extern "C" void * __real_malloc(std::size_t);
extern "C" void * __real_calloc(std::size_t, std::size_t);
extern "C" void * __real_realloc(void *, std::size_t);
extern "C" void __real_free(void *);

extern "C" void * __wrap_malloc(std::size_t n)
{
    record_heap_call();
    return __real_malloc(n);
}
extern "C" void * __wrap_calloc(std::size_t n, std::size_t size)
{
    record_heap_call();
    return __real_calloc(n, size);
}
extern "C" void * __wrap_realloc(void *p, std::size_t n)
{
    record_heap_call();
    return __real_realloc(p, n);
}
extern "C" void __wrap_free(void *p)
{
    if (p != nullptr) { record_heap_call(); }
    __real_free(p);
}

// Also intercept libstdc++'s entry points, including when the host uses its DLL.
void * operator new(std::size_t n)
{
    record_heap_call();
    void *p = std::malloc(n == 0U ? 1U : n);
    if (p == nullptr) { std::abort(); }
    return p;
}
void * operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }

extern "C" void TrajectoryTest_BeginMemoryAudit(void)
{
    heap_calls = 0U;
    tracking = true;
}

extern "C" unsigned TrajectoryTest_EndMemoryAudit(void)
{
    tracking = false;
    return heap_calls;
}

/* Independently call the official API, checking that C conversion, time
 * synchronization and terminal sampling have not changed its results. */
extern "C" void TrajectoryTest_CompareOfficial(const JointTrajectory_Input_t *in,
                                              double duration)
{
    ruckig::Ruckig<JOINT_TRAJECTORY_AXES> planner(.005);
    ruckig::InputParameter<JOINT_TRAJECTORY_AXES> input;
    ruckig::Trajectory<JOINT_TRAJECTORY_AXES> trajectory;
    input.synchronization = ruckig::Synchronization::Time;
    input.duration_discretization = ruckig::DurationDiscretization::Discrete;
    for (std::size_t i = 0; i < JOINT_TRAJECTORY_AXES; ++i) {
        const auto& a = in->axis[i];
        input.current_position[i] = a.position;
        input.current_velocity[i] = a.velocity;
        input.current_acceleration[i] = a.acceleration;
        input.target_position[i] = a.target_position;
        input.target_velocity[i] = a.target_velocity;
        input.target_acceleration[i] = a.target_acceleration;
        input.max_velocity[i] = a.max_velocity;
        input.max_acceleration[i] = a.max_acceleration;
        input.max_jerk[i] = a.max_jerk;
    }
    assert(planner.calculate(input, trajectory) == ruckig::Result::Working);
    assert(duration == trajectory.get_duration());
    for (unsigned step = 0U; step <= 60U; ++step) {
        const double t = duration * step / 60.0;
        std::array<double, JOINT_TRAJECTORY_AXES> p {}, v {}, a {}, j {};
        std::size_t section {};
        trajectory.at_time(t, p, v, a, j, section);
        JointTrajectory_Sample_t actual {};
        assert(JointTrajectory_Sample(t, &actual) >= 0);
        for (std::size_t i = 0; i < JOINT_TRAJECTORY_AXES; ++i) {
            assert(std::abs(actual.position[i] - p[i]) < 1e-12);
            assert(std::abs(actual.velocity[i] - v[i]) < 1e-12);
            assert(std::abs(actual.acceleration[i] - a[i]) < 1e-12);
            assert(std::abs(actual.jerk[i] - j[i]) < 1e-12);
        }
    }
}
