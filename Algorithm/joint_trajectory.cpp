#include "joint_trajectory.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <new>

#include <ruckig/ruckig.hpp>

namespace {

constexpr std::size_t axes = JOINT_TRAJECTORY_AXES;
using Planner = ruckig::Ruckig<axes, ruckig::StandardVector, false>;

struct Context {
    Planner planner;
    ruckig::InputParameter<axes> input;
    ruckig::Trajectory<axes> trajectory;
    bool ready {false};
};

/* ponytail: one Motion owner needs one planner; add caller-owned contexts only
 * if a second independent trajectory actually needs to coexist. */
alignas(Context) std::byte storage[sizeof(Context)];
Context *context;

static_assert(JOINT_TRAJECTORY_OK == static_cast<int>(ruckig::Result::Working));
static_assert(JOINT_TRAJECTORY_INVALID_INPUT ==
              static_cast<int>(ruckig::Result::ErrorInvalidInput));
static_assert(JOINT_TRAJECTORY_SYNCHRONIZATION_ERROR ==
              static_cast<int>(ruckig::Result::ErrorSynchronizationCalculation));

bool valid_axis(const JointTrajectory_Axis_t& axis)
{
    return std::isfinite(axis.position) && std::isfinite(axis.velocity) &&
        std::isfinite(axis.acceleration) && std::isfinite(axis.target_position) &&
        std::isfinite(axis.target_velocity) && std::isfinite(axis.target_acceleration) &&
        std::isfinite(axis.max_velocity) && axis.max_velocity > 0.0 &&
        std::isfinite(axis.max_acceleration) && axis.max_acceleration > 0.0 &&
        std::isfinite(axis.max_jerk) && axis.max_jerk > 0.0;
}

} // namespace

extern "C" JointTrajectory_Result_t JointTrajectory_Init(double period_s)
{
    JointTrajectory_Reset();
    if (!std::isfinite(period_s) || period_s <= 0.0) {
        return JOINT_TRAJECTORY_INVALID_INPUT;
    }
    if (context == nullptr) {
        context = ::new (static_cast<void *>(storage)) Context {};
    }
    context->planner.delta_time = period_s;
    context->input.control_interface = ruckig::ControlInterface::Position;
    context->input.synchronization = ruckig::Synchronization::Time;
    context->input.duration_discretization = ruckig::DurationDiscretization::Discrete;
    return JOINT_TRAJECTORY_OK;
}

extern "C" void JointTrajectory_Reset(void)
{
    if (context != nullptr) {
        context->ready = false;
    }
}

static JointTrajectory_Result_t calculate(
    const JointTrajectory_Input_t *input, double *duration_s, bool braking)
{
    JointTrajectory_Reset();
    if (context == nullptr) {
        return JOINT_TRAJECTORY_NOT_READY;
    }
    if (input == nullptr || duration_s == nullptr) {
        return JOINT_TRAJECTORY_INVALID_INPUT;
    }
    for (std::size_t i = 0; i < axes; ++i) {
        if (!valid_axis(input->axis[i])) {
            return JOINT_TRAJECTORY_INVALID_INPUT;
        }
    }
    for (std::size_t i = 0; i < axes; ++i) {
        const auto& a = input->axis[i];
        context->input.current_position[i] = a.position;
        context->input.current_velocity[i] = a.velocity;
        context->input.current_acceleration[i] = a.acceleration;
        context->input.target_position[i] = a.target_position;
        context->input.target_velocity[i] = a.target_velocity;
        context->input.target_acceleration[i] = a.target_acceleration;
        context->input.max_velocity[i] = a.max_velocity;
        context->input.max_acceleration[i] = a.max_acceleration;
        context->input.max_jerk[i] = a.max_jerk;
    }
    context->input.control_interface = braking ?
        ruckig::ControlInterface::Velocity : ruckig::ControlInterface::Position;
    /* Shortest per-axis braking avoids stretching one joint's stopping travel.
     * The same clock still samples both; an already stopped joint holds. */
    context->input.synchronization = braking ?
        ruckig::Synchronization::None : ruckig::Synchronization::Time;
    if (braking) {
        context->input.target_velocity.fill(0.0);
        context->input.target_acceleration.fill(0.0);
    }
    const auto result = context->planner.calculate(context->input, context->trajectory);
    if (result != ruckig::Result::Working) {
        return static_cast<JointTrajectory_Result_t>(result);
    }
    const double duration = context->trajectory.get_duration();
    if (!std::isfinite(duration) || duration < 0.0) {
        return JOINT_TRAJECTORY_ERROR;
    }
    *duration_s = duration;
    context->ready = true;
    return JOINT_TRAJECTORY_OK;
}

extern "C" JointTrajectory_Result_t JointTrajectory_Calculate(
    const JointTrajectory_Input_t *input, double *duration_s)
{
    return calculate(input, duration_s, false);
}

extern "C" JointTrajectory_Result_t JointTrajectory_Brake(
    const JointTrajectory_Input_t *input, double *duration_s)
{
    return calculate(input, duration_s, true);
}

extern "C" JointTrajectory_Result_t JointTrajectory_PositionBounds(
    double minimum[axes], double maximum[axes])
{
    if (context == nullptr || !context->ready) { return JOINT_TRAJECTORY_NOT_READY; }
    if (minimum == nullptr || maximum == nullptr) {
        return JOINT_TRAJECTORY_INVALID_INPUT;
    }
    std::array<ruckig::Bound, axes> bounds {};
    context->trajectory.get_position_extrema(bounds);
    for (const auto& bound : bounds) {
        if (!std::isfinite(bound.min) || !std::isfinite(bound.max)) {
            return JOINT_TRAJECTORY_ERROR;
        }
    }
    for (std::size_t i = 0; i < axes; ++i) {
        minimum[i] = bounds[i].min;
        maximum[i] = bounds[i].max;
    }
    return JOINT_TRAJECTORY_OK;
}

extern "C" JointTrajectory_Result_t JointTrajectory_Sample(
    double time_s, JointTrajectory_Sample_t *output)
{
    if (context == nullptr || !context->ready) {
        return JOINT_TRAJECTORY_NOT_READY;
    }
    if (output == nullptr || !std::isfinite(time_s) || time_s < 0.0) {
        return JOINT_TRAJECTORY_INVALID_INPUT;
    }
    const double duration = context->trajectory.get_duration();
    const double t = time_s < duration ? time_s : duration;
    std::array<double, axes> position {}, velocity {}, acceleration {}, jerk {};
    std::size_t section {};
    context->trajectory.at_time(t, position, velocity, acceleration, jerk, section);
    JointTrajectory_Sample_t sample {};
    sample.time_s = t;
    for (std::size_t i = 0; i < axes; ++i) {
        if (!std::isfinite(position[i]) || !std::isfinite(velocity[i]) ||
            !std::isfinite(acceleration[i]) || !std::isfinite(jerk[i])) {
            return JOINT_TRAJECTORY_ERROR;
        }
        sample.position[i] = position[i];
        sample.velocity[i] = velocity[i];
        sample.acceleration[i] = acceleration[i];
        sample.jerk[i] = jerk[i];
    }
    *output = sample;
    return time_s >= duration ? JOINT_TRAJECTORY_FINISHED : JOINT_TRAJECTORY_OK;
}
