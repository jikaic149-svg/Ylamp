#include "joint_trajectory.h"
#include "robot_model.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

void TrajectoryTest_BeginMemoryAudit(void);
unsigned TrajectoryTest_EndMemoryAudit(void);
void TrajectoryTest_CompareOfficial(const JointTrajectory_Input_t *input, double duration);

_Static_assert(JOINT_TRAJECTORY_AXES == ROBOT_MODEL_JOINT_COUNT,
               "Explicitly adapt trajectory and model together when adding joints");

static JointTrajectory_Input_t make_input(double distance1, double distance2)
{
    JointTrajectory_Input_t input = {0};
    input.axis[0].position = -.25;
    input.axis[1].position = .2;
    input.axis[0].target_position = -.25 + distance1;
    input.axis[1].target_position = .2 + distance2;
    for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
        input.axis[i].max_velocity = .8 + .1*i;
        input.axis[i].max_acceleration = 1.2 + .2*i;
        input.axis[i].max_jerk = 4.0 + i;
    }
    return input;
}

static double plan(const JointTrajectory_Input_t *input)
{
    double duration = -123.0;
    assert(JointTrajectory_Calculate(input, &duration) == JOINT_TRAJECTORY_OK);
    assert(isfinite(duration) && duration >= 0.0);
    assert(fabs(duration/.005 - round(duration/.005)) < 1e-8);
    return duration;
}

static void check_curve(const JointTrajectory_Input_t *input)
{
    const double duration = plan(input);
    TrajectoryTest_CompareOfficial(input, duration);
    JointTrajectory_Sample_t start = {0}, end = {0}, after = {0};
    assert(JointTrajectory_Sample(0.0, &start) >= 0);
    assert(JointTrajectory_Sample(duration, &end) == JOINT_TRAJECTORY_FINISHED);
    assert(JointTrajectory_Sample(duration + 10.0, &after) == JOINT_TRAJECTORY_FINISHED);
    assert(memcmp(&end, &after, sizeof(end)) == 0);
    for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
        assert(fabs(start.position[i] - input->axis[i].position) < 1e-8);
        assert(fabs(start.velocity[i] - input->axis[i].velocity) < 1e-8);
        assert(fabs(start.acceleration[i] - input->axis[i].acceleration) < 1e-8);
        assert(fabs(end.position[i] - input->axis[i].target_position) < 1e-8);
        assert(fabs(end.velocity[i] - input->axis[i].target_velocity) < 1e-8);
        assert(fabs(end.acceleration[i] - input->axis[i].target_acceleration) < 1e-8);
    }
    for (unsigned step = 0U; step <= 400U; ++step) {
        JointTrajectory_Sample_t sample = {0};
        const double t = duration * step / 400.0;
        assert(JointTrajectory_Sample(t, &sample) >= 0);
        assert(sample.time_s == t);
        float v[2], a[2];
        for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
            assert(fabs(sample.velocity[i]) <= input->axis[i].max_velocity + 1e-8);
            assert(fabs(sample.acceleration[i]) <= input->axis[i].max_acceleration + 1e-8);
            assert(fabs(sample.jerk[i]) <= input->axis[i].max_jerk + 1e-8);
            v[i] = (float)sample.velocity[i];
            a[i] = (float)sample.acceleration[i];
        }
        /* Organic boundary: planned accelerations feed the common model.
         * No DragMode filters/feedback cancellation or hardware in this test. */
        RobotModel_Dynamics_t dynamics;
        assert(RobotModel_Dynamics(&g_robot_model, (float)sample.position[1],
                                    v, a, &dynamics));
        for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
            const float ff = dynamics.inertial_nm[i] +
                dynamics.coupling_nm[i] + dynamics.gravity_nm[i];
            assert(isfinite(ff));
        }
        const double h = 1e-5;
        if (t > h && t + h < duration) {
            JointTrajectory_Sample_t left = {0}, right = {0};
            assert(JointTrajectory_Sample(t-h, &left) == JOINT_TRAJECTORY_OK);
            assert(JointTrajectory_Sample(t+h, &right) == JOINT_TRAJECTORY_OK);
            for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
                assert(fabs((right.position[i]-left.position[i])/(2*h) -
                            sample.velocity[i]) < 1e-7);
                /* jerk is piecewise constant, not continuous at switches. */
                if (left.jerk[i] == sample.jerk[i] && right.jerk[i] == sample.jerk[i]) {
                    assert(fabs((right.velocity[i]-left.velocity[i])/(2*h) -
                                sample.acceleration[i]) < 1e-7);
                    assert(fabs((right.acceleration[i]-left.acceleration[i])/(2*h) -
                                sample.jerk[i]) < 1e-7);
                }
            }
        }
    }
}

static void invalid_inputs(void)
{
    JointTrajectory_Input_t input = make_input(.5, -.2);
    JointTrajectory_Sample_t output, saved;
    memset(&output, 0x5a, sizeof(output));
    saved = output;
    (void)plan(&input);
    assert(JointTrajectory_Sample(NAN, &output) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Sample(INFINITY, &output) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Sample(-.001, &output) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Sample(0.0, NULL) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(memcmp(&saved, &output, sizeof(output)) == 0);

    /* Each numeric field on either axis, not just the first malformed value. */
    const double bad[] = {NAN, INFINITY, -INFINITY};
    for (unsigned axis = 0U; axis < JOINT_TRAJECTORY_AXES; ++axis) {
        for (unsigned field = 0U; field < 9U; ++field) {
            for (unsigned value = 0U; value < 3U; ++value) {
                JointTrajectory_Input_t broken = input;
                double *fields[] = {
                    &broken.axis[axis].position, &broken.axis[axis].velocity,
                    &broken.axis[axis].acceleration, &broken.axis[axis].target_position,
                    &broken.axis[axis].target_velocity, &broken.axis[axis].target_acceleration,
                    &broken.axis[axis].max_velocity, &broken.axis[axis].max_acceleration,
                    &broken.axis[axis].max_jerk
                };
                *fields[field] = bad[value];
                double duration = -42.0;
                assert(JointTrajectory_Calculate(&broken, &duration) ==
                       JOINT_TRAJECTORY_INVALID_INPUT);
                assert(duration == -42.0);
                assert(JointTrajectory_Sample(0.0, &output) == JOINT_TRAJECTORY_NOT_READY);
                assert(memcmp(&saved, &output, sizeof(output)) == 0);
            }
        }
    }
    for (unsigned field = 0U; field < 3U; ++field) {
        for (unsigned negative = 0U; negative < 2U; ++negative) {
            JointTrajectory_Input_t broken = input;
            double *limits[] = {&broken.axis[0].max_velocity,
                                &broken.axis[0].max_acceleration, &broken.axis[0].max_jerk};
            *limits[field] = negative ? -1.0 : 0.0;
            double duration = -42.0;
            assert(JointTrajectory_Calculate(&broken, &duration) ==
                   JOINT_TRAJECTORY_INVALID_INPUT);
            assert(duration == -42.0);
        }
    }
    double duration = -42.0;
    assert(JointTrajectory_Calculate(NULL, &duration) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Calculate(&input, NULL) == JOINT_TRAJECTORY_INVALID_INPUT);
    input.axis[0].target_velocity = input.axis[0].max_velocity * 2;
    assert(JointTrajectory_Calculate(&input, &duration) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(duration == -42.0);
    assert(JointTrajectory_Sample(0.0, &output) == JOINT_TRAJECTORY_NOT_READY);
    assert(memcmp(&saved, &output, sizeof(output)) == 0);
}

int main(void)
{
    TrajectoryTest_BeginMemoryAudit();
    JointTrajectory_Input_t input = make_input(.8, -.4);
    JointTrajectory_Sample_t sample = {0};
    double duration = -1.0;
    assert(JointTrajectory_Calculate(&input, &duration) == JOINT_TRAJECTORY_NOT_READY);
    assert(JointTrajectory_Sample(0.0, &sample) == JOINT_TRAJECTORY_NOT_READY);
    assert(JointTrajectory_Init(0.0) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Init(NAN) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Init(-.005) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Init(INFINITY) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Init(.005) == JOINT_TRAJECTORY_OK);
    double low[2] = {123, 456}, high[2] = {789, 987};
    assert(JointTrajectory_PositionBounds(low, high) == JOINT_TRAJECTORY_NOT_READY);
    assert(low[0] == 123 && high[1] == 987);
    for (unsigned n = 0; n < 16; ++n) {
        input = make_input(0, 0);
        input.axis[0].velocity = n & 1U ? -.4 : .3;
        input.axis[1].velocity = n & 2U ? -.1 : .2;
        input.axis[0].acceleration = n & 4U ? -.2 : .25;
        input.axis[1].acceleration = n & 8U ? -.15 : .1;
        assert(JointTrajectory_Brake(&input, &duration) == JOINT_TRAJECTORY_OK);
        assert(JointTrajectory_PositionBounds(low, high) == JOINT_TRAJECTORY_OK);
        assert(JointTrajectory_PositionBounds(NULL, high) == JOINT_TRAJECTORY_INVALID_INPUT);
        assert(JointTrajectory_Sample(0, &sample) == JOINT_TRAJECTORY_OK);
        for (unsigned i = 0; i < 2; ++i) {
            assert(fabs(sample.position[i] - input.axis[i].position) < 1e-8);
            assert(fabs(sample.velocity[i] - input.axis[i].velocity) < 1e-8);
            assert(fabs(sample.acceleration[i] - input.axis[i].acceleration) < 1e-8);
        }
        for (unsigned k = 0; k <= 400; ++k) {
            assert(JointTrajectory_Sample(duration * k / 400, &sample) >= 0);
            for (unsigned i = 0; i < 2; ++i) {
                assert(sample.position[i] >= low[i] - 1e-8 &&
                       sample.position[i] <= high[i] + 1e-8);
                assert(fabs(sample.jerk[i]) <= input.axis[i].max_jerk + 1e-8);
            }
        }
        assert(JointTrajectory_Sample(duration, &sample) == JOINT_TRAJECTORY_FINISHED);
        for (unsigned i = 0; i < 2; ++i) {
            assert(fabs(sample.velocity[i]) < 1e-8);
            assert(fabs(sample.acceleration[i]) < 1e-8);
        }
    }
    input = make_input(.1, -.1);
    duration = 123;
    input.axis[0].max_jerk = NAN;
    assert(JointTrajectory_Brake(&input, &duration) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(duration == 123);
    assert(JointTrajectory_PositionBounds(low, high) == JOINT_TRAJECTORY_NOT_READY);

    const double moves[] = {0.0, 1e-6, .001, .05, .8, 2.0, -.3, -1.5};
    for (unsigned i = 0U; i < sizeof(moves)/sizeof(moves[0]); ++i) {
        for (unsigned j = 0U; j < sizeof(moves)/sizeof(moves[0]); ++j) {
            input = make_input(moves[i], moves[j]);
            check_curve(&input);
        }
    }
    input = make_input(.7, -.4);
    input.axis[0].velocity = .15;
    input.axis[0].acceleration = .1;
    input.axis[1].velocity = -.1;
    input.axis[1].acceleration = .15;
    check_curve(&input);
    input.axis[0].target_velocity = .05;
    input.axis[0].target_acceleration = -.1;
    check_curve(&input);

    /* Explicit replanning preserves the sampled p/v/a at the new start. */
    duration = plan(&input);
    assert(JointTrajectory_Sample(duration*.4, &sample) == JOINT_TRAJECTORY_OK);
    for (unsigned i = 0U; i < JOINT_TRAJECTORY_AXES; ++i) {
        input.axis[i].position = sample.position[i];
        input.axis[i].velocity = sample.velocity[i];
        input.axis[i].acceleration = sample.acceleration[i];
        input.axis[i].target_position -= .3;
        input.axis[i].target_velocity = 0.0;
        input.axis[i].target_acceleration = 0.0;
    }
    check_curve(&input);

    /* An initially overspeed state is recoverable, not a new runtime trip. */
    input = make_input(1.5, -.2);
    input.axis[0].velocity = 1.6;
    duration = plan(&input);
    assert(JointTrajectory_Sample(0.0, &sample) == JOINT_TRAJECTORY_OK);
    assert(fabs(sample.velocity[0] - 1.6) < 1e-8);
    assert(JointTrajectory_Sample(duration, &sample) == JOINT_TRAJECTORY_FINISHED);
    assert(fabs(sample.velocity[0]) < 1e-8);

    invalid_inputs();
    input = make_input(.1, -.1);
    (void)plan(&input);
    JointTrajectory_Reset();
    assert(JointTrajectory_Sample(0.0, &sample) == JOINT_TRAJECTORY_NOT_READY);
    (void)plan(&input);
    assert(JointTrajectory_Init(.01) == JOINT_TRAJECTORY_OK);
    assert(JointTrajectory_Sample(0.0, &sample) == JOINT_TRAJECTORY_NOT_READY);
    assert(JointTrajectory_Calculate(&input, &duration) == JOINT_TRAJECTORY_OK);
    assert(fabs(duration/.01 - round(duration/.01)) < 1e-8);
    assert(JointTrajectory_Init(NAN) == JOINT_TRAJECTORY_INVALID_INPUT);
    assert(JointTrajectory_Sample(0.0, &sample) == JOINT_TRAJECTORY_NOT_READY);
    assert(TrajectoryTest_EndMemoryAudit() == 0U);
    return 0;
}
