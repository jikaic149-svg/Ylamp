#include "robot_model.h"
#include "joint_coordinates.h"

#include <assert.h>
#include <math.h>
#include <string.h>

int main(void)
{
    for (unsigned joint = 0; joint < ROBOT_MODEL_JOINT_COUNT; ++joint) {
        float positive, negative, zero;
        assert(RobotModel_Friction(&g_robot_friction[joint], .3f, .02f, &positive));
        assert(RobotModel_Friction(&g_robot_friction[joint], -.3f, .02f, &negative));
        assert(RobotModel_Friction(&g_robot_friction[joint], 0.0f, .02f, &zero));
        assert(positive > 0.0f && fabsf(positive + negative) < 1e-7f && zero == 0.0f);
        float saved = positive;
        assert(!RobotModel_Friction(&g_robot_friction[joint], NAN, .02f, &positive));
        assert(positive == saved);
        assert(!RobotModel_Friction(&g_robot_friction[joint], .3f, 0.0f, &positive));
        RobotModel_Friction_t invalid = {.coulomb_nm = -1.0f};
        assert(!RobotModel_Friction(&invalid, .3f, .02f, &positive));
        assert(positive == saved);
    }
    const float v[2] = {.7f, -.4f}, acc[2] = {-.3f, .9f};
    for (int n = -30; n <= 30; ++n) {
        const float q1 = .31f, q2 = (float)n * .1f, h = .001f;
        RobotModel_Dynamics_t d;
        RobotModel_Geometry_t g, plus, minus;
        assert(RobotModel_Dynamics(&g_robot_model, q2, v, acc, &d));
        assert(d.inertia[0] * d.inertia[2] > d.inertia[1] * d.inertia[1]);
        assert(fabsf(d.inertial_nm[0] -
            (d.inertia[0]*acc[0] + d.inertia[1]*acc[1])) < 1e-7f);
        assert(fabsf(d.gravity_nm[1] - g_robot_model.gravity_sin_nm*sinf(q2)) < 1e-7f);
        RobotModel_Parameters_t cad = g_robot_model;
        cad.extra_inertia_kg_m2[0] = cad.extra_inertia_kg_m2[1] = 0.0f;
        RobotModel_Dynamics_t nominal;
        assert(RobotModel_Dynamics(&cad, q2, v, acc, &nominal));
        for (unsigned joint = 0U; joint < 2U; ++joint) {
            assert(fabsf(d.inertial_nm[joint] - nominal.inertial_nm[joint] -
                g_robot_model.extra_inertia_kg_m2[joint]*acc[joint]) < 1e-7f);
            assert(d.coupling_nm[joint] == nominal.coupling_nm[joint]);
        }
        assert(RobotModel_Geometry(&g_robot_tool_point, q1, q2, &g));
        for (unsigned axis = 0U; axis < 2U; ++axis) {
            assert(RobotModel_Geometry(&g_robot_tool_point,
                q1 + (axis == 0U ? h : 0.0f), q2 + (axis == 1U ? h : 0.0f), &plus));
            assert(RobotModel_Geometry(&g_robot_tool_point,
                q1 - (axis == 0U ? h : 0.0f), q2 - (axis == 1U ? h : 0.0f), &minus));
            for (unsigned row = 0U; row < 3U; ++row) {
                assert(fabsf((plus.position_m[row] - minus.position_m[row])/(2.0f*h) -
                    g.jacobian[row][axis]) < .00003f);
            }
        }
    }
    RobotModel_Dynamics_t d = {0}, saved;
    const float zero[2] = {0};
    assert(RobotModel_Dynamics(&g_robot_model, 0.0f, zero, zero, &d));
    assert(d.gravity_nm[0] == 0.0f && d.gravity_nm[1] == 0.0f);
    saved = d;
    assert(!RobotModel_Dynamics(&g_robot_model, NAN, v, acc, &d));
    assert(memcmp(&d, &saved, sizeof(d)) == 0);
    RobotModel_Parameters_t invalid = g_robot_model;
    invalid.extra_inertia_kg_m2[0] = -0.01f;
    assert(!RobotModel_Dynamics(&invalid, .1f, v, acc, &d));
    assert(memcmp(&d, &saved, sizeof(d)) == 0);
    invalid = g_robot_model;
    invalid.d = -1.0f;
    assert(!RobotModel_Dynamics(&invalid, .1f, v, acc, &d));
    assert(memcmp(&d, &saved, sizeof(d)) == 0);
    const float raw = 3.11f, anchor = -3.17f;
    assert(fabsf(raw + JointCoordinates_Offset(raw, anchor) - anchor) < .01f);
    return 0;
}
