#include "motor_config.h"
#include "joint_coordinates.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2;

static float radians(float degrees)
{
    return degrees * 0.01745329251994329577f;
}

int main(void)
{
    const float expected[][4] = {
        {-88.7282485f, -201.6521075f, 201.6521075f, MOTOR_REFERENCE_MIDPOINT},
        {146.180992f, -35.377224f, 46.564378f, MOTOR_REFERENCE_USER_APPROVED},
        {17.291375f, -86.610664f, 115.456767f, MOTOR_REFERENCE_USER_APPROVED},
        {55.7024265f, -235.7024265f, 294.2975735f, MOTOR_REFERENCE_USER_APPROVED},
        {-57.024737f, -22.632856f, 22.632856f, MOTOR_REFERENCE_MIDPOINT}
    };
    for (unsigned i = 0U; i < MOTOR_CAN_JOINT_COUNT; ++i) {
        const MotorJointReference_Config_t *r = &g_joint_reference_config[i];
        assert(r->kind == (uint8_t)expected[i][3]);
        assert(fabsf(r->zero_rad - radians(expected[i][0])) < 1e-6f);
        const float lower = JointCoordinates_Angle(r->observed_lower_rad, r->zero_rad);
        const float upper = JointCoordinates_Angle(r->observed_upper_rad, r->zero_rad);
        assert(fabsf(lower - radians(expected[i][1])) < 1e-6f);
        assert(fabsf(upper - radians(expected[i][2])) < 1e-6f);
        assert(JointCoordinates_Angle(r->zero_rad, r->zero_rad) == 0.0f);
        assert(fabsf((upper - lower) -
            (r->observed_upper_rad - r->observed_lower_rad)) < 1e-6f);
        assert(JointCoordinates_Angle(r->observed_upper_rad + 1.0f, r->zero_rad) > upper);
        if (i != 0U && i != 3U) {
            for (int turn = -2; turn <= 2; ++turn) {
                const float raw = r->zero_rad + turn * JOINT_TURN_RAD;
                assert(fabsf(JointCoordinates_Reference(raw, r->observed_lower_rad,
                    r->observed_upper_rad) - r->zero_rad) < 2e-6f);
            }
            assert(fabsf(JointCoordinates_Reference(r->zero_rad + 0.1f,
                r->observed_lower_rad, r->observed_upper_rad) - r->zero_rad - 0.1f) < 1e-6f);
        } else {
            assert(r->observed_upper_rad - r->observed_lower_rad > JOINT_TURN_RAD);
            assert(JointCoordinates_Reference(r->observed_upper_rad,
                r->observed_lower_rad, r->observed_upper_rad) == r->observed_upper_rad);
        }
        if (r->kind == MOTOR_REFERENCE_MIDPOINT) {
            assert(fabsf(lower + upper) < 1e-6f);
        }
    }
    assert(isnan(JointCoordinates_Angle(NAN, 0.0f)));
    assert(isnan(JointCoordinates_Angle(0.0f, INFINITY)));
    assert(isnan(JointCoordinates_Angle(INFINITY, 0.0f)));
    const MotorJointReference_Config_t *j4 = &g_joint_reference_config[3];
    assert(fabsf(j4->observed_lower_rad-radians(-180.f))<1e-6f);
    assert(fabsf(j4->observed_upper_rad-radians(350.f))<1e-6f);
    assert(j4->zero_rad>j4->observed_lower_rad && j4->zero_rad<j4->observed_upper_rad);
    const float j4_raw[]={radians(-180.f),radians(179.12854f),radians(350.f)};
    for (unsigned i=0;i<sizeof j4_raw/sizeof j4_raw[0];++i) {
        assert(JointCoordinates_Reference(j4_raw[i],j4->observed_lower_rad,
                                         j4->observed_upper_rad)==j4_raw[i]);
    }
    const MotorJointReference_Config_t *j2 = &g_joint_reference_config[1];
    const float before = JointCoordinates_Reference(3.1414f,
        j2->observed_lower_rad, j2->observed_upper_rad);
    const float after = JointCoordinates_Reference(-3.1414f,
        j2->observed_lower_rad, j2->observed_upper_rad);
    assert(fabsf(after - before) < 0.001f);
    assert(isnan(JointCoordinates_Reference(NAN, 0, 1)));
    assert(isnan(JointCoordinates_Reference(0, 1, -1)));
    const float raw = 3.11f, anchor = -3.17f;
    assert(fabsf(raw + JointCoordinates_Offset(raw, anchor) - anchor) < .01f);
    assert(MOTOR_CAN_JOINT_COUNT == 5U);
    puts("Joint references: five measured windows / calibrated multiturn J1 PASS");
    return 0;
}
