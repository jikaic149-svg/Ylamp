#include "robot_model.h"

#include <math.h>
#include <stddef.h>

bool RobotModel_Friction(const RobotModel_Friction_t *p, float velocity,
                         float smooth_velocity, float *torque_nm)
{
    if (p == NULL || torque_nm == NULL || !isfinite(velocity) ||
        !isfinite(smooth_velocity) || smooth_velocity <= 0.0f ||
        !isfinite(p->coulomb_nm) || p->coulomb_nm < 0.0f ||
        !isfinite(p->viscous_nm_s) || p->viscous_nm_s < 0.0f) { return false; }
    const float result = p->coulomb_nm * tanhf(velocity / smooth_velocity) +
        p->viscous_nm_s * velocity;
    if (!isfinite(result)) { return false; }
    *torque_nm = result;
    return true;
}

bool RobotModel_Dynamics(const RobotModel_Parameters_t *p, float q,
                         const float v[2], const float acc[2],
                         RobotModel_Dynamics_t *output)
{
    if (p == NULL || v == NULL || acc == NULL || output == NULL ||
        !isfinite(q) || !isfinite(v[0]) || !isfinite(v[1]) ||
        !isfinite(acc[0]) || !isfinite(acc[1]) ||
        !isfinite(p->extra_inertia_kg_m2[0]) || p->extra_inertia_kg_m2[0] < 0.0f ||
        !isfinite(p->extra_inertia_kg_m2[1]) || p->extra_inertia_kg_m2[1] < 0.0f) {
        return false;
    }
    const float s = sinf(q), c = cosf(q), s2 = sinf(2.0f*q), c2 = cosf(2.0f*q);
    const float a = p->a0 + p->extra_inertia_kg_m2[0] + p->a_cos2*c2 + p->a_sin2*s2;
    const float d = p->d + p->extra_inertia_kg_m2[1];
    const float b = p->b_cos*c + p->b_sin*s;
    const float ap = -2.0f*p->a_cos2*s2 + 2.0f*p->a_sin2*c2;
    const float bp = -p->b_cos*s + p->b_sin*c;
    const RobotModel_Dynamics_t result = {
        .inertia = {a, b, d},
        .gravity_nm = {0.0f, p->gravity_sin_nm*s},
        .coupling_nm = {ap*v[0]*v[1] + bp*v[1]*v[1], -0.5f*ap*v[0]*v[0]},
        .inertial_nm = {a*acc[0] + b*acc[1], b*acc[0] + d*acc[1]}
    };
    if (!isfinite(a) || !isfinite(b) || !isfinite(d) ||
        a <= 0.0f || p->d <= 0.0f || d <= 0.0f || a*d <= b*b) { return false; }
    for (unsigned i = 0U; i < 2U; ++i) {
        if (!isfinite(result.gravity_nm[i]) || !isfinite(result.coupling_nm[i]) ||
            !isfinite(result.inertial_nm[i])) { return false; }
    }
    *output = result;
    return true;
}

bool RobotModel_Geometry(const RobotModel_Point_t *point, float q1, float q2,
                         RobotModel_Geometry_t *output)
{
    if (point == NULL || output == NULL || !isfinite(q1) || !isfinite(q2)) {
        return false;
    }
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!isfinite(point->j2_origin_m[i]) ||
            !isfinite(point->point_from_j2_m[i])) { return false; }
    }
    const float *g = point->point_from_j2_m, *o = point->j2_origin_m;
    const float c = cosf(q2), s = sinf(q2), cy = cosf(q1), sy = sinf(q1);
    const float dx = c*g[0] + s*g[2], dz = -s*g[0] + c*g[2];
    const float x = o[0] + dx, y = o[1] + g[1], z = o[2] + dz;
    const float r2 = hypotf(g[0], g[2]);
    const RobotModel_Geometry_t result = {
        .position_m = {cy*x - sy*y, sy*x + cy*y, z},
        .jacobian = {{-cy*y - sy*x, cy*dz},
                     {-sy*y + cy*x, sy*dz}, {0.0f, -dx}},
        .radius_m = {hypotf(x, y), r2},
        .reference_radius_m = {hypotf(fabsf(o[0]) + r2, y), r2}
    };
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!isfinite(result.position_m[i]) ||
            !isfinite(result.jacobian[i][0]) ||
            !isfinite(result.jacobian[i][1])) { return false; }
    }
    if (!isfinite(result.reference_radius_m[0]) ||
        result.reference_radius_m[0] < 0.001f || r2 < 0.001f) { return false; }
    *output = result;
    return true;
}
