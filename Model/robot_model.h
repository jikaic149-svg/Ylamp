#ifndef YLAMP_ROBOT_MODEL_H
#define YLAMP_ROBOT_MODEL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 当前机械模型明确为2轴，不因遥测可承载6轴而假装动力学已扩轴。
 * CAD: J1=+Z，J2=+Y，q2=0连杆朝上。SI单位；编码器极性仍待实机核验。 */
#define ROBOT_MODEL_JOINT_COUNT 2U

typedef struct {
    float a0, a_cos2, a_sin2; /* kg*m^2 */
    float b_cos, b_sin;
    float d;
    float gravity_sin_nm;
    float extra_inertia_kg_m2[2]; /* Output-side effective addition, separate from CAD. */
} RobotModel_Parameters_t;

typedef struct {
    float coulomb_nm, viscous_nm_s;
} RobotModel_Friction_t;

typedef struct {
    float j2_origin_m[3];
    float point_from_j2_m[3]; /* 固定在J2连杆上的工具/名义握持点。 */
} RobotModel_Point_t;

typedef struct {
    float position_m[3];
    float jacobian[3][2]; /* 平移雅可比，m/rad；列为J1/J2。 */
    float radius_m[2], reference_radius_m[2];
} RobotModel_Geometry_t;

typedef struct {
    float inertia[3]; /* 对称矩阵的A/B/D。 */
    float gravity_nm[2];
    float coupling_nm[2]; /* c(q,dq)，不含摩擦。 */
    float inertial_nm[2]; /* M(q)*ddq；ddq由调用模式决定来源。 */
} RobotModel_Dynamics_t;

/* 显式输入的纯物理计算；不读取Mode/config/硬件，不滤波、不限幅、不缩放。
 * q、dq、ddq已在同一CAD正向和rad/s制；失败不修改输出。 */
bool RobotModel_Dynamics(const RobotModel_Parameters_t *parameters, float q2,
                         const float dq[2], const float ddq[2],
                         RobotModel_Dynamics_t *output);
bool RobotModel_Geometry(const RobotModel_Point_t *point, float q1, float q2,
                         RobotModel_Geometry_t *output);
/* Effective Coulomb + viscous friction. Explicit smooth-zero regularization;
 * no observer, state, force estimate, gain change or automatic enable here. */
bool RobotModel_Friction(const RobotModel_Friction_t *parameters, float velocity_rad_s,
                         float smooth_velocity_rad_s, float *torque_nm);

extern const RobotModel_Parameters_t g_robot_model;
extern const RobotModel_Point_t g_robot_tool_point;
extern const RobotModel_Friction_t g_robot_friction[ROBOT_MODEL_JOINT_COUNT];

#ifdef __cplusplus
}
#endif
#endif
