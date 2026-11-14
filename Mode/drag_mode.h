#ifndef YLAMP_DRAG_MODE_H
#define YLAMP_DRAG_MODE_H

#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 拖拽模式单模块接口：
 * Enter -> Update -> Stop/Resume -> Reset；不读取时钟、不发送CAN、不操作电源。
 * 状态由调用者持有，Task负责有效反馈、使能确认、故障出口和实际命令提交。
 * 公共几何和刚体动力学归Model；本模块仅持有拖动控制行为。 */
typedef RobotModel_Geometry_t DragGeometry_t;

/* p=Rz(q1)*(origin+Ry(q2)*point)。不求逆；朝上秩退化也有界。 */
#define DragGeometry_Evaluate RobotModel_Geometry
/* D=J^T*diag(b_xy,b_xy,b_z)*J，返回D11/D12/D22。
 * 原Kd/角速度参数定义参考力臂处的手端阻尼/线速度。
 * v为关节速度；不含重力、惯量、摩擦、时钟或发送。 */
bool DragGeometry_Damping(const MotorTask_Config_t *config,
                          const DragGeometry_t *geometry, const float v[2],
                          float damping[3]);

typedef enum {
    DRAG_MODE_OFF = 0, DRAG_MODE_ENTERING, DRAG_MODE_DRAGGING,
    DRAG_MODE_STOPPING, DRAG_MODE_HOLDING, DRAG_MODE_RESUMING
} DragMode_Phase_t;

typedef struct {
    DragMode_Phase_t phase;
    float hold_target_rad[2];
    bool initialized;
    uint32_t sample_ms[2], sample_count[2];
    float velocity[2], acceleration[2];
    float sample_age_s[2];
    bool derivative_ready[2];
    float support_blend, hold_blend;
    float gravity_ff[2], dynamic_ff[2], kd[2];
} DragMode_State_t;

typedef struct {
    bool valid;
    DragMode_Phase_t phase;
    float q2_rad;
    float inertia[3]; /* A/B/D，名义 CAD 方向，未含未知驱动惯量。 */
    float gravity_nm[2];
    float velocity_coupling_nm[2]; /* 未缩放/不限幅的名义 c(q,dq)。 */
    float inertial_nm[2];          /* 未缩放/不限幅的名义 M*ddq_est。 */
    float dynamic_ff_nm[2];        /* 已缩放、限幅/限斜率的实际动态前馈请求。 */
    float grip_radius_m[2];
    float damping[3];              /* 实际D11/D12/D22；交叉项已按剩余FF预算缩放。 */
    float grip_ff_nm[2];           /* 仅交叉阻尼；不含/不改原有重力和动态项。 */
    float filtered_velocity_rad_s[2];
    float acceleration_rad_s2[2];
    float hold_blend;
} DragMode_Status_t;

void DragMode_Reset(DragMode_State_t *state);
bool DragMode_ValidConfig(const MotorTask_Config_t *config);
/* Enter only after both enable acknowledgements. No hardware access.
 * Stop captures actual positions once; Resume keeps these targets until Kp=0.
 * Reset is the fault/exit path; it never claims that hardware is disabled. */
bool DragMode_Enter(const MotorTask_Config_t *config,
                    const DM_Motor_Feedback_t feedback[2], DragMode_State_t *state);
bool DragMode_Stop(const MotorTask_Config_t *config,
                   const DM_Motor_Feedback_t feedback[2], DragMode_State_t *state);
bool DragMode_Resume(DragMode_State_t *state);

/* 纯计算，无 CAN/时钟/使能。调用者提供新鲜、有效、同一控制轮的两轴反馈。
 * OFF 仅计算预览，输出零前馈/零刚度；绝不替调用者发送。
 * dt_s 为真实控制步长；速度/加速度滤波仅在新反馈且采样时间前进时更新。
 * q2=参考圈角-用户朝上零点；不使用IMU。动态项按部署参数有限比例回灌。
 * 名义握持点的D=J^T*Bx*J：对角进入Kd，交叉项只用原FF剩余预算。
 * 两轴导数须有效且新鲜；建立支撑/停止/保持/释放保持不继续加速度反馈。
 * 成功后一起提交 state/status/commands；失败不修改输出。不可在 ISR 调用。 */
bool DragMode_Update(const MotorTask_Config_t *config,
                     const DM_Motor_Feedback_t feedback[2], float dt_s,
                     DragMode_State_t *state, DragMode_Status_t *status,
                     DM_Motor_MIT_Command_t commands[2]);

#ifdef __cplusplus
}
#endif
#endif
