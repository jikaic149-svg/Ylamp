#ifndef YLAMP_FIVE_CONTROL_H
#define YLAMP_FIVE_CONTROL_H
#include "five_mit.h"
#include "motor_config.h"

enum { FIVE_ALL = (1U << FIVE_AXES) - 1U };
typedef enum {
    FIVE_WAIT, FIVE_GRAVITY, FIVE_MOTION, FIVE_POSITION, FIVE_DEGRADED,
    FIVE_IMPEDANCE, FIVE_REVERSE_J3, FIVE_DRAG /* Append: preserve existing USB phase values. */
} FivePhase;
enum {
    FIVE_BAD_FEEDBACK=1U, FIVE_BAD_STATE=2U, FIVE_HOT=4U, FIVE_RANGE=8U,
    FIVE_TIMING=16U, FIVE_LIMIT=32U, FIVE_TX=64U, FIVE_TRACK=128U
};
typedef struct {
    FivePhase phase;
    uint8_t fresh, enabled, submitted, limited;
    uint32_t fault, gravity_at, ticks;
    bool gravity_valid;
    DM_Motor_Feedback_t feedback[FIVE_AXES];
    float reference_rad[FIVE_AXES], gravity_nm[FIVE_AXES]; /* G support: J2 candidate + existing J3 trim. */
} FiveControl_Status;

/* Sole motor owner, ordinary app task. All setters are explicit, never from USB ISR. */
bool FiveControl_Init(DM_Motor_t *const motors[FIVE_AXES], uint32_t now);
void FiveControl_Tick(uint32_t now);
const FiveControl_Status *FiveControl_StatusNow(void);
BSP_CAN_Status_t FiveControl_Move(const double target_deg[FIVE_AXES],uint32_t duration_ms);
/* Same owner/gates/effort/hold/exit as Move; finite single-axis velocity PI.
 * joint=1..5, nonzero signed rad/s; duration_ms >2*ramp_ms >0.
 * Captures all five start positions in the owner, integrates the speed reference
 * analytically and checks its endpoint against original travel. No rest gate. */
BSP_CAN_Status_t FiveControl_Speed(unsigned joint,float velocity_rad_s,
                                  uint32_t duration_ms,uint32_t ramp_ms);
BSP_CAN_Status_t FiveControl_Drag(void); /* Select healthy manual idle; idempotent, no enable. */
void FiveControl_Gravity(void); /* Explicit pure-G comparison; cancel without enable/disable. */
/* End external task ownership: cancel motion/hold into manual idle; preserve
 * existing DRAG or explicitly selected pure G. Never clear faults or enable. */
void FiveControl_Release(void);
BSP_CAN_Status_t FiveControl_KeyPress(void); /* Explicit cold -> DRAG; DRAG <-> entry-pose hold. */
#ifdef YLAMP_J3_REVERSE_TEST
BSP_CAN_Status_t FiveControl_ReverseJ3(void); /* Supported, healthy pure G only; no enable. */
#endif
BSP_CAN_Status_t FiveControl_Acknowledge(void); /* Explicit fault clear -> manual idle; no motion/enable. */
/* Explicit, supported legacy-J3 acquisition after a controller-only restart:
 * only J3 has never replied; the other four are fresh, enabled and stationary;
 * no gravity vector or fault other than missing feedback. Neutral -> one FC ->
 * fresh feedback -> existing gravity submission. Once per boot, no fault clear,
 * disable, reset, trajectory or automatic retry. */
BSP_CAN_Status_t FiveControl_ReadLegacyJ3(void);
/* Operator-confirmed *already disabled*, unloaded cold commissioning only.
 * This is not a live takeover. No reset, disable, fault clear or automatic retry. */
BSP_CAN_Status_t FiveControl_ColdStart(void);
#endif
