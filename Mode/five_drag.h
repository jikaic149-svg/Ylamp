#ifndef YLAMP_FIVE_DRAG_H
#define YLAMP_FIVE_DRAG_H
#include "five_mit.h"

/* Manual dragging only. Ratios are fractions of the existing total friction
 * map, never an additional full friction model or a gravity multiplier. */
typedef struct {
    float fraction, damping_nm_s_rad, deadband_rad_s, assist_max_nm, total_max_nm;
} FiveDrag_Axis;
extern const FiveDrag_Axis g_five_drag[FIVE_AXES];
extern const float g_five_drag_filter_s, g_five_drag_ramp_s;
typedef struct {
    bool active;
    uint32_t entered_ms, sampled_ms;
    float velocity_rad_s[FIVE_AXES];
} FiveDrag_State;

/* Sole-owner lifecycle; no enable, CAN, anchor spring or reference trajectory. */
bool FiveDrag_Enter(FiveDrag_State *mode,const FiveMIT_State *state,uint32_t now);
void FiveDrag_Stop(FiveDrag_State *mode);
/* Uses actual measured velocity; no zero-speed kick or guessed intent.
 * Preserve the supplied gravity, trim only the manual overlay to its budget.
 * Output, clipping mask and filter state are unchanged on invalid input. */
bool FiveDrag_Sample(FiveDrag_State *mode,const FiveMIT_State *state,
                     const float gravity_nm[FIVE_AXES],uint32_t now,
                     float torque_nm[FIVE_AXES],unsigned *limited);
#endif
