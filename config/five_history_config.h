#ifndef YLAMP_FIVE_HISTORY_CONFIG_H
#define YLAMP_FIVE_HISTORY_CONFIG_H
/* Board02 W25Q64 journal: 8KiB immediately BEFORE the original IMU-bias
 * sectors 0x7fe000/0x7ff000. Never erase those or any other user's data.
 * A nonempty, unrecognised initial partition is reported, not formatted. */
#define FIVE_HISTORY_BASE 0x007fc000U
#define FIVE_HISTORY_BYTES 8192U
/* Per-axis displacement from the last VERIFIED committed checkpoint.
 * Any axis triggers one five-axis record; small steps accumulate.
 * These are recording thresholds, NOT motion or cold-start gates. */
#define FIVE_HISTORY_STEP_DEG {150.f,150.f,150.f,150.f,150.f}
/* User-marked CURRENT J4 circle: phase in [0,360), N=-1.
 * Seed only on first acquisition without a compatible saved checkpoint,
 * and only if that equivalent lies in the configured travel window.
 * Never pin N during subsequent movement or override a newer checkpoint. */
#define FIVE_HISTORY_INITIAL_TURN_MASK (1U << 3)
#define FIVE_HISTORY_INITIAL_TURNS {0,0,0,-1,0}
#define FIVE_HISTORY_PAGE_TIMEOUT_MS 100U
#define FIVE_HISTORY_ERASE_TIMEOUT_MS 5000U
#endif
