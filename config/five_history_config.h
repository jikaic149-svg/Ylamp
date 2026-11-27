#ifndef YLAMP_FIVE_HISTORY_CONFIG_H
#define YLAMP_FIVE_HISTORY_CONFIG_H
/* Board02 W25Q64: the five-axis journal owns only 0x7fc000–0x7fdfff.
 * Do not access or erase any other sectors from this module.
 * A nonempty, unrecognised initial partition is reported, not formatted. */
#define FIVE_HISTORY_BASE 0x007fc000U
#define FIVE_HISTORY_BYTES 8192U
/* Encoder phase zero is [0,2*pi): a change of the continuous turn N always
 * triggers a five-axis checkpoint, without waiting for a displacement.
 * Keep this extra per-axis displacement checkpoint <180 degrees so startup's
 * nearest-saved-angle restoration does not become ambiguous during a turn.
 * These are supplemental recording thresholds, NOT motion or cold-start gates. */
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
