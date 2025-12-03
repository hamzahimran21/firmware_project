/*
 * Watchdog Timer Driver
 * Auto-resets MCU if code hangs - prevents bricking
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

/*
 * Initialize and enable watchdog timer
 * timeout_ms: Time before reset (max ~16 seconds)
 *
 * IMPORTANT: Once enabled, watchdog_reset() must be called
 * periodically or the MCU will reset!
 */
void watchdog_init(uint32_t timeout_ms);

/*
 * Reset (pet) the watchdog timer
 * Call this regularly in main loop to prevent reset
 */
void watchdog_reset(void);

/*
 * Disable watchdog (can only be done once after reset)
 * Use this if you need to disable for debugging
 */
void watchdog_disable(void);

#endif /* WATCHDOG_H */
