/*
 * Watchdog Timer Driver Implementation
 *
 * The SAMS70 watchdog runs on the slow clock (32.768 kHz).
 * WDV value = timeout_ms * 32.768 / 1000 / 128 = timeout_ms / 3906
 * Max WDV = 4095 -> Max timeout ~16 seconds
 */

#include "watchdog.h"
#include "sams70.h"

void watchdog_init(uint32_t timeout_ms)
{
    /*
     * Calculate WDV value
     * WDT clock = SLCK/128 = 32768/128 = 256 Hz
     * WDV = timeout_ms * 256 / 1000 = timeout_ms / 3.9
     * Use timeout_ms / 4 as approximation (slightly shorter timeout)
     */
    uint32_t wdv = timeout_ms / 4;
    if (wdv > 4095) wdv = 4095;  /* Max 12-bit value */
    if (wdv < 1) wdv = 1;

    /*
     * Configure watchdog:
     * - WDV: Counter value (timeout)
     * - WDD: Delta value (set same as WDV for simple operation)
     * - WDRSTEN: Enable reset on timeout
     * - WDDBGHLT: Halt watchdog when debugger connected
     * - WDIDLEHLT: Halt watchdog in idle mode
     */
    WDT->WDT_MR = WDT_MR_WDV(wdv) |
                  WDT_MR_WDD(wdv) |
                  WDT_MR_WDRSTEN |
                  WDT_MR_WDDBGHLT |
                  WDT_MR_WDIDLEHLT;
}

void watchdog_reset(void)
{
    /* Restart the watchdog counter */
    WDT->WDT_CR = WDT_CR_WDRSTT | WDT_CR_KEY;
}

void watchdog_disable(void)
{
    /*
     * Disable watchdog
     * NOTE: Can only be done once after MCU reset!
     * Once disabled, cannot be re-enabled until next reset.
     */
    WDT->WDT_MR = WDT_MR_WDDIS;
}
