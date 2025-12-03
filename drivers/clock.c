/*
 * Clock configuration implementation for ATSAMS70Q21
 *
 * Uses INTERNAL 12MHz RC oscillator (no external crystal needed)
 * Target: 300MHz CPU, 150MHz MCK
 */

#include "clock.h"
#include "sams70.h"

/* Timeout for clock operations (prevents hanging forever) */
#define CLOCK_TIMEOUT 1000000

void clock_init(void)
{
    volatile uint32_t timeout;

    /*
     * 1. Enable internal 12MHz RC oscillator
     * This is already enabled by default, but ensure it's on
     */
    CKGR_MOR = PMC_MOR_KEY |
               PMC_MOR_MOSCRCEN |          /* Enable internal RC */
               (0x2 << 4);                 /* MOSCRCF = 12MHz */

    /* Wait for RC oscillator to stabilize (with timeout) */
    timeout = CLOCK_TIMEOUT;
    while (!(PMC_SR & (1 << 17)) && --timeout);  /* MOSCRCS bit */
    if (timeout == 0) return;  /* Failed - exit but don't hang */

    /*
     * 2. Select internal RC as main clock source
     * (Don't select external crystal - it may not exist)
     */
    CKGR_MOR = PMC_MOR_KEY |
               PMC_MOR_MOSCRCEN |
               (0x2 << 4);                 /* Keep using internal RC */

    /* Small delay */
    for (volatile int i = 0; i < 10000; i++);

    /*
     * 3. Configure PLLA for 300MHz
     * PLLA = 12MHz * 25 / 1 = 300MHz
     */
    CKGR_PLLAR = PMC_PLLAR_ONE |
                 PMC_PLLAR_MULA(24) |      /* MUL = 24+1 = 25 */
                 PMC_PLLAR_PLLACOUNT(0x3F) |
                 PMC_PLLAR_DIVA(1);

    /* Wait for PLLA to lock (with timeout) */
    timeout = CLOCK_TIMEOUT;
    while (!(PMC_SR & PMC_SR_LOCKA) && --timeout);
    if (timeout == 0) return;

    /*
     * 4. Switch to PLLA as master clock
     * MCK = PLLA / 2 = 300MHz / 2 = 150MHz
     */

    /* First set prescaler/divider while still on slow clock */
    PMC_MCKR = (PMC_MCKR & ~0x73) |
               PMC_MCKR_PRES_CLK |         /* Prescaler = 1 */
               PMC_MCKR_MDIV_PCK_DIV2;     /* MCK = PCK/2 */

    timeout = CLOCK_TIMEOUT;
    while (!(PMC_SR & PMC_SR_MCKRDY) && --timeout);
    if (timeout == 0) return;

    /* Now switch clock source to PLLA */
    PMC_MCKR = (PMC_MCKR & ~0x3) | PMC_MCKR_CSS_PLLA;

    timeout = CLOCK_TIMEOUT;
    while (!(PMC_SR & PMC_SR_MCKRDY) && --timeout);

    /* Done - CPU at 300MHz, MCK at 150MHz */
}

void delay_ms(uint32_t ms)
{
    /* Rough delay using CPU cycles
     * At 300 MHz: ~300,000 cycles per ms
     * Loop overhead ~3 cycles
     */
    volatile uint32_t count = ms * (CPU_FREQ / 3000);
    while (count--);
}

void delay_us(uint32_t us)
{
    /* Rough delay for microseconds
     * At 300 MHz: ~300 cycles per us
     */
    volatile uint32_t count = us * (CPU_FREQ / 3000000);
    if (count == 0) count = 1;
    while (count--);
}
