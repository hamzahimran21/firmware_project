/*
 * Clock configuration for ATSAMS70Q21
 * Configure PLL to achieve 300MHz CPU clock
 */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

/* Clock frequencies */
#define XTAL_FREQ       12000000UL   /* 12 MHz crystal */
#define CPU_FREQ        300000000UL  /* 300 MHz CPU */
#define MCK_FREQ        150000000UL  /* 150 MHz Master Clock */

/*
 * Initialize system clocks
 * XTAL: 12 MHz -> PLLA: 300 MHz -> CPU: 300 MHz, MCK: 150 MHz
 */
void clock_init(void);

/*
 * Delay functions
 */
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#endif /* CLOCK_H */
