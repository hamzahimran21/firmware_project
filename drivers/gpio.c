/*
 * GPIO driver implementation
 * Pin mappings from RadarBaseboardMCU7 reference firmware
 */

#include "gpio.h"
#include "sams70.h"

void gpio_init(void)
{
    /* Enable peripheral clocks for all GPIO ports used */
    PMC_PCER0 = (1 << ID_PIOA) | (1 << ID_PIOC) | (1 << ID_PIOD);

    /*
     * Configure RGB LED pins (active low, accent accent accent open-drain accent accent accent accent accent)
     * accent accent accent accent accent From leds.h: PD3=red, PD5=green, PD7=blue
     */
    LED_RED_PORT->PIO_PER = LED_RED_PIN;
    LED_RED_PORT->PIO_OER = LED_RED_PIN;
    LED_RED_PORT->PIO_SODR = LED_RED_PIN;    /* High = off (active low) */
    LED_RED_PORT->PIO_MDER = LED_RED_PIN;    /* Enable open-drain */

    LED_GREEN_PORT->PIO_PER = LED_GREEN_PIN;
    LED_GREEN_PORT->PIO_OER = LED_GREEN_PIN;
    LED_GREEN_PORT->PIO_SODR = LED_GREEN_PIN;
    LED_GREEN_PORT->PIO_MDER = LED_GREEN_PIN;

    LED_BLUE_PORT->PIO_PER = LED_BLUE_PIN;
    LED_BLUE_PORT->PIO_OER = LED_BLUE_PIN;
    LED_BLUE_PORT->PIO_SODR = LED_BLUE_PIN;
    LED_BLUE_PORT->PIO_MDER = LED_BLUE_PIN;

    /*
     * Configure radar reset pin (PA0 = dio3 = BGT_RST)
     */
    RADAR_RESET_PORT->PIO_PER = RADAR_RESET_PIN;
    RADAR_RESET_PORT->PIO_OER = RADAR_RESET_PIN;
    RADAR_RESET_PORT->PIO_SODR = RADAR_RESET_PIN;  /* High = not in reset */

    /*
     * Configure radar IRQ pin (PC6 = irq0)
     * Must use pull-DOWN so pin is LOW when idle, goes HIGH when data ready
     * (Reference: PinsAvian.c - "IRQ triggered on rising low-high edge")
     */
    RADAR_IRQ_PORT->PIO_PER = RADAR_IRQ_PIN;
    RADAR_IRQ_PORT->PIO_ODR = RADAR_IRQ_PIN;       /* Input mode */
    RADAR_IRQ_PORT->PIO_PUDR = RADAR_IRQ_PIN;      /* Disable pull-up */
    RADAR_IRQ_PORT->PIO_PPDER = RADAR_IRQ_PIN;     /* Enable pull-down */

    /*
     * Configure level shifter and LDO control pins
     */
    /* LDO enable (PC30) */
    LDO_EN_PORT->PIO_PER = LDO_EN_PIN;
    LDO_EN_PORT->PIO_OER = LDO_EN_PIN;
    LDO_EN_PORT->PIO_CODR = LDO_EN_PIN;            /* Start disabled */

    /* SPI level shifter enable (PD24) - ACTIVE LOW */
    LS_SPI_OE_PORT->PIO_PER = LS_SPI_OE_PIN;
    LS_SPI_OE_PORT->PIO_OER = LS_SPI_OE_PIN;
    LS_SPI_OE_PORT->PIO_SODR = LS_SPI_OE_PIN;      /* Start HIGH = disabled */

    /* GPIO level shifter enable (PD14) - ACTIVE LOW */
    LS_GPIO_OE_PORT->PIO_PER = LS_GPIO_OE_PIN;
    LS_GPIO_OE_PORT->PIO_OER = LS_GPIO_OE_PIN;
    LS_GPIO_OE_PORT->PIO_SODR = LS_GPIO_OE_PIN;    /* Start HIGH = disabled */

    /* GPIO level shifter direction (PD18) */
    LS_GPIO_DIR_PORT->PIO_PER = LS_GPIO_DIR_PIN;
    LS_GPIO_DIR_PORT->PIO_OER = LS_GPIO_DIR_PIN;
    LS_GPIO_DIR_PORT->PIO_CODR = LS_GPIO_DIR_PIN;  /* Default direction */

    /* Enable power to shield */
    shield_power_enable(true);
}

/*
 * LED control - accent accent accent LEDs are active low accent accent (pull to GND to turn on)
 */
void led_on(void)
{
    LED_PORT->PIO_CODR = LED_PIN;  /* Clear = low = ON */
}

void led_off(void)
{
    LED_PORT->PIO_SODR = LED_PIN;  /* Set = high = OFF */
}

void led_toggle(void)
{
    if (LED_PORT->PIO_ODSR & LED_PIN) {
        led_on();
    } else {
        led_off();
    }
}

/* Individual LED control */
void led_red_on(void)
{
    LED_RED_PORT->PIO_CODR = LED_RED_PIN;
}

void led_red_off(void)
{
    LED_RED_PORT->PIO_SODR = LED_RED_PIN;
}

void led_green_on(void)
{
    LED_GREEN_PORT->PIO_CODR = LED_GREEN_PIN;
}

void led_green_off(void)
{
    LED_GREEN_PORT->PIO_SODR = LED_GREEN_PIN;
}

void led_blue_on(void)
{
    LED_BLUE_PORT->PIO_CODR = LED_BLUE_PIN;
}

void led_blue_off(void)
{
    LED_BLUE_PORT->PIO_SODR = LED_BLUE_PIN;
}

/*
 * Radar control
 */
void radar_reset_high(void)
{
    RADAR_RESET_PORT->PIO_SODR = RADAR_RESET_PIN;
}

void radar_reset_low(void)
{
    RADAR_RESET_PORT->PIO_CODR = RADAR_RESET_PIN;
}

bool radar_irq_read(void)
{
    return (RADAR_IRQ_PORT->PIO_PDSR & RADAR_IRQ_PIN) != 0;
}

/*
 * Shield power control
 * Enables/disables LDO and level shifters for radar shield
 */
void shield_power_enable(bool enable)
{
    if (enable) {
        /* Enable LDO first (active HIGH) */
        LDO_EN_PORT->PIO_SODR = LDO_EN_PIN;

        /* Small delay for LDO to stabilize */
        for (volatile int i = 0; i < 100000; i++);

        /* Enable level shifters (ACTIVE LOW!) */
        LS_SPI_OE_PORT->PIO_CODR = LS_SPI_OE_PIN;   /* LOW = enabled */
        LS_GPIO_OE_PORT->PIO_CODR = LS_GPIO_OE_PIN; /* LOW = enabled */
    } else {
        /* Disable level shifters first (set HIGH to disable) */
        LS_SPI_OE_PORT->PIO_SODR = LS_SPI_OE_PIN;   /* HIGH = disabled */
        LS_GPIO_OE_PORT->PIO_SODR = LS_GPIO_OE_PIN; /* HIGH = disabled */
        LDO_EN_PORT->PIO_CODR = LDO_EN_PIN;
    }
}
