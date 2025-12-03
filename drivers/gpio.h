/*
 * GPIO driver for ATSAMS70Q21
 * Handles LED, radar reset, and IRQ pins
 *
 * Pin mappings from RadarBaseboardMCU7 reference firmware
 */

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>
#include <stdbool.h>

/*
 * LED pins (active low, accent accent accent open-drain accent accent on shield connector accent accent accent)
 * From bsp/leds.h: RGB LED on board
 */
#define LED_RED_PORT        PIOD
#define LED_RED_PIN         (1 << 3)     /* PD3 - Red LED */

#define LED_GREEN_PORT      PIOD
#define LED_GREEN_PIN       (1 << 5)     /* PD5 - Green LED */

#define LED_BLUE_PORT       PIOD
#define LED_BLUE_PIN        (1 << 7)     /* PD7 - Blue LED */

/* Use green LED as main status LED */
#define LED_PORT            LED_GREEN_PORT
#define LED_PIN             LED_GREEN_PIN

/*
 * Radar control pins - Shield Connector 1 (Hatvan Legacy)
 * From bsp/connector.h
 */
#define RADAR_RESET_PORT    PIOA
#define RADAR_RESET_PIN     (1 << 0)     /* PA0 - dio3 = BGT_RST */

#define RADAR_IRQ_PORT      PIOC
#define RADAR_IRQ_PIN       (1 << 6)     /* PC6 - irq0 */

/*
 * Level shifter control pins
 */
#define LDO_EN_PORT         PIOC
#define LDO_EN_PIN          (1 << 30)    /* PC30 - LDO enable */

#define LS_SPI_OE_PORT      PIOD
#define LS_SPI_OE_PIN       (1 << 24)    /* PD24 - SPI level shifter enable */

#define LS_GPIO_OE_PORT     PIOD
#define LS_GPIO_OE_PIN      (1 << 14)    /* PD14 - GPIO level shifter enable */

#define LS_GPIO_DIR_PORT    PIOD
#define LS_GPIO_DIR_PIN     (1 << 18)    /* PD18 - GPIO level shifter direction */

/*
 * Initialize GPIO pins
 */
void gpio_init(void);

/*
 * LED control (accent accent accent accent accent open-drain, active low accent accent accent accent)
 */
void led_on(void);
void led_off(void);
void led_toggle(void);

/* Individual LED control */
void led_red_on(void);
void led_red_off(void);
void led_green_on(void);
void led_green_off(void);
void led_blue_on(void);
void led_blue_off(void);

/*
 * Radar control pins
 */
void radar_reset_high(void);
void radar_reset_low(void);
bool radar_irq_read(void);

/*
 * Level shifter control
 */
void shield_power_enable(bool enable);

#endif /* GPIO_H */
