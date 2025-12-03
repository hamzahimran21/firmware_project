/*
 * BJT60 Presence Detection Firmware
 */

#include "clock.h"
#include "gpio.h"
#include "spi.h"
#include "avian_radar.h"
#include "presence_detection.h"

static presence_ctx_t presence_ctx;

static void disable_watchdog(void)
{
    volatile uint32_t *WDT_MR = (volatile uint32_t *)0x400E1854;
    *WDT_MR = (1 << 15);
}

static void blink(int count)
{
    for (int i = 0; i < count; i++) {
        *((volatile uint32_t *)(0x400E1434)) = (1 << 5);
        for (volatile int j = 0; j < 800000; j++);
        *((volatile uint32_t *)(0x400E1430)) = (1 << 5);
        for (volatile int j = 0; j < 800000; j++);
    }
    for (volatile int j = 0; j < 1500000; j++);
}

static void led_set(int on)
{
    if (on) {
        *((volatile uint32_t *)(0x400E1434)) = (1 << 5);
    } else {
        *((volatile uint32_t *)(0x400E1430)) = (1 << 5);
    }
}

int main(void)
{
    disable_watchdog();

    /* Enable LED */
    *((volatile uint32_t *)(0x400E0610)) = (1 << 16);
    *((volatile uint32_t *)(0x400E1400)) = (1 << 5);
    *((volatile uint32_t *)(0x400E1410)) = (1 << 5);

    clock_init();
    gpio_init();
    spi_init();

    /* Radar init */
    bool radar_ok = radar_init();

    if (radar_ok) {
        /* SUCCESS: 3 blinks then LED OFF */
        blink(3);
        led_set(0);
    } else {
        /* FAILED: LED stays ON solid */
        led_set(1);
        while(1);  /* Stop here */
    }

    presence_init(&presence_ctx);
    radar_start();

    /* Main loop - LED ON = presence, LED OFF = no presence */
    int no_frame_count = 0;

    while (1) {
        if (radar_frame_ready()) {
            /* Got a frame! Quick blink to show it */
            led_set(1);
            for (volatile int j = 0; j < 100000; j++);

            const radar_frame_t *frame = radar_get_frame();

            if (frame && frame->valid) {
                bool presence = presence_detect(&presence_ctx, frame);
                led_set(presence);
            } else {
                led_set(0);
            }

            radar_start_frame();
            no_frame_count = 0;
        } else {
            /* No frame - blink every ~10 seconds to show we're alive */
            no_frame_count++;
            if (no_frame_count > 10000000) {
                no_frame_count = 0;
                led_set(1);
                for (volatile int j = 0; j < 50000; j++);
                led_set(0);
            }
        }
    }

    return 0;
}
