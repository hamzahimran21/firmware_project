/*
 * Avian Radar Configuration
 * Hardcoded values for presence detection
 */

#ifndef AVIAN_CONFIG_H
#define AVIAN_CONFIG_H

#include <stdint.h>

/* Target radar configuration (from Python presence_detection.py) */
#define RADAR_START_FREQ_HZ         58000000000ULL  /* 58 GHz */
#define RADAR_END_FREQ_HZ           61232137439ULL  /* 61.232 GHz */
#define RADAR_BANDWIDTH_HZ          3232137439ULL   /* 3.232 GHz */
#define RADAR_SAMPLE_RATE_HZ        1000000UL       /* 1 MHz */
#define RADAR_NUM_SAMPLES           64
#define RADAR_NUM_CHIRPS            32
#define RADAR_CHIRP_TIME_US         411238UL        /* 411.238 µs */
#define RADAR_FRAME_TIME_MS         200             /* 200 ms (5 Hz) */
#define RADAR_TX_POWER              31              /* Max power */
#define RADAR_IF_GAIN_DB            33              /* IF gain */
#define RADAR_HP_CUTOFF_HZ          80000UL         /* 80 kHz */
#define RADAR_LP_CUTOFF_HZ          500000UL        /* 500 kHz */

/* Simplified register configuration
 * These values need to be calculated from frequency/timing parameters
 * Reference: Use Infineon's configuration tool or SDK functions
 */
typedef struct {
    /* Power and analog control */
    uint32_t pacr1;
    uint32_t pacr2;

    /* ADC configuration */
    uint32_t adc0;
    uint32_t adc1;

    /* PLL configuration (4 PLLs x 8 registers = 32 values) */
    uint32_t pll1[8];
    uint32_t pll2[8];
    uint32_t pll3[8];
    uint32_t pll4[8];

    /* Chirp sequence configuration */
    uint32_t cs1;
    uint32_t cs1_u[3];
    uint32_t cs1_d[3];

    /* Timing configuration */
    uint32_t ccr0;
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;
} avian_config_t;

/* Get default configuration for presence detection
 * These are placeholder values - need to be filled with actual register values
 */
void avian_get_presence_config(avian_config_t *config);

#endif /* AVIAN_CONFIG_H */
