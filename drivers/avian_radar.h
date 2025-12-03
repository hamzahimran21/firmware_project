/*
 * Avian Radar Driver for BGT60TR13C
 * Handles sensor initialization and frame acquisition
 */

#ifndef AVIAN_RADAR_H
#define AVIAN_RADAR_H

#include <stdint.h>
#include <stdbool.h>

/* Avian register addresses (from reference firmware) */
#define AVIAN_REG_MAIN          0x00
#define AVIAN_REG_ADC0          0x01
#define AVIAN_REG_SFCTL         0x06    /* SPI/FIFO control - CONFIRMED from reference */
#define AVIAN_REG_FSTAT         0x5A    /* FIFO status register */

/* Main control register bits */
#define AVIAN_MAIN_FRAME_START  (1 << 0)
#define AVIAN_MAIN_SW_RESET     (1 << 1)
#define AVIAN_MAIN_FSM_RESET    (1 << 2)
#define AVIAN_MAIN_FIFO_RESET   (1 << 3)

/* STAT1 register bits */
#define AVIAN_STAT1_FRAME_END   (1 << 0)

/* FSTAT register bits */
#define AVIAN_FSTAT_FILL_MASK   0x1FFF   /* FIFO fill level bits [12:0] */
#define AVIAN_FSTAT_FOU_ERR     (1 << 13) /* FIFO overflow/underflow error */
#define AVIAN_FSTAT_EMPTY       (1 << 14) /* FIFO empty */
#define AVIAN_FSTAT_FULL        (1 << 15) /* FIFO full */

/* Expected ADC0 values for detection */
#define AVIAN_ADC0_BGT60TR13C   0x0A0240
#define AVIAN_ADC0_BGT60TR13E   0x0A0200

/* Radar configuration for presence detection
 * Must match values from Radar Fusion GUI export (avian_registers.h)
 */
#define RADAR_NUM_SAMPLES       64
#define RADAR_NUM_CHIRPS        32   /* Match register config (updated) */
#define RADAR_NUM_RX_ANTENNAS   1    /* Match register config (updated) */
#define RADAR_FRAME_SIZE        (RADAR_NUM_SAMPLES * RADAR_NUM_CHIRPS * RADAR_NUM_RX_ANTENNAS)

/* FIFO burst read address */
#define AVIAN_FIFO_READ_ADDR    0x60

/*
 * Radar frame data structure
 */
typedef struct {
    int16_t samples[RADAR_FRAME_SIZE];  /* Raw ADC samples (12-bit unpacked to 16-bit) */
    uint32_t timestamp;
    bool valid;
} radar_frame_t;

/*
 * Initialize radar sensor
 * Returns true if initialization successful
 */
bool radar_init(void);

/*
 * Start continuous frame acquisition mode
 */
void radar_start(void);

/*
 * Stop frame acquisition
 */
void radar_stop(void);

/*
 * Trigger single frame acquisition
 */
void radar_start_frame(void);

/*
 * Get the current radar frame (non-blocking)
 * Returns pointer to frame data, or NULL if not ready
 */
const radar_frame_t* radar_get_frame(void);

/*
 * Check if frame is ready to be read
 */
bool radar_frame_ready(void);

/*
 * Reset the FIFO
 */
void radar_reset_fifo(void);

#endif /* AVIAN_RADAR_H */
