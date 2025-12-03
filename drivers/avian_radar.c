/*
 * Avian Radar Driver Implementation
 *
 * Implements initialization and FIFO-based frame acquisition
 * for BGT60TR13C radar sensor.
 */

#include "avian_radar.h"
#include "avian_registers.h"
#include "spi.h"
#include "gpio.h"
#include "clock.h"
#include <string.h>

/* Static frame buffer */
static radar_frame_t current_frame;
static volatile bool acquisition_running = false;
static volatile uint32_t frame_counter = 0;

/* Expected number of 12-bit samples per frame */
#define SAMPLES_PER_FRAME   (RADAR_NUM_SAMPLES * RADAR_NUM_CHIRPS)

/* Each sample is 12 bits, packed as 2 samples in 3 bytes */
#define BYTES_PER_FRAME     ((SAMPLES_PER_FRAME * 3) / 2)

/*
 * Write Avian register via SPI
 * Format: [ADDR<<1 | 1][DATA23:16][DATA15:8][DATA7:0]
 */
static void avian_write_reg(uint8_t addr, uint32_t value)
{
    uint8_t tx_buf[4];

    tx_buf[0] = (addr << 1) | 0x01;  /* Write bit = 1 */
    tx_buf[1] = (value >> 16) & 0xFF;
    tx_buf[2] = (value >> 8) & 0xFF;
    tx_buf[3] = value & 0xFF;

    spi_select();
    spi_transfer_buffer(tx_buf, NULL, 4);
    spi_deselect();
}

/* Store last raw RX bytes for debugging */
static uint8_t last_rx[4];

/*
 * Read Avian register via SPI
 * Format: [ADDR<<1 | 0][0][0][0] -> returns 24-bit value
 */
static uint32_t avian_read_reg(uint8_t addr)
{
    uint8_t tx_buf[4] = {0};
    uint8_t rx_buf[4] = {0xAA, 0xAA, 0xAA, 0xAA};  /* Pre-fill to detect if not overwritten */

    tx_buf[0] = (addr << 1) | 0x00;  /* Read bit = 0 */

    spi_select();
    spi_transfer_buffer(tx_buf, rx_buf, 4);
    spi_deselect();

    /* Save raw bytes for debug */
    last_rx[0] = rx_buf[0];
    last_rx[1] = rx_buf[1];
    last_rx[2] = rx_buf[2];
    last_rx[3] = rx_buf[3];

    /* Return 24-bit value from bytes 1-3 */
    return ((uint32_t)rx_buf[1] << 16) |
           ((uint32_t)rx_buf[2] << 8) |
           rx_buf[3];
}

/*
 * Hardware reset via GPIO
 */
static void avian_hardware_reset(void)
{
    radar_reset_low();
    delay_ms(10);
    radar_reset_high();
    delay_ms(50);  /* Wait for sensor to boot */
}

/*
 * Detect if Avian sensor is present
 */
static bool avian_detect(void)
{
    /* Configure high-speed SPI compensation first */
    avian_write_reg(AVIAN_REG_SFCTL, 0x100000);
    delay_ms(1);

    /* Read ADC0 register to verify device presence */
    uint32_t adc0 = avian_read_reg(AVIAN_REG_ADC0);

    /* Check against known BGT60TR13C/E reset values */
    return (adc0 == AVIAN_ADC0_BGT60TR13C) || (adc0 == AVIAN_ADC0_BGT60TR13E);
}

/*
 * Read FIFO status
 * Returns number of samples available in FIFO
 */
static uint16_t avian_get_fifo_count(void)
{
    uint32_t fstat = avian_read_reg(AVIAN_REG_FSTAT);
    return fstat & AVIAN_FSTAT_FILL_MASK;
}

/*
 * Check for FIFO errors
 */
static bool avian_check_fifo_error(void)
{
    uint32_t fstat = avian_read_reg(AVIAN_REG_FSTAT);
    return (fstat & AVIAN_FSTAT_FOU_ERR) != 0;
}

/*
 * Read samples from FIFO using burst mode
 * Samples are 12-bit packed: 2 samples in 3 bytes
 * Output: unpacked to 16-bit signed values
 */
static bool avian_read_fifo(int16_t *samples, uint16_t num_samples)
{
    /* Calculate bytes to read (2 samples = 3 bytes) */
    uint16_t bytes_to_read = (num_samples * 3) / 2;

    /* Temporary buffer for packed 12-bit data */
    static uint8_t packed_buf[BYTES_PER_FRAME + 4];

    /* Send burst read command
     * Format: 0xFF (burst), ADDR<<1 (read), 0, 0
     */
    uint8_t cmd[4] = {
        0xFF,                           /* Burst command */
        AVIAN_FIFO_READ_ADDR << 1,      /* Read address */
        0,
        0
    };

    spi_select();

    /* Send burst prefix and receive GSR0 in response */
    uint8_t gsr0_response[4];
    spi_transfer_buffer(cmd, gsr0_response, 4);

    /* Check GSR0 for FIFO overflow (bit 3) */
    if (gsr0_response[0] & 0x08) {
        spi_deselect();
        return false;  /* FIFO overflow error */
    }

    /* Burst read the packed data */
    memset(packed_buf, 0, bytes_to_read);
    spi_transfer_buffer(NULL, packed_buf, bytes_to_read);

    spi_deselect();

    /* Unpack 12-bit samples to 16-bit
     * Input:  [B0][B1][B2] = Sample0[11:0], Sample1[11:0]
     *         B0 = S0[11:4]
     *         B1 = S0[3:0] | S1[11:8]
     *         B2 = S1[7:0]
     */
    for (uint16_t i = 0; i < num_samples; i += 2) {
        uint16_t byte_idx = (i * 3) / 2;

        uint8_t b0 = packed_buf[byte_idx];
        uint8_t b1 = packed_buf[byte_idx + 1];
        uint8_t b2 = packed_buf[byte_idx + 2];

        /* Extract sample 0: B0[7:0] << 4 | B1[7:4] */
        uint16_t s0_raw = ((uint16_t)b0 << 4) | (b1 >> 4);

        /* Extract sample 1: B1[3:0] << 8 | B2[7:0] */
        uint16_t s1_raw = ((uint16_t)(b1 & 0x0F) << 8) | b2;

        /* Convert 12-bit unsigned to signed 16-bit
         * 12-bit value: 0-4095, center at 2048
         * Convert to signed: subtract 2048
         */
        samples[i] = (int16_t)(s0_raw) - 2048;
        if (i + 1 < num_samples) {
            samples[i + 1] = (int16_t)(s1_raw) - 2048;
        }
    }

    return true;
}

/*
 * Check if frame acquisition is complete
 * Uses radar IRQ pin (PC6) - goes HIGH when data is ready
 */
static bool avian_frame_complete(void)
{
    /* Check radar IRQ pin - asserted when frame data is ready */
    return radar_irq_read();
}

/* Debug blink function - fast blinks */
static void debug_blink(int count)
{
    for (int i = 0; i < count; i++) {
        *((volatile uint32_t *)(0x400E1434)) = (1 << 5);
        for (volatile int j = 0; j < 300000; j++);
        *((volatile uint32_t *)(0x400E1430)) = (1 << 5);
        for (volatile int j = 0; j < 300000; j++);
    }
    for (volatile int j = 0; j < 800000; j++);
}

/* LONG blink - clearly different from fast blinks */
static void long_blink(int count)
{
    for (int i = 0; i < count; i++) {
        *((volatile uint32_t *)(0x400E1434)) = (1 << 5);
        for (volatile int j = 0; j < 1500000; j++);  /* ON for ~500ms */
        *((volatile uint32_t *)(0x400E1430)) = (1 << 5);
        for (volatile int j = 0; j < 1500000; j++);  /* OFF for ~500ms */
    }
    for (volatile int j = 0; j < 1000000; j++);
}

/*
 * Initialize radar sensor
 */
bool radar_init(void)
{
    /* 1. Hardware reset */
    avian_hardware_reset();

    /* 2. Configure SFCTL for high-speed SPI */
    avian_write_reg(AVIAN_REG_SFCTL, 0x100000);
    delay_ms(1);

    /* 3. Read ADC0 to detect device */
    uint32_t adc0 = avian_read_reg(AVIAN_REG_ADC0);

    /* Check if valid chip detected */
    if (adc0 != AVIAN_ADC0_BGT60TR13C && adc0 != AVIAN_ADC0_BGT60TR13E) {
        return false;  /* Chip not detected */
    }

    /* 3. Software reset */
    avian_write_reg(AVIAN_REG_MAIN, AVIAN_MAIN_SW_RESET);
    delay_ms(10);

    /* 4. Reset FIFO and FSM */
    avian_write_reg(AVIAN_REG_MAIN, AVIAN_MAIN_FIFO_RESET | AVIAN_MAIN_FSM_RESET);
    delay_ms(5);
    avian_write_reg(AVIAN_REG_MAIN, 0);
    delay_ms(5);

    /* 5. Program all registers from exported configuration */
    for (uint32_t i = 0; i < AVIAN_NUM_REGS; i++) {
        uint32_t reg_data = avian_register_config[i];
        uint8_t addr = (reg_data >> 24) & 0xFF;
        uint32_t value = reg_data & 0xFFFFFF;

        avian_write_reg(addr, value);
        delay_ms(1);
    }

    /* 6. Initialize frame buffer */
    memset(&current_frame, 0, sizeof(current_frame));
    current_frame.valid = false;
    acquisition_running = false;
    frame_counter = 0;

    return true;
}

/*
 * Start continuous frame acquisition
 */
void radar_start(void)
{
    /* Reset FIFO before starting */
    radar_reset_fifo();

    /* Start frame acquisition */
    avian_write_reg(AVIAN_REG_MAIN, AVIAN_MAIN_FRAME_START);
    acquisition_running = true;
}

/*
 * Trigger single frame acquisition
 */
void radar_start_frame(void)
{
    if (!acquisition_running) {
        /* Reset FIFO and start new frame */
        radar_reset_fifo();
        avian_write_reg(AVIAN_REG_MAIN, AVIAN_MAIN_FRAME_START);
        acquisition_running = true;
    }
}

/*
 * Stop frame acquisition
 */
void radar_stop(void)
{
    avian_write_reg(AVIAN_REG_MAIN, 0);
    acquisition_running = false;
}

/*
 * Reset the FIFO
 */
void radar_reset_fifo(void)
{
    avian_write_reg(AVIAN_REG_MAIN, AVIAN_MAIN_FIFO_RESET | AVIAN_MAIN_FSM_RESET);
    delay_ms(1);
    avian_write_reg(AVIAN_REG_MAIN, 0);
    current_frame.valid = false;
}

/*
 * Check if frame is ready
 */
bool radar_frame_ready(void)
{
    if (!acquisition_running) {
        return false;
    }

    /* Check if FIFO has complete frame */
    return avian_frame_complete();
}

/*
 * Get the current radar frame
 */
const radar_frame_t* radar_get_frame(void)
{
    if (!acquisition_running) {
        return NULL;
    }

    /* Check for FIFO errors */
    if (avian_check_fifo_error()) {
        /* FIFO overflow - reset and return NULL */
        radar_reset_fifo();
        acquisition_running = false;
        return NULL;
    }

    /* Read samples from FIFO */
    if (!avian_read_fifo(current_frame.samples, SAMPLES_PER_FRAME)) {
        /* Read error */
        radar_reset_fifo();
        acquisition_running = false;
        return NULL;
    }

    /* Mark frame as valid */
    current_frame.valid = true;
    current_frame.timestamp = frame_counter++;

    /* Stop acquisition (will be restarted by radar_start_frame) */
    acquisition_running = false;

    return &current_frame;
}
