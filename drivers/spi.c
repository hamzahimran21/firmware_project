/*
 * SPI driver implementation - GPIO Bit-Bang Version
 *
 * Correct pin mappings from RadarBaseboardMCU7 spi_custom.h:
 *   MISO = PA12 (GPIO input)
 *   MOSI = PA13 (GPIO output)
 *   CLK  = PA14 (GPIO output)
 *   CS   = PA11 (GPIO output)
 *
 * SPI Mode 0: CPOL=0, CPHA=0
 *   - Clock idles LOW
 *   - Data sampled on RISING edge
 *   - Data changed on FALLING edge
 */

#include "spi.h"
#include "sams70.h"

/* Correct SPI pin definitions - from reference firmware spi_custom.h */
#define SPI_MISO_PIN    (1 << 12)  /* PA12 - MISO (input) */
#define SPI_MOSI_PIN    (1 << 13)  /* PA13 - MOSI (output) */
#define SPI_CLK_PIN     (1 << 14)  /* PA14 - CLK (output) */
#define SPI_CS_PIN      (1 << 11)  /* PA11 - CS (output, active low) */

/* All SPI pins are on PIOA */
#define SPI_PORT        PIOA

/* Bit-bang delay for clock timing
 * SLOWED DOWN significantly for level shifter compatibility
 * At 300MHz CPU: 200 iterations ≈ 2-5 µs → ~100-200 kHz SPI clock
 */
#define SPI_DELAY()  do { \
    for (volatile int _d = 0; _d < 200; _d++) { __asm__ volatile("nop"); } \
} while(0)

/* Shorter delay for setup/hold times */
#define SPI_DELAY_SHORT()  do { \
    for (volatile int _d = 0; _d < 100; _d++) { __asm__ volatile("nop"); } \
} while(0)

/*
 * Initialize SPI GPIO pins
 */
void spi_init(void)
{
    /* Enable peripheral clock for PIOA */
    PMC_PCER0 = (1 << ID_PIOA);

    /* Configure all SPI pins as GPIO (PIO controlled) */
    SPI_PORT->PIO_PER = SPI_MISO_PIN | SPI_MOSI_PIN | SPI_CLK_PIN | SPI_CS_PIN;

    /* Configure outputs: MOSI, CLK, CS */
    SPI_PORT->PIO_OER = SPI_MOSI_PIN | SPI_CLK_PIN | SPI_CS_PIN;

    /* Configure input: MISO */
    SPI_PORT->PIO_ODR = SPI_MISO_PIN;

    /* Enable pull-up on MISO for stable reads */
    SPI_PORT->PIO_PUER = SPI_MISO_PIN;

    /* Disable pull-up on outputs */
    SPI_PORT->PIO_PUDR = SPI_MOSI_PIN | SPI_CLK_PIN | SPI_CS_PIN;

    /* Set initial states:
     * - CS high (deselected)
     * - CLK low (idle for Mode 0)
     * - MOSI low
     */
    SPI_PORT->PIO_SODR = SPI_CS_PIN;     /* CS = HIGH (deselected) */
    SPI_PORT->PIO_CODR = SPI_CLK_PIN;    /* CLK = LOW (idle) */
    SPI_PORT->PIO_CODR = SPI_MOSI_PIN;   /* MOSI = LOW */

    /* Long delay for pins and level shifters to settle */
    for (volatile int i = 0; i < 500000; i++);  /* ~5-10ms at 300MHz */
}

/*
 * Assert chip select (active low)
 */
void spi_select(void)
{
    SPI_PORT->PIO_CODR = SPI_CS_PIN;  /* CS = LOW (selected) */
    SPI_DELAY_SHORT();  /* Setup time after CS goes low */
}

/*
 * Deassert chip select
 */
void spi_deselect(void)
{
    SPI_DELAY_SHORT();  /* Hold time before CS goes high */
    SPI_PORT->PIO_SODR = SPI_CS_PIN;  /* CS = HIGH (deselected) */
    SPI_DELAY_SHORT();  /* CS high time between transactions */
}

/*
 * Transfer a single byte (full duplex)
 * SPI Mode 0: CPOL=0, CPHA=0
 *   - Data is set on falling edge (or before first rising)
 *   - Data is sampled on rising edge
 */
uint8_t spi_transfer(uint8_t tx_data)
{
    uint8_t rx_data = 0;

    for (int bit = 7; bit >= 0; bit--) {
        /* Set MOSI (data changes while CLK is low) */
        if (tx_data & (1 << bit)) {
            SPI_PORT->PIO_SODR = SPI_MOSI_PIN;  /* MOSI = HIGH */
        } else {
            SPI_PORT->PIO_CODR = SPI_MOSI_PIN;  /* MOSI = LOW */
        }

        SPI_DELAY_SHORT();  /* Data setup time */

        /* Rising edge - data is sampled */
        SPI_PORT->PIO_SODR = SPI_CLK_PIN;  /* CLK = HIGH */

        SPI_DELAY();  /* Clock high time */

        /* Read MISO on rising edge (sample while CLK is high) */
        if (SPI_PORT->PIO_PDSR & SPI_MISO_PIN) {
            rx_data |= (1 << bit);
        }

        /* Falling edge */
        SPI_PORT->PIO_CODR = SPI_CLK_PIN;  /* CLK = LOW */

        SPI_DELAY();  /* Clock low time */
    }

    return rx_data;
}

/*
 * Transfer multiple bytes
 */
void spi_transfer_buffer(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        uint8_t tx_data = tx_buf ? tx_buf[i] : 0x00;
        uint8_t rx_data = spi_transfer(tx_data);
        if (rx_buf) {
            rx_buf[i] = rx_data;
        }
    }
}
