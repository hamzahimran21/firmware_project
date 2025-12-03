/*
 * Avian BGT60TR13C Register Configuration
 * Exported from Infineon Radar Fusion GUI
 * Date: 2025-11-30 (updated for presence detection)
 */

#ifndef AVIAN_REGISTERS_H
#define AVIAN_REGISTERS_H

#include <stdint.h>

/* Configuration parameters from Radar Fusion GUI export */
#define AVIAN_START_FREQ_HZ     58000000000ULL  /* 58 GHz */
#define AVIAN_END_FREQ_HZ       63500000000ULL  /* 63.5 GHz */
#define AVIAN_NUM_SAMPLES       64
#define AVIAN_NUM_CHIRPS        32              /* Updated: was 64 */
#define AVIAN_NUM_RX_ANTENNAS   1               /* Updated: was 3 */
#define AVIAN_SAMPLE_RATE_HZ    2000000UL       /* 2 MHz */

/* Frame timing */
#define AVIAN_CHIRP_TIME_US     591             /* ~591 us */
#define AVIAN_FRAME_TIME_MS     77              /* ~77 ms (~13 Hz frame rate) */

/* Register configuration from Infineon Radar Fusion GUI export
 * Format: Upper byte = register address, Lower 24 bits = value
 * File: BGT60TR13C_export_registers_20251130-200209.h
 */
#define AVIAN_NUM_REGS 38

static const uint32_t avian_register_config[AVIAN_NUM_REGS] = {
    0x11e8270UL,
    0x30a0210UL,
    0x9e967fdUL,
    0xb0805b4UL,
    0xd1023ffUL,
    0xf010d00UL,
    0x11000000UL,
    0x13000000UL,
    0x15000000UL,
    0x17000be0UL,
    0x19000000UL,
    0x1b000000UL,
    0x1d000000UL,
    0x1f000b60UL,
    0x21103c51UL,
    0x231ff41fUL,
    0x25701ce7UL,
    0x2d000490UL,
    0x3b000480UL,
    0x49000480UL,
    0x57000480UL,
    0x5911be0eUL,
    0x5b646c0aUL,
    0x5d01f000UL,
    0x5f787e1eUL,
    0x61a2a850UL,
    0x63000c88UL,
    0x65000172UL,
    0x67000040UL,
    0x69000000UL,
    0x6b000000UL,
    0x6d000000UL,
    0x6f393b10UL,
    0x7f000100UL,
    0x8f000100UL,
    0x9f000100UL,
    0xad000000UL,
    0xb7000000UL,
};

#endif /* AVIAN_REGISTERS_H */
