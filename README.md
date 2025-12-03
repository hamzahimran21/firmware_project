# BJT60 Presence Detection Firmware

Bare-metal firmware for the **Infineon BGT60TR13C** 60GHz radar sensor running on the **ATSAMS70Q21** microcontroller. This firmware performs presence detection directly on the MCU without requiring a PC connection.

## Project Overview

This project implements a standalone presence detection system using FMCW radar technology. The radar sensor transmits 60GHz signals and analyzes reflections to detect human presence within its field of view.

### Hardware

| Component | Description |
|-----------|-------------|
| **MCU** | ATSAMS70Q21 (ARM Cortex-M7 @ 300MHz) |
| **Radar** | Infineon BGT60TR13C (60GHz FMCW) |
| **Board** | RadarBaseboardMCU7 |
| **Interface** | SPI (GPIO bit-bang) |

### Key Features

- FFT-based presence detection algorithm
- No PC required - runs entirely on the MCU
- Uses CMSIS-DSP library for optimized signal processing
- Configurable radar parameters via exported register configuration
- LED indication for presence status

## Project Structure

```
bjt60_firmware/
|-- src/                    # Application source code
|   |-- main.c              # Entry point and main loop
|   |-- presence_detection.c # Detection algorithm
|   |-- presence_detection.h
|   |-- startup.s           # ARM Cortex-M7 startup code
|
|-- drivers/                # Hardware drivers
|   |-- avian_radar.c       # BGT60TR13C radar driver
|   |-- avian_radar.h
|   |-- avian_registers.h   # Radar register configuration
|   |-- avian_config.h      # Radar parameter definitions
|   |-- spi.c               # SPI bit-bang driver
|   |-- spi.h
|   |-- gpio.c              # GPIO driver (LEDs, reset, IRQ)
|   |-- gpio.h
|   |-- clock.c             # Clock configuration (300MHz)
|   |-- clock.h
|   |-- watchdog.c          # Watchdog timer driver
|   |-- watchdog.h
|
|-- include/                # MCU header files
|   |-- sams70.h            # ATSAMS70Q21 register definitions
|
|-- lib/                    # External libraries
|   |-- CMSIS-DSP/          # ARM CMSIS-DSP library (FFT functions)
|   |-- CMSIS_5/            # ARM CMSIS Core headers
|
|-- docs/                   # Reference files
|   |-- BGT60TR13C_export_registers_*.h  # Radar Fusion GUI export
|   |-- BGT60TR13C_settings_*.json       # Radar settings
|
|-- Makefile                # Build configuration
|-- link.ld                 # Linker script
|-- HOW_IT_WORKS.md         # Detailed technical documentation
```

## Building the Firmware

### Prerequisites

- ARM GCC Toolchain (`arm-none-eabi-gcc`)
- Make

### Build Commands

```bash
# Build the firmware
make

# Clean build files
make clean

# Build and show size
make all
```

### Output Files

After building, the following files are generated in `build/`:
- `bjt60_presence.elf` - ELF executable with debug symbols
- `bjt60_presence.bin` - Binary file for flashing
- `bjt60_presence.hex` - Intel HEX format
- `bjt60_presence.map` - Memory map

## Flashing the Firmware

Using BOSSAC (SAM-BA compatible):

```bash
make flash
```

Or manually:

```bash
bossac -p /dev/ttyACM0 -e -w -v -b build/bjt60_presence.bin -R
```

## Radar Configuration

The radar is configured with:
- **Frequency Range**: 58 GHz - 63.5 GHz
- **Samples per Chirp**: 64
- **Chirps per Frame**: 32
- **RX Antennas**: 1
- **Frame Rate**: ~13 Hz

Configuration is stored in `drivers/avian_registers.h` and was exported from the Infineon Radar Fusion GUI.

## LED Behavior

| Pattern | Meaning |
|---------|---------|
| 3 quick blinks at startup | Radar initialization successful |
| Solid LED after startup | Radar initialization failed |
| LED ON | Presence detected |
| LED OFF | No presence detected |
| Periodic blink (~10s) | System heartbeat (idle state) |

## Detection Algorithm

The presence detection uses a dual IIR filter approach:

1. **Data Acquisition**: Read radar frame (2048 samples)
2. **Preprocessing**: Average samples across chirps
3. **Windowing**: Apply Blackman-Harris window
4. **FFT**: 64-point FFT using CMSIS-DSP
5. **Filtering**: Dual exponential moving averages (fast + slow)
6. **Detection**: Threshold comparison of filter difference

## Pin Assignments

| Pin | Function |
|-----|----------|
| PA11 | SPI CS (chip select) |
| PA12 | SPI MISO |
| PA13 | SPI MOSI |
| PA14 | SPI CLK |
| PA0 | Radar Reset |
| PC6 | Radar IRQ |
| PD5 | Green LED (status) |
| PC30 | LDO Enable |
| PD24 | SPI Level Shifter Enable |

## Memory Usage

- **Flash**: 2MB @ 0x00400000
- **RAM**: 384KB @ 0x20400000
- **Stack**: 8KB
- **Heap**: 16KB

Typical firmware size: ~87KB code + data

## Dependencies

- **CMSIS-DSP**: ARM's optimized DSP library for FFT operations
- **CMSIS Core**: ARM Cortex-M core headers

## References

- [Infineon BGT60TR13C Datasheet](https://www.infineon.com/cms/en/product/sensor/radar-sensors/radar-sensors-for-iot/60ghz-radar/)
- [ATSAMS70 Datasheet](https://www.microchip.com/en-us/product/ATSAMS70Q21)
- [CMSIS-DSP Documentation](https://arm-software.github.io/CMSIS-DSP/latest/)
- [Infineon Radar Fusion GUI](https://www.infineon.com/cms/en/tools/landing/infineontoolbox.html)

## License

This project is provided for educational purposes.
