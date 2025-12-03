# How It Works - Technical Deep Dive

This document explains how each file in the BJT60 firmware works together to create a standalone presence detection system.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Boot Sequence](#boot-sequence)
3. [Source Files](#source-files)
4. [Driver Files](#driver-files)
5. [Include Files](#include-files)
6. [Build System](#build-system)
7. [Data Flow](#data-flow)
8. [Algorithm Details](#algorithm-details)

---

## System Architecture

```
+------------------+     SPI      +------------------+
|   ATSAMS70Q21    |<------------>|   BGT60TR13C     |
|   (Cortex-M7)    |              |   (60GHz Radar)  |
|                  |     IRQ      |                  |
|   300MHz CPU     |<-------------|   FMCW Sensor    |
|   384KB RAM      |              |                  |
+--------+---------+              +------------------+
         |
         | GPIO
         v
      [LED]
   (Presence Status)
```

The system consists of:
- **MCU**: Runs the main loop, reads radar data, processes FFT, makes detection decisions
- **Radar**: Generates FMCW chirps, captures reflections, stores samples in FIFO
- **LED**: Indicates presence detection status to the user

---

## Boot Sequence

1. **Reset** - CPU starts at Reset_Handler in startup.s
2. **Memory Init** - Copy .data from Flash to RAM, zero .bss
3. **FPU Enable** - Enable floating-point unit for DSP calculations
4. **main()** - Application entry point
5. **Watchdog Disable** - Prevent automatic resets during development
6. **LED Init** - Configure status LED GPIO
7. **Clock Init** - Set CPU to 300MHz, MCK to 150MHz
8. **GPIO Init** - Configure all pins (LEDs, level shifters, radar control)
9. **SPI Init** - Configure SPI pins for radar communication
10. **Radar Init** - Reset radar, write configuration registers
11. **Main Loop** - Continuously poll for frames and detect presence

---

## Source Files

### src/main.c

**Purpose**: Application entry point and main control loop.

**What it does**:
- Disables the watchdog timer to prevent unwanted resets
- Initializes the LED for visual feedback
- Calls all hardware initialization functions
- Attempts to initialize the radar sensor
- Shows success (3 blinks) or failure (solid LED) based on radar init
- Runs the main loop that:
  - Checks if a radar frame is ready
  - Reads the frame data
  - Runs the presence detection algorithm
  - Updates the LED based on detection result
  - Provides periodic "heartbeat" blink every 10 seconds during idle state

**Key functions**:
- `disable_watchdog()` - Writes to watchdog control register to disable it
- `blink(count)` - Flashes LED a specified number of times
- `led_set(on)` - Turns LED on or off
- `main()` - The main application function

---

### src/presence_detection.c

**Purpose**: Implements the FFT-based presence detection algorithm.

**What it does**:
- Initializes the CMSIS-DSP FFT instance for 64-point transforms
- Processes each radar frame through a signal processing pipeline
- Uses dual IIR filters (slow and fast) to track background and targets
- Makes a binary presence/no-presence decision based on threshold

**Algorithm pipeline**:
1. **Chirp Averaging**: Averages all chirps together to reduce noise
2. **Windowing**: Applies Blackman-Harris window to reduce spectral leakage
3. **FFT**: Computes 64-point FFT to convert time-domain to frequency-domain
4. **Magnitude**: Calculates magnitude of complex FFT output
5. **IIR Filtering**: Updates slow average (background) and fast average (target)
6. **Detection**: Compares difference to threshold (0.0007)

**Key parameters**:
- `THRESHOLD_PRESENCE` = 0.0007 - Detection sensitivity
- `ALPHA_SLOW` = 0.001 - Slow filter coefficient (background tracking)
- `ALPHA_FAST` = 0.6 - Fast filter coefficient (target tracking)
- Detection range: FFT bins 8-32 (avoids DC and high-frequency noise)

---

### src/presence_detection.h

**Purpose**: Header file defining the presence detection interface.

**What it defines**:
- Algorithm configuration constants (thresholds, filter coefficients)
- `presence_ctx_t` structure to hold algorithm state between calls
- Function prototypes for `presence_init()` and `presence_detect()`

---

### src/startup.s

**Purpose**: ARM assembly startup code for the Cortex-M7.

**What it does**:
- Defines the vector table (addresses of exception handlers)
- Implements Reset_Handler that runs on power-on/reset
- Copies initialized data (.data) from Flash to RAM
- Zeros uninitialized data (.bss)
- Enables the Floating-Point Unit (FPU)
- Calls main()
- Provides default (empty) handlers for all interrupts

**Vector table includes**:
- Stack pointer (top of RAM)
- Reset handler (entry point)
- Fault handlers (NMI, HardFault, etc.)
- System handlers (SysTick, PendSV, SVC)
- Peripheral interrupt handlers (UART, SPI, Timer, etc.)

---

## Driver Files

### drivers/avian_radar.c

**Purpose**: Driver for the BGT60TR13C radar sensor.

**What it does**:
- Provides low-level SPI communication with the radar
- Implements hardware reset sequence
- Detects radar presence by reading ADC0 register
- Writes all 38 configuration registers on init
- Manages FIFO-based frame acquisition
- Unpacks 12-bit packed samples to 16-bit values

**Key functions**:
- `avian_write_reg(addr, value)` - Write a 24-bit value to a register
- `avian_read_reg(addr)` - Read a 24-bit value from a register
- `avian_hardware_reset()` - Perform hardware reset via GPIO
- `radar_init()` - Full initialization sequence
- `radar_start()` - Begin frame acquisition
- `radar_frame_ready()` - Check if frame data is available (polls IRQ pin)
- `radar_get_frame()` - Read frame data from FIFO

**SPI Protocol**:
- Write: `[ADDR<<1 | 1][DATA23:16][DATA15:8][DATA7:0]`
- Read: `[ADDR<<1 | 0][0][0][0]` -> returns 24-bit value
- Burst Read: `[0xFF][ADDR<<1][0][0]` + read N bytes

**Sample unpacking**:
The radar stores 12-bit samples packed as 2 samples in 3 bytes:
- Byte 0: Sample0[11:4]
- Byte 1: Sample0[3:0] | Sample1[11:8]
- Byte 2: Sample1[7:0]

---

### drivers/avian_radar.h

**Purpose**: Header file for radar driver interface.

**What it defines**:
- Register addresses (MAIN, ADC0, SFCTL, FSTAT)
- Control register bit masks
- Radar configuration constants (samples, chirps, antennas)
- `radar_frame_t` structure containing sample buffer and metadata
- Function prototypes for all radar operations

---

### drivers/avian_registers.h

**Purpose**: Contains the radar register configuration exported from Radar Fusion GUI.

**What it defines**:
- Radar parameters (frequency range, sample rate, timing)
- Array of 38 register values in format `[ADDR<<24 | VALUE]`

**How to update**:
1. Open Infineon Radar Fusion GUI
2. Configure desired radar parameters
3. Export register configuration
4. Replace the `avian_register_config[]` array

---

### drivers/avian_config.h

**Purpose**: Defines target radar configuration parameters.

**What it defines**:
- Frequency parameters (start, end, bandwidth)
- Timing parameters (sample rate, chirp time, frame time)
- Gain and filter settings
- Structure for organized register configuration

---

### drivers/spi.c

**Purpose**: GPIO bit-bang SPI driver for radar communication.

**What it does**:
- Configures GPIO pins PA11-PA14 for SPI function
- Implements SPI Mode 0 (CPOL=0, CPHA=0) protocol
- Provides bit-bang transfer at ~100-200 kHz
- Handles chip select assertion/deassertion

**Pin assignments**:
- PA11 = CS (Chip Select, active low)
- PA12 = MISO (Master In, Slave Out)
- PA13 = MOSI (Master Out, Slave In)
- PA14 = CLK (Serial Clock)

**Why bit-bang instead of hardware SPI**:
The radar is connected through level shifters that require slower signal transitions. Hardware SPI runs too fast, causing signal integrity issues.

**Key functions**:
- `spi_init()` - Configure GPIO pins
- `spi_select()` / `spi_deselect()` - Control chip select
- `spi_transfer(tx)` - Send one byte, receive one byte
- `spi_transfer_buffer(tx, rx, len)` - Transfer multiple bytes

---

### drivers/spi.h

**Purpose**: Header file for SPI driver.

**What it defines**:
- Function prototypes for SPI operations

---

### drivers/gpio.c

**Purpose**: GPIO driver for LEDs, radar control, and level shifters.

**What it does**:
- Enables peripheral clocks for GPIO ports
- Configures RGB LED pins as open-drain outputs
- Configures radar reset pin as output
- Configures radar IRQ pin as input with pull-down
- Configures level shifter enable pins
- Provides functions to control LEDs and read IRQ

**Pin assignments**:
| Pin | Function | Configuration |
|-----|----------|---------------|
| PD3 | Red LED | Open-drain, active low |
| PD5 | Green LED | Open-drain, active low |
| PD7 | Blue LED | Open-drain, active low |
| PA0 | Radar Reset | Push-pull output |
| PC6 | Radar IRQ | Input with pull-down |
| PC30 | LDO Enable | Push-pull output |
| PD24 | SPI Level Shifter | Active low enable |
| PD14 | GPIO Level Shifter | Active low enable |

**Key functions**:
- `gpio_init()` - Configure all GPIO pins
- `led_on()` / `led_off()` / `led_toggle()` - Control status LED
- `radar_reset_high()` / `radar_reset_low()` - Control radar reset
- `radar_irq_read()` - Read radar interrupt pin state
- `shield_power_enable(bool)` - Enable/disable radar power and level shifters

---

### drivers/gpio.h

**Purpose**: Header file for GPIO driver.

**What it defines**:
- Pin definitions for all GPIOs
- Function prototypes for GPIO operations

---

### drivers/clock.c

**Purpose**: System clock configuration.

**What it does**:
- Enables the internal 12MHz RC oscillator
- Configures PLLA to multiply 12MHz to 300MHz
- Switches CPU clock source to PLLA
- Configures MCK divider for 150MHz master clock
- Provides delay functions

**Clock tree**:
```
12MHz RC -> PLLA (x25) -> 300MHz CPU
                      -> /2 -> 150MHz MCK (peripherals)
```

**Key functions**:
- `clock_init()` - Configure all clocks
- `delay_ms(ms)` - Blocking delay in milliseconds
- `delay_us(us)` - Blocking delay in microseconds

---

### drivers/clock.h

**Purpose**: Header file for clock driver.

**What it defines**:
- Clock frequency constants (XTAL, CPU, MCK)
- Function prototypes

---

### drivers/watchdog.c

**Purpose**: Watchdog timer driver.

**What it does**:
- Provides functions to configure and control the watchdog timer
- Watchdog can auto-reset the MCU if code hangs (not used in this firmware)

**Key functions**:
- `watchdog_init(timeout_ms)` - Enable watchdog with specified timeout
- `watchdog_reset()` - "Pet" the watchdog to prevent reset
- `watchdog_disable()` - Disable watchdog (can only be done once after MCU reset)

---

### drivers/watchdog.h

**Purpose**: Header file for watchdog driver.

**What it defines**:
- Function prototypes for watchdog operations

---

## Include Files

### include/sams70.h

**Purpose**: ATSAMS70Q21 microcontroller register definitions.

**What it defines**:
- Peripheral base addresses
- PMC (Power Management Controller) registers
- PIO (Parallel I/O) register structure and instances
- SPI register structure
- UART register structure
- WDT (Watchdog Timer) register structure
- Bit field definitions for all registers

This file allows the firmware to access hardware registers using named constants instead of raw memory addresses.

---

## Build System

### Makefile

**Purpose**: Build automation using GNU Make.

**What it does**:
- Defines toolchain (arm-none-eabi-gcc)
- Specifies compiler flags for Cortex-M7 with FPU
- Lists all source files (application + CMSIS-DSP)
- Defines rules to compile .c and .s files
- Links everything into an ELF file
- Creates .bin and .hex output files

**Key targets**:
- `make` or `make all` - Build everything
- `make clean` - Remove build files
- `make flash` - Flash to target using bossac

**Compiler flags**:
- `-mcpu=cortex-m7` - Target CPU
- `-mfloat-abi=hard` - Hardware floating point
- `-mfpu=fpv5-d16` - FPU type
- `-DARM_MATH_CM7` - CMSIS-DSP Cortex-M7 optimizations

---

### link.ld

**Purpose**: Linker script defining memory layout.

**What it defines**:
- Memory regions (Flash and RAM addresses/sizes)
- Section placement (.text, .data, .bss)
- Stack and heap sizes
- Symbols for startup code (_estack, _sdata, etc.)

**Memory map**:
- Flash: 2MB starting at 0x00400000
- RAM: 384KB starting at 0x20400000
- Stack: 8KB at end of RAM
- Heap: 16KB before stack

---

## Data Flow

```
                        RADAR SENSOR
                             |
                    [FIFO - 2048 samples]
                             |
                      SPI Burst Read
                             |
                    [12-bit unpacking]
                             |
                      radar_frame_t
                             |
           +----------------+----------------+
           |                                 |
    [Average across chirps]                  |
           |                                 |
    [Blackman-Harris window]                 |
           |                                 |
    [64-point FFT]                           |
           |                                 |
    [Complex magnitude]                      |
           |                                 |
    [IIR filters (slow/fast)]                |
           |                                 |
    [Threshold detection]                    |
           |                                 |
           v                                 |
    presence_detected ---------------------->|
           |                                 |
           v                                 v
         [LED]                        [Main Loop]
```

---

## Algorithm Details

### FFT-Based Range Detection

The radar transmits frequency-modulated continuous wave (FMCW) chirps. When a chirp reflects off an object, the received signal has a frequency offset proportional to the object's distance.

The FFT converts this time-domain signal to frequency-domain, where:
- **FFT bin index** corresponds to **range** (distance)
- **FFT magnitude** corresponds to **signal strength** (reflection intensity)

### Dual IIR Filter Approach

Two exponential moving averages track the signal:

1. **Slow Average** (alpha = 0.001)
   - Tracks the static background
   - Updates slowly, ignoring brief changes
   - Represents "what the scene normally looks like"

2. **Fast Average** (alpha = 0.6)
   - Tracks rapid changes
   - Responds quickly to new targets
   - Represents "what's happening right now"

**Detection logic**:
```
if (fast_average - slow_average) > threshold:
    presence = TRUE
else:
    presence = FALSE
```

When a person enters the scene, the fast average jumps up while the slow average stays low, creating a large difference that exceeds the threshold.

### Threshold Selection

The threshold (0.0007) determines sensitivity:
- **Lower threshold**: More sensitive, may false trigger on noise
- **Higher threshold**: Less sensitive, may miss weak targets

This value was tuned for typical indoor presence detection scenarios.

---

## Files to Upload to GitHub

Essential files for the project:

```
bjt60_firmware/
|-- src/
|   |-- main.c
|   |-- presence_detection.c
|   |-- presence_detection.h
|   |-- startup.s
|
|-- drivers/
|   |-- avian_radar.c
|   |-- avian_radar.h
|   |-- avian_registers.h
|   |-- avian_config.h
|   |-- spi.c
|   |-- spi.h
|   |-- gpio.c
|   |-- gpio.h
|   |-- clock.c
|   |-- clock.h
|   |-- watchdog.c
|   |-- watchdog.h
|
|-- include/
|   |-- sams70.h
|
|-- lib/
|   |-- CMSIS-DSP/          (external dependency)
|   |-- CMSIS_5/            (external dependency)
|
|-- docs/
|   |-- BGT60TR13C_export_registers_*.h
|   |-- BGT60TR13C_settings_*.json
|
|-- Makefile
|-- link.ld
|-- README.md
|-- HOW_IT_WORKS.md
|-- .gitignore
```

Note: The `lib/` folder contains external dependencies (CMSIS-DSP, CMSIS_5). You may want to:
1. Include them as Git submodules, or
2. Add instructions to download them separately, or
3. Include only the necessary files used by the project

