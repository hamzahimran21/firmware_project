# Makefile for BJT60 Presence Detection Firmware
# Target: ATSAMS70Q21 (ARM Cortex-M7)

# Project name
PROJECT = bjt60_presence

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)as
LD = $(PREFIX)ld
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE = $(PREFIX)size

# MCU configuration
MCU = cortex-m7
ARCH = armv7e-m
FPU = fpv5-d16

# Directories
SRC_DIR = src
DRV_DIR = drivers
INC_DIR = include
LIB_DIR = lib
BUILD_DIR = build

# CMSIS-DSP paths
CMSIS_DSP = $(LIB_DIR)/CMSIS-DSP
CMSIS_INC = $(CMSIS_DSP)/Include
CMSIS_PRIV = $(CMSIS_DSP)/PrivateInclude
CMSIS_SRC = $(CMSIS_DSP)/Source

# CMSIS Core paths
CMSIS_CORE = $(LIB_DIR)/CMSIS_5/CMSIS/Core/Include

# CMSIS-DSP sources (only what we need for FFT)
CMSIS_C = $(CMSIS_SRC)/TransformFunctions/arm_rfft_fast_f32.c \
          $(CMSIS_SRC)/TransformFunctions/arm_rfft_fast_init_f32.c \
          $(CMSIS_SRC)/TransformFunctions/arm_cfft_f32.c \
          $(CMSIS_SRC)/TransformFunctions/arm_cfft_init_f32.c \
          $(CMSIS_SRC)/TransformFunctions/arm_cfft_radix8_f32.c \
          $(CMSIS_SRC)/TransformFunctions/arm_bitreversal2.c \
          $(CMSIS_SRC)/CommonTables/arm_common_tables.c \
          $(CMSIS_SRC)/CommonTables/arm_const_structs.c \
          $(CMSIS_SRC)/ComplexMathFunctions/arm_cmplx_mag_f32.c

# Source files
SRC_C = $(wildcard $(SRC_DIR)/*.c) \
        $(wildcard $(DRV_DIR)/*.c) \
        $(CMSIS_C)

SRC_S = $(SRC_DIR)/startup.s

# Object files
OBJ_C = $(patsubst %.c,$(BUILD_DIR)/%.o,$(notdir $(SRC_C)))
OBJ_S = $(BUILD_DIR)/startup.o
OBJ = $(OBJ_C) $(OBJ_S)

# Include paths
INC = -I$(INC_DIR) \
      -I$(SRC_DIR) \
      -I$(DRV_DIR) \
      -I$(CMSIS_INC) \
      -I$(CMSIS_PRIV) \
      -I$(CMSIS_CORE)

# Compiler flags
CFLAGS = -mcpu=$(MCU) \
         -march=$(ARCH) \
         -mthumb \
         -mfloat-abi=hard \
         -mfpu=$(FPU) \
         -O2 \
         -g \
         -Wall \
         -Wextra \
         -ffunction-sections \
         -fdata-sections \
         -std=gnu11 \
         -DARM_MATH_CM7 \
         -D__FPU_PRESENT=1 \
         $(INC)

# Assembler flags
ASFLAGS = -mcpu=$(MCU) \
          -march=$(ARCH) \
          -mthumb \
          -mfloat-abi=hard \
          -mfpu=$(FPU) \
          -g

# Linker flags
LDFLAGS = -T link.ld \
          -mcpu=$(MCU) \
          -march=$(ARCH) \
          -mthumb \
          -mfloat-abi=hard \
          -mfpu=$(FPU) \
          -specs=nosys.specs \
          -specs=nano.specs \
          -Wl,--gc-sections \
          -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map

# Targets
.PHONY: all clean flash

all: $(BUILD_DIR)/$(PROJECT).bin $(BUILD_DIR)/$(PROJECT).hex

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile C sources
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(DRV_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Compile CMSIS-DSP sources
$(BUILD_DIR)/%.o: $(CMSIS_SRC)/TransformFunctions/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(CMSIS_SRC)/CommonTables/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(CMSIS_SRC)/ComplexMathFunctions/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Assemble startup code
$(BUILD_DIR)/startup.o: $(SRC_S) | $(BUILD_DIR)
	$(CC) $(ASFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(PROJECT).elf: $(OBJ)
	$(CC) $(LDFLAGS) $(OBJ) -o $@ -lm
	$(SIZE) $@

# Create bin file
$(BUILD_DIR)/$(PROJECT).bin: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

# Create hex file
$(BUILD_DIR)/$(PROJECT).hex: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJCOPY) -O ihex $< $@

# Clean build files
clean:
	rm -rf $(BUILD_DIR)

# Flash firmware (using bossac)
flash: $(BUILD_DIR)/$(PROJECT).bin
	bossac -p /dev/ttyACM0 -e -w -v -b $< -R

# Disassemble
disasm: $(BUILD_DIR)/$(PROJECT).elf
	$(OBJDUMP) -d $< > $(BUILD_DIR)/$(PROJECT).asm
