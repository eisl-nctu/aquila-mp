# ======================================================================
#  This Makefile builds a FreeRTOS application *.elf for Aquila.
# ======================================================================
RISCV_ABI = -march=rv32ima_zicsr_zifencei -mabi=ilp32
GCC_LIB = rv32im/ilp32

FREERTOS_DIR = ../FreeRTOS
LIBC = ../elibc
BUILD_DIR = ./build

PROJ = rtos_matrix
OUT_ELF = ./$(PROJ).elf

# Extract configNUMBER_OF_CORES from FreeRTOSConfig.h -------------------------
NUM_CORES := $(shell grep -E '^\s*\#define\s+configNUMBER_OF_CORES\s+[0-9]+' FreeRTOSConfig.h | sed 's/.*configNUMBER_OF_CORES\s\+\([0-9]\+\).*/\1/')

# Select appropriate linker script and crt0 file based on core count
LINKER_SCRIPT = $(PROJ).ld
CRT0_FILE = $(LIBC)/crt0.c

$(info Building for $(NUM_CORES) cores using $(LINKER_SCRIPT) and $(CRT0_FILE))

# FreeRTOS settings -------------------------------------------------------------
FREERTOS_SOURCE_DIR = $(FREERTOS_DIR)/Source

FREERTOS_SRC = \
	$(FREERTOS_SOURCE_DIR)/list.c \
	$(FREERTOS_SOURCE_DIR)/queue.c \
	$(FREERTOS_SOURCE_DIR)/tasks.c \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang/heap_3.c \
	$(FREERTOS_SOURCE_DIR)/timers.c \
	$(FREERTOS_SOURCE_DIR)/event_groups.c \
	$(FREERTOS_SOURCE_DIR)/croutine.c \
	$(FREERTOS_SOURCE_DIR)/stream_buffer.c

FREERTOS_INCLUDES := -I $(FREERTOS_SOURCE_DIR)/include

FREERTOS_BUILD_DIR = $(BUILD_DIR)/FreeRTOS
FREERTOS_OBJS = $(patsubst %.c,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(FREERTOS_SRC)))

VPATH += \
	$(FREERTOS_SOURCE_DIR) \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang

# Platform specific FreeRTOS settings for this application ----------------------
ARCH = RISC-V
ARCH_PORTABLE_INC = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/
ARCH_PORTABLE_SRC = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/port.c
ARCH_PORTABLE_ASM = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/portASM.S

PORT_OBJS := $(patsubst %.c,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(ARCH_PORTABLE_SRC)))
PORT_OBJS += $(patsubst %.S,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(ARCH_PORTABLE_ASM)))
FREERTOS_OBJS += $(PORT_OBJS)

VPATH += $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)

# Application source, include, and object files for compilation ----------------
APP_SRC_DIR = .
APP_SRC = $(APP_SRC_DIR)/$(PROJ).c

# Suspend test application
SUSPEND_TEST_PROJ = rtos_run_suspend_test
SUSPEND_TEST_ELF = ./$(SUSPEND_TEST_PROJ).elf
SUSPEND_TEST_SRC = $(APP_SRC_DIR)/$(SUSPEND_TEST_PROJ).c

LIB_SRC = \
	$(CRT0_FILE) \
	$(LIBC)/stdio.c \
	$(LIBC)/stdlib.c \
	$(LIBC)/string.c \
	$(LIBC)/uart.c \
	$(LIBC)/time.c

APP_INCLUDES = \
	-I ./ \
	-I $(FREERTOS_SOURCE_DIR)/include \
	-I $(ARCH_PORTABLE_INC)


LIB_INCLUDES = \
	-I $(LIBC)

APP_BUILD_DIR = $(BUILD_DIR)/app
APP_OBJS := $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(notdir $(APP_SRC)))

LIB_BUILD_DIR = $(BUILD_DIR)/lib
LIB_OBJS := $(patsubst %.c,$(LIB_BUILD_DIR)/%.o,$(notdir $(LIB_SRC)))

VPATH += $(APP_SRC_DIR) $(LIBC)

# Add application & platform-specific include paths for FreeRTOS ---------------
FREERTOS_INCLUDES += \
	-I ./ \
	-I $(ARCH_PORTABLE_INC) \
	-I $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions

# List of object files to compile for the system -------------------------------
OUT_OBJS = \
	$(APP_OBJS) \
	$(LIB_OBJS) \
	$(FREERTOS_OBJS)

BUILD_DIRECTORIES = \
	$(APP_BUILD_DIR) \
	$(LIB_BUILD_DIR) \
	$(FREERTOS_BUILD_DIR)

# RISC-V compilation tool ------------------------------------------------------
CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

GCC = $(CCPATH)/$(CROSS)-gcc
AS = $(CCPATH)/$(CROSS)-as
LD = $(CCPATH)/$(CROSS)-ld
OBJDUMP = $(CCPATH)/$(CROSS)-objdump
STRIP = $(CCPATH)/$(CROSS)-strip

GCCVERSION = $(shell $(GCC) --version | grep gcc | sed 's/^.* //g')

CFLAGS += -Wall -O2 -fno-builtin $(RISCV_ABI) -mstrict-align
ASMFLAGS = $(RISCV_ABI)
LDFLAGS = -Wl,-Map,"$(BUILD_DIR)/$(PROJ).map" -T$(LINKER_SCRIPT) -nostartfiles -static -L$(RISCV)/lib/gcc/riscv32-unknown-elf/$(GCCVERSION)/$(GCC_LIB) -lgcc -Wl,--no-gc-sections

.PHONY: clean directories app_compile frtos_compile out_elf validate_files help
all: validate_files directories $(OUT_OBJS) $(OUT_ELF)
directories: $(BUILD_DIRECTORIES)
app_compile: directories $(APP_OBJS) $(LIB_OBJS)
frtos_compile: directories $(FREERTOS_OBJS)
out_elf: directories $(OUT_ELF)

# Compile Object Files ---------------------------------------------------------
$(APP_BUILD_DIR)/%.o : %.c FreeRTOSConfig.h
	@echo "[APP Objects] : $@ -----------------------------------------"
	@echo "Building: $<"
	$(GCC) $(CFLAGS) $(LIB_INCLUDES) $(APP_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(LIB_BUILD_DIR)/%.o : %.c
	@echo "[LIB Objects] : $@ -----------------------------------------"
	@echo "Building: $<"
	$(GCC) $(CFLAGS) $(LIB_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(FREERTOS_BUILD_DIR)/%.o : %.c FreeRTOSConfig.h
	@echo "[FreeRTOS Objects] : $@ ------------------------------------"
	@echo "Building: $<"
	$(GCC) $(CFLAGS) $(FREERTOS_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(FREERTOS_BUILD_DIR)/%.o : %.S
	@echo "[FreeRTOS Objects] : $@ ------------------------------------"
	@echo "Building: $<"
	$(GCC) $(ASMFLAGS) $(FREERTOS_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

# Generate ELF -----------------------------------------------------------------
$(OUT_ELF): $(OUT_OBJS)
	@echo '============================================================'
	@echo 'Building target: $@'
	@echo '--------------------------------'
	$(GCC) $(LDFLAGS) -o $@ $(OUT_OBJS)
	$(OBJDUMP) -d $@ > $(BUILD_DIR)/$(PROJ).objdump
	$(STRIP) -g $@
	@echo 'Finished building target: $@'
	@echo ' '

$(BUILD_DIRECTORIES):
	mkdir -p $@

clean:
	rm -rf *.elf
	rm -rf $(BUILD_DIR)

