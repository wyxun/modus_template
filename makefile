# ==============================================================================
# Project Makefile — Multi-chip support via target/$(TARGET_CHIP)/
# Toolchain : LLVM for bare-metal targets
# Usage     : make [BUILD=debug|release] [TARGET_CHIP=stm32g431] ...
# ==============================================================================
.DEFAULT_GOAL := all
# Resolve the target= alias before a personal file selects its probe.
ifdef target
    ifneq ($(origin TARGET_CHIP),command line)
        override TARGET_CHIP := $(target)
    endif
endif
-include config.local.mk

ifeq ($(OS),Windows_NT)
    EXE_SUFFIX = .exe
    ifneq ($(strip $(SW_ROOT)),)
        MSYS2_BIN ?= $(SW_ROOT)/msys64/mingw64/bin
        ARM_LLVM_BIN ?= $(SW_ROOT)/llvm_for_arm/bin
        WCH_OPENOCD_ROOT ?= $(SW_ROOT)/MounRiver/MounRiver_Studio2/resources/app/resources/win32/components/WCH/OpenOCD/OpenOCD
    endif
    RISCV_LLVM_BIN ?= $(MSYS2_BIN)
    ifneq ($(strip $(MSYS2_BIN)),)
        ifndef STM32_OPENOCD_BIN
            STM32_OPENOCD_BIN = $(MSYS2_BIN)/openocd.exe
            STM32_OPENOCD_SCRIPTS ?= $(MSYS2_BIN)/../share/openocd/scripts
        endif
    endif
    SWD_SPEED_KHZ ?= 4000
endif

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
TARGET = template

TARGET_CHIP ?= stm32g431
CLASS_SOURCES = class/template_class.c template_driver.c

include target/$(TARGET_CHIP)/target.mk

# ------------------------------------------------------------------------------
# Toolchain directories are optional when the tools are on PATH.
# ------------------------------------------------------------------------------
ifeq ($(TARGET_CHIP),ch592)
    TARGET_TRIPLE = --target=riscv32-none-elf
    LLVM_PATH = $(if $(RISCV_LLVM_BIN),$(RISCV_LLVM_BIN)/)
else
    TARGET_TRIPLE = --target=armv7em-none-eabi
    LLVM_PATH = $(LLVM_UTILS_PATH)
endif

# Always use llvm_for_arm's multi-architecture binary utilities
LLVM_UTILS_PATH = $(if $(ARM_LLVM_BIN),$(ARM_LLVM_BIN)/)
LLVM_NM = $(LLVM_UTILS_PATH)llvm-nm$(EXE_SUFFIX)

CC  = "$(LLVM_PATH)clang$(EXE_SUFFIX)"
AS  = $(CC)
CP  = "$(LLVM_UTILS_PATH)llvm-objcopy$(EXE_SUFFIX)"
SZ  = "$(LLVM_UTILS_PATH)llvm-size$(EXE_SUFFIX)"
LD  = $(CC)
AR  = "$(LLVM_UTILS_PATH)llvm-ar$(EXE_SUFFIX)"
NM  = "$(LLVM_NM)"

# ------------------------------------------------------------------------------
# Directories
# ------------------------------------------------------------------------------
BUILD_DIR = build

ifeq ($(OS),Windows_NT)
    SHELL = cmd.exe
    BUILD_DIR_WIN = $(subst /,\,$(BUILD_DIR))
    MKDIR = if not exist "$(BUILD_DIR_WIN)" mkdir "$(BUILD_DIR_WIN)"
    RMDIR = if exist "$(BUILD_DIR_WIN)" rd /s /q "$(BUILD_DIR_WIN)"
else
    MKDIR = mkdir -p $(BUILD_DIR)
    RMDIR = rm -rf $(BUILD_DIR)
endif

# ------------------------------------------------------------------------------
# MODUS framework
# ------------------------------------------------------------------------------
MODUS_ROOT = modus

# Build mode: BUILD=debug | BUILD=debug-rel (default) | BUILD=release
BUILD ?= debug-rel

MODUS_ENABLE ?= 1

# Module switches — default for debug; overridden by release below
MSHELL_ENABLE    = 1
MWAVEFORM_ENABLE = 1
MWAVEFORM_RTT_BUFFER_SIZE = 4096
MSTORAGE_ENABLE  = 1
MBLINFO_ENABLE   = 1
MODUS_USE_LOG    = 1

# Build mode overrides (MUST be before modus.mk include)
ifeq ($(BUILD),release)
    # Production: no debug modules, completely compiled-out MODUS
    OPT = -Os
    MODUS_ENABLE = 1
    MSHELL_ENABLE    = 0
    MWAVEFORM_ENABLE = 0
    MSTORAGE_ENABLE  = 0
    MBLINFO_ENABLE   = 0
    MODUS_USE_LOG    = 0
else ifeq ($(BUILD),debug-rel)
    # Release-close debugging: same -Os, but debug modules on
    OPT = -Os
else
    # Daily development: no optimization
    OPT = -O0
endif

# 默认关闭 MODUS 内置移植，各芯片 target.mk 可自行覆盖启用
MODUS_USE_DEFAULT_PERFC_PORT ?= 0

ifeq ($(MODUS_ENABLE),1)
    include $(MODUS_ROOT)/modus.mk
    C_DEFS += -DMODUS_ENABLE=1
else
    C_DEFS += -DMODUS_ENABLE=0 -D__NO_USE_LOG__ -D__NO_USE_ASSERT
    MODUS_INCLUDES =                                                            \
        -I$(MODUS_ROOT)                                                         \
        -I$(MODUS_ROOT)/src                                                     \
        -I$(MODUS_ROOT)/src/mdi                                                 \
        -I$(MODUS_ROOT)/src/arch                                                \
        -I$(MODUS_ROOT)/src/arch/cortex-m                                       \
        -I$(MODUS_ROOT)/src/arch/riscv                                          \
        -I$(LIB_PLOOC_DIR)                                                      \
        -I$(LIB_PERF_DIR)
    MODUS_CFLAGS = -DMSHELL_ENABLE=0 -DMWAVEFORM_ENABLE=0                       \
        -D__NO_USE_LOG__ -D__NO_USE_ASSERT
    MODUS_SRCS =                                                                \
        $(MODUS_ROOT)/src/arch/perfc_port.c                                     \
        $(MODUS_ROOT)/src/utilities/mringbuf.c
endif

LIB_PERF_DIR  = $(MODUS_ROOT)/lib/perf_counter
LIB_PLOOC_DIR = $(MODUS_ROOT)/lib/plooc

# MODUS sources are automatically handled via MODUS_SRCS from modus.mk
PERIF_LIB_SOURCES =                                                             \
    $(LIB_PERF_DIR)/perf_counter.c

PERIPHERAL_SOURCES = $(wildcard peripheral/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/$(TARGET_CHIP)/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/$(TARGET_CHIP)/mdi/*.c)
PERIPHERAL_SOURCES += $(wildcard peripheral/driver/*.c)


# MFOC numeric backend and source bindings.
MFOC_SOURCES ?=
MFOC_INCLUDES ?=
MFOC_NUMERIC ?= float
ifeq ($(MFOC_NUMERIC),float)
    C_DEFS += -DMFOC_NUMERIC_FLOAT=1
else ifeq ($(MFOC_NUMERIC),fixed)
    C_DEFS += -DMFOC_NUMERIC_FIXED=1
else
    $(error MFOC_NUMERIC must be float or fixed)
endif
ifeq ($(MFOC_NUMERIC),float)
ifneq ($(filter -DMFOC_APP_ENABLED=1,$(C_DEFS)),)
    C_DEFS += -DMWAVEFORM_PREPARE_SAMPLE=mfoc_debug_CaptureWaveform
endif
endif

# ------------------------------------------------------------------------------
# All C sources  (chip-specific vars set by target.mk)
# ------------------------------------------------------------------------------
C_SOURCES =                                                                     \
    src/main.c                                                                  \
    $(PERFC_PORT_C)                                                             \
    $(IT_C)                                                                     \
    $(SYSTEM_C)                                                                 \
    $(CHIP_SOURCES)                                                             \
    $(MODUS_SRCS)                                                               \
    $(PERIF_LIB_SOURCES)                                                        \
    $(PERIPHERAL_SOURCES)                                                       \
    $(MFOC_SOURCES)                                                              \
    $(MFOC_TARGET_SOURCES)

ifeq ($(MODUS_ENABLE),1)
    C_SOURCES += $(CLASS_SOURCES)
endif

ASM_SOURCES = $(STARTUP_S)

# ------------------------------------------------------------------------------
# Defines
# ------------------------------------------------------------------------------
TRACE_MCU_WRITE_DECL = extern void user_trace_output(const char*);
TRACE_MCU_WRITE_CALL = user_trace_output
TRACE_MCU_WRITE_VALUE = $(TRACE_MCU_WRITE_DECL) $(TRACE_MCU_WRITE_CALL)

C_DEFS +=                                                                       \
    -DMSHELL_MAX_CMDS=32                                                        \
    -D__PERFC_USE_USER_CUSTOM_PORTING__=1                                       \
    -D__C_LANGUAGE_EXTENSIONS_PERFC_PT__=1                                      \
    -D__PERFC_CFG_PORTING_INCLUDE__=\"perfc_port.h\"                            \
    -D__COMPILER_HAS_GNU_EXTENSIONS__=1                                         \
    -DTRACE_USE_LIBC_PRINTF=0                                                   \
    -DTRACE_MCU_WRITE_STRING="$(TRACE_MCU_WRITE_VALUE)"                         \
    -DMODUS_CFG_USER_CONFIG_INCLUSION="\"userconfig.h\""

# ------------------------------------------------------------------------------
# Include paths
# ------------------------------------------------------------------------------
C_INCLUDES =                                                                    \
    -I.                                                                         \
    -Isrc                                                                       \
    -I$(CMSIS_CORE)                                                             \
    -I$(CMSIS_DEV)                                                              \
    $(TARGET_INCLUDES)                                                          \
    $(MODUS_INCLUDES)                                                           \
    -I$(MODUS_ROOT)/src/utilities                                               \
    -I$(MODUS_ROOT)/src/mdebug                                                  \
    -I$(MODUS_ROOT)/src/mdebug/segger_rtt                                       \
    -I$(LIB_PLOOC_DIR)                                                          \
    -I$(LIB_PERF_DIR)                                                           \
    -Iperipheral                                                                \
    -Iperipheral/$(TARGET_CHIP)                                                 \
    -Iperipheral/driver                                                         \
    -Iclass                                                                     \
    $(MFOC_INCLUDES)

# ------------------------------------------------------------------------------
# Flags
# ------------------------------------------------------------------------------
$(info C_DEFS is $(C_DEFS))
CFLAGS  = $(TARGET_TRIPLE) $(CPU_FLAGS) $(C_DEFS) $(C_INCLUDES) $(MODUS_CFLAGS) \
        $(OPT) -g
CFLAGS += -Wall -Wextra -std=gnu11
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -fno-exceptions
CFLAGS += -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare          \
          -Wno-compare-distinct-pointer-types -Wno-unused-command-line-argument

ASFLAGS = $(TARGET_TRIPLE) $(CPU_FLAGS) -g

LDFLAGS += $(TARGET_TRIPLE) $(CPU_FLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
STD_LIBS ?= -lcrt0 -lc -lm
LDFLAGS += $(STD_LIBS)

# ------------------------------------------------------------------------------
.PHONY: all clean size flash rtt debug-rel
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin                      \
        $(BUILD_DIR)/$(TARGET).hex size

release:
	"$(MAKE)" BUILD=release all

debug-rel:
	"$(MAKE)" BUILD=debug-rel all

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
ASM_OBJECTS = $(patsubst %.s,$(BUILD_DIR)/%.o,$(patsubst %.S,$(BUILD_DIR)/%.o,  \
                $(notdir $(ASM_SOURCES))))
OBJECTS += $(ASM_OBJECTS)

vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

# 针对高频控制步相关的密集计算模块强制启用 -O2 优化
$(BUILD_DIR)/motor.o: CFLAGS += -O2
# Preserve separate-operation rounding in MFOC numeric callers.
ifneq ($(strip $(MFOC_SOURCES) $(MFOC_TARGET_SOURCES)),)
$(addprefix $(BUILD_DIR)/,$(notdir $(MFOC_SOURCES:.c=.o) $(MFOC_TARGET_SOURCES:.c=.o))): CFLAGS += -ffp-contract=off
endif

# ------------------------------------------------------------------------------
# Build configuration signature tracking
# Triggers recompile whenever target, flags, or numeric backend changes
# ------------------------------------------------------------------------------
BUILD_CONFIG_SIGNATURE = target_$(TARGET_CHIP)-build_$(BUILD)-num_$(MFOC_NUMERIC)\
                        -mod_$(MODUS_ENABLE)

CONFIG_STAMP = $(BUILD_DIR)/.config_stamp

ifeq ($(OS),Windows_NT)
    CONFIG_STAMP_WIN = $(BUILD_DIR_WIN)\.config_stamp
    PREV_CONFIG := $(shell if exist "$(CONFIG_STAMP_WIN)" type "$(CONFIG_STAMP_WIN)")
else
    PREV_CONFIG := $(shell cat $(CONFIG_STAMP) 2>/dev/null)
endif

ifneq ($(strip $(PREV_CONFIG)),$(strip $(BUILD_CONFIG_SIGNATURE)))
.PHONY: $(CONFIG_STAMP)
endif

$(CONFIG_STAMP): | $(BUILD_DIR)
ifeq ($(OS),Windows_NT)
	@echo $(BUILD_CONFIG_SIGNATURE)>"$(CONFIG_STAMP_WIN)"
else
	@echo $(BUILD_CONFIG_SIGNATURE) > $(CONFIG_STAMP)
endif

$(OBJECTS): $(CONFIG_STAMP) makefile $(wildcard config.local.mk)

-include $(OBJECTS:.o=.d)

$(BUILD_DIR)/%.o: %.c $(CONFIG_STAMP) | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -R .ARM.attributes $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex -R .ARM.attributes -R ._user_heap_stack $< $@

$(BUILD_DIR):
	$(MKDIR)

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

clean:
	$(RMDIR)

# ------------------------------------------------------------------------------
# Flash / Debug
# ------------------------------------------------------------------------------

PROBE_PROFILE ?= default
OPENOCD_CONFIG = -f target/$(TARGET_CHIP)/openocd.cfg
OPENOCD_SPEED_KHZ = $(SWD_SPEED_KHZ)

ifneq ($(filter at32f407 at32f413 at32f421,$(TARGET_CHIP)),)
    ifneq ($(strip $(AT32_OPENOCD_ROOT)),)
        OPENOCD_BIN ?= $(AT32_OPENOCD_ROOT)/bin/openocd$(EXE_SUFFIX)
        OPENOCD_SCRIPTS ?= $(AT32_OPENOCD_ROOT)/scripts
    else ifeq ($(OS),Windows_NT)
        OPENOCD_BIN ?= $(if $(MSYS2_BIN),$(MSYS2_BIN)/)openocd-at32.exe
        ifneq ($(strip $(MSYS2_BIN)),)
            OPENOCD_SCRIPTS ?= $(MSYS2_BIN)/../share/openocd/scripts
        endif
    endif
    ifeq ($(PROBE_PROFILE),atlink-ez-winusb)
        OPENOCD_CONFIG = -f interface/cmsis-dap.cfg                              \
            -c "cmsis_dap_backend usb_bulk" -c "cmsis_dap_usb interface 3"     \
            -c "cmsis_dap_vid_pid 0x2e3c 0xf000"                                \
            $(if $(ATLINK_EZ_SERIAL),-c "adapter serial $(ATLINK_EZ_SERIAL)")   \
            -c "transport select swd" -f target/$(TARGET_CHIP)xx.cfg
    endif
else ifeq ($(TARGET_CHIP),ch592)
    ifneq ($(strip $(WCH_OPENOCD_ROOT)),)
        OPENOCD_BIN ?= $(WCH_OPENOCD_ROOT)/bin/openocd$(EXE_SUFFIX)
        OPENOCD_SCRIPTS ?= $(WCH_OPENOCD_ROOT)/scripts
    endif
    OPENOCD_SPEED_KHZ = 1000
else
    OPENOCD_BIN ?= $(or $(STM32_OPENOCD_BIN),openocd)
    OPENOCD_SCRIPTS ?= $(STM32_OPENOCD_SCRIPTS)
endif

OPENOCD_BIN ?= openocd
OPENOCD_CMD ?= "$(OPENOCD_BIN)" $(if $(OPENOCD_SCRIPTS),-s "$(OPENOCD_SCRIPTS)") \
    $(OPENOCD_CONFIG) $(OPENOCD_TARGET_ARGS)                                      \
    $(if $(OPENOCD_SPEED_KHZ),-c "adapter speed $(OPENOCD_SPEED_KHZ)")           \
    $(if $(filter Windows_NT,$(OS)),-c "tcl_port 0")

FLASH_CMD ?= $(OPENOCD_CMD) -c "init" -c "reset halt"                           \
    -c "sleep 200" -c "program $< verify" -c "reset run" -c "exit"

flash: $(BUILD_DIR)/$(TARGET).hex
	$(FLASH_CMD)

download:
	$(OPENOCD_CMD) -c "init" -c "reset halt" -c "sleep 200"                        \
	    -c "program $(BUILD_DIR)/$(TARGET).hex verify" -c "reset run" -c "exit"

debug-server:

	$(OPENOCD_CMD)

openocd: debug-server

debug: $(BUILD_DIR)/$(TARGET).elf
	gdb-multiarch -x debug.gdb $<

# ------------------------------------------------------------------------------
# RTT
# ------------------------------------------------------------------------------
ifeq ($(OS),Windows_NT)
    RTT_ADDR = $(shell powershell -NoProfile -Command                           \
        "$$nm = & '$(LLVM_NM)' '$(BUILD_DIR)/$(TARGET).elf' 2>$$null |           \
         Select-String '_SEGGER_RTT$$'; if ($$nm) { '0x' +                      \
         ($$nm -split ' ')[0] }" 2>nul)
else
    RTT_ADDR = $(shell $(NM) $(BUILD_DIR)/$(TARGET).elf 2>/dev/null |           \
        awk '/_SEGGER_RTT$$/ {print "0x"$$1}')
endif

RTT_POLL_MS ?= 1
RTT_SHELL_PORT ?= 9090
RTT_WAVE_PORT ?= 9091
RTT_SERVERS = -c "rtt server start $(RTT_SHELL_PORT) 0"                            \
    -c "rtt server start $(RTT_WAVE_PORT) 1"

RTT_CMD ?= -c "init" -c "catch { resume }"                                      \
    -c "rtt setup $(or $(RTT_ADDR),$(error RTT control block missing. Use BUILD=debug or BUILD=debug-rel)) 0xa8 \"SEGGER RTT\"" \
    -c "rtt start" -c "rtt polling_interval $(RTT_POLL_MS)"                      \
    $(RTT_SERVERS)

rtt-addr: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"

rtt: $(BUILD_DIR)/$(TARGET).elf
	@echo "RTT CB address: $(RTT_ADDR)"
	$(OPENOCD_CMD) $(RTT_CMD)

ifeq ($(OS),Windows_NT)
.PHONY: auto
auto:
	$(if $(filter release,$(BUILD)),$(error auto requires BUILD=debug or BUILD=debug-rel for RTT))
	@powershell -NoProfile -Command "if (Get-NetTCPConnection -State Listen -LocalPort 4444,$(RTT_SHELL_PORT),$(RTT_WAVE_PORT) -ErrorAction SilentlyContinue) { Write-Error 'OpenOCD or RTT port is already in use'; exit 1 }"
	"$(MAKE)" clean
	"$(MAKE)" all
	"$(MAKE)" flash
	@start "" /b "$(MAKE_COMMAND)" rtt > openocd_rtt.log 2>&1
	@powershell -NoProfile -Command "$$deadline = (Get-Date).AddSeconds(10); do { if ((Get-NetTCPConnection -State Listen -LocalPort $(RTT_SHELL_PORT),$(RTT_WAVE_PORT) -ErrorAction SilentlyContinue | Select-Object -ExpandProperty LocalPort -Unique | Measure-Object).Count -eq 2) { exit 0 }; Start-Sleep -Milliseconds 250 } while ((Get-Date) -lt $$deadline); Write-Error 'RTT server did not start. See openocd_rtt.log'; exit 1"
endif

# Flash via already-running OpenOCD telnet (port 4444)
flash-rtt: $(BUILD_DIR)/$(TARGET).hex
ifeq ($(OS),Windows_NT)
	@powershell -NoProfile -Command                                                \
	    "$$hex = (Resolve-Path '$<').Path;                                         \
	     $$c = New-Object System.Net.Sockets.TcpClient('localhost', 4444);         \
	     $$s = $$c.GetStream();                                                    \
	     $$w = New-Object System.IO.StreamWriter($$s);                             \
	     $$w.WriteLine(\"program $$hex verify reset\"); $$w.WriteLine('exit');     \
	     $$w.Flush(); Start-Sleep 2; $$c.Close()"
else
	@ABS=$$(realpath $<);                                                          \
	printf "program $$ABS verify reset\nexit\n" | nc -w 10 localhost 4444;         \
	sleep 1;                                                                       \
	printf "rtt stop\nrtt start\n" | nc -w 3 localhost 4444
endif

# ------------------------------------------------------------------------------
# Info
# ------------------------------------------------------------------------------
info:
	@echo "TARGET      = $(TARGET)"
	@echo "TARGET_CHIP = $(TARGET_CHIP)"
	@echo "BUILD_DIR   = $(BUILD_DIR)"
	@echo "LDSCRIPT    = $(LDSCRIPT)"
	@echo "LLVM_PATH   = $(LLVM_PATH)"
	@echo "C_SOURCES   = $(C_SOURCES)"
	@echo "ASM_SOURCES = $(ASM_SOURCES)"

.PHONY: all release debug-rel clean flash download flash-rtt debug-server       \
    debug size info rtt rtt-addr openocd
