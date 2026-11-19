# =============================================================================
# target/ch592/target.mk
# WeAct Studio CH592F — RISC-V 32-bit QingKe V4C 内核
# =============================================================================

# CPU architecture flags for RISC-V RV32IMAC
CPU_FLAGS = -march=rv32imac -mabi=ilp32

# Chip preprocessor defines
C_DEFS += -DCH592 -D__riscv -DUSERCONFIG_MSHELL_ON_SERIAL=1

# CMSIS / Peripheral Library Paths
CHIPLIB_ROOT = vendor/riscv/ch592/EVT/EXAM/SRC
SYS_INC_PATH ?= $(if $(ARM_LLVM_BIN),$(ARM_LLVM_BIN)/../lib/clang-runtimes/arm-none-eabi/armv7m_soft_nofp/include)
TARGET_INCLUDES = -I$(CHIPLIB_ROOT)/StdPeriphDriver/inc -I$(CHIPLIB_ROOT)/RVMSIS -Itarget/ch592 $(if $(SYS_INC_PATH),-I"$(SYS_INC_PATH)") -include stddef.h

# Linker script, startup, interrupt handler, perf_counter port
LDSCRIPT     = target/ch592/CH592_FLASH.ld
STARTUP_S    = target/ch592/startup_CH592.S target/ch592/ch592_exception.S
IT_C         = target/ch592/ch592_it.c

# Chip core sources (scanned automatically via PERIPHERAL_SOURCES)
CHIP_SOURCES = \
    $(CHIPLIB_ROOT)/StdPeriphDriver/CH59x_gpio.c \
    $(CHIPLIB_ROOT)/StdPeriphDriver/CH59x_uart1.c \
    $(CHIPLIB_ROOT)/StdPeriphDriver/CH59x_sys.c \
    $(CHIPLIB_ROOT)/StdPeriphDriver/CH59x_clk.c

# Use LLVM LLD linker for cross-architecture ELF linking, disable standard C library dependency
STD_LIBS =
LDFLAGS += -fuse-ld=lld -nostdlib
LDFLAGS += -L$(CHIPLIB_ROOT)/StdPeriphDriver -lISP592

# ------------------------------------------------------------------------------
# Flash configuration for CH592
# Supports: 'openocd' (default, uses WCH-LinkE), 'wchisp' (USB ISP), 'wchcmd' (official WCHISPTool_CMD)
# ------------------------------------------------------------------------------
CH592_FLASH_METHOD ?= openocd

ifeq ($(CH592_FLASH_METHOD),openocd)
    OPENOCD_TARGET_ARGS = -c "chip_id CH59x"
    # WCH's OpenOCD adds RTT servers but has no polling_interval command.
    RTT_CMD ?= -c "init" $(RTT_SERVERS)
else ifeq ($(CH592_FLASH_METHOD),wchisp)
    FLASH_CMD = wchisp flash $<
else ifeq ($(CH592_FLASH_METHOD),wchcmd)
    ifeq ($(OS),Windows_NT)
        CONFIG_INI = target/ch592/config.ini
        FLASH_CMD = WCHISPTool_CMD -p USB -c $(CONFIG_INI) -o program -f $<
    else
        CONFIG_INI = target/ch592/config.ini
        FLASH_CMD = WCHISPTool_CMD -p /dev/ch37x -c $(CONFIG_INI) -o program -f $<
    endif
else
    # Auto-detect behavior (backward compatibility: wchisp > wchcmd)
    ifeq ($(OS),Windows_NT)
        WCHISP_PATH := $(shell where wchisp 2>nul)
        WCHCMD_PATH := $(shell where WCHISPTool_CMD 2>nul)
        ifneq ($(WCHISP_PATH),)
            FLASH_CMD = wchisp flash $<
        else ifneq ($(WCHCMD_PATH),)
            CONFIG_INI = target/ch592/config.ini
            ifeq ($(wildcard $(CONFIG_INI)),)
                FLASH_CMD = @echo [ERROR] WCHISPTool_CMD requires target/ch592/config.ini && \
                            echo Please open WchIspStudio GUI, select CH592, configure and click 'File -> Save UI Config' to save as 'target/ch592/config.ini' && \
                            exit 1
            else
                FLASH_CMD = WCHISPTool_CMD -p USB -c $(CONFIG_INI) -o program -f $<
            endif
        else
            # Fall back to openocd if no ISP tool is found
        endif
    else
        WCHISP_PATH := $(shell which wchisp 2>/dev/null)
        WCHCMD_PATH := $(shell which WCHISPTool_CMD 2>/dev/null)
        ifneq ($(WCHISP_PATH),)
            FLASH_CMD = wchisp flash $<
        else ifneq ($(WCHCMD_PATH),)
            CONFIG_INI = target/ch592/config.ini
            ifeq ($(wildcard $(CONFIG_INI)),)
                FLASH_CMD = @echo "[ERROR] WCHISPTool_CMD requires target/ch592/config.ini" && \
                            echo "Please open WchIspStudio GUI, select CH592, configure and click 'File -> Save UI Config' to save as 'target/ch592/config.ini'" && \
                            exit 1
            else
                FLASH_CMD = WCHISPTool_CMD -p /dev/ch37x -c $(CONFIG_INI) -o program -f $<
            endif
        else
            # Fall back to openocd if no ISP tool is found
        endif
    endif
endif

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1


