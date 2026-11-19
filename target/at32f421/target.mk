# =============================================================================
# target/at32f421/target.mk
# AT32F421F8P7 — Cortex-M4, no FPU
# =============================================================================

# CPU architecture
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=soft

# Chip preprocessor defines
C_DEFS += -DAT32F421F8P7 -DUSE_STDPERIPH_DRIVER

# CMSIS / peripheral library paths
CHIPLIB_ROOT = vendor/cortex-m/AT32F421_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/at32f421/AT32F421x8_FLASH.ld
STARTUP_S    = $(CMSIS_DEV)/startup/gcc/startup_at32f421.s
SYSTEM_C     = $(CMSIS_DEV)/system_at32f421.c
IT_C         = target/at32f421/at32f421_it.c


# Chip-specific include paths
TARGET_INCLUDES = -I$(DRV_INC) -Itarget/at32f421

# ------------------------------------------------------------------
# AT32F421 peripheral driver selection
# ------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_SCFG   ?= 0
USE_DRV_TMR    ?= 0
USE_DRV_USART  ?= 1
USE_DRV_SPI    ?= 0
USE_DRV_I2C    ?= 0
USE_DRV_DMA    ?= 0
USE_DRV_ADC    ?= 0
USE_DRV_CMP    ?= 0
USE_DRV_DEBUG  ?= 1
USE_DRV_EXINT  ?= 0
USE_DRV_WDT    ?= 0
USE_DRV_WWDT   ?= 0
USE_DRV_CRC    ?= 0
USE_DRV_PWC    ?= 0

DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_flash.c
endif
ifeq ($(USE_DRV_SCFG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_scfg.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_tmr.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_usart.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_i2c.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_dma.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_adc.c
endif
ifeq ($(USE_DRV_CMP),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_cmp.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_debug.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_exint.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_wwdt.c
endif
ifeq ($(USE_DRV_CRC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_crc.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f421_pwc.c
endif

CHIP_SOURCES = $(DRV_SOURCES)

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1

