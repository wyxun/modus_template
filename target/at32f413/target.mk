# =============================================================================
# target/at32f413/target.mk
# AT32F413RCT7 — Cortex-M4F, single-precision FPU (Motor EVB V1)
# =============================================================================

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DAT32F413RCT7 -DUSE_STDPERIPH_DRIVER                                \
    -DAT_MOTOR_EVB_V1 -DCORE_DEBUG_OVERRIDE_FAULT_HANDLER

# The second board validates the FIXED control path by default.
MFOC_NUMERIC ?= fixed
AT32F413_MFOC_HALL ?= 0
TARGET_CONFIG_SIGNATURE = -hall_$(AT32F413_MFOC_HALL)
C_DEFS += -DMFOC_APP_ENABLED=1 -DMFOC_APP_ENCODER_DIAGNOSTIC=0 \
    -DMFOC_USE_PERFC_FSM=1 -DMFOC_TRIG_BACKEND=1 -DMFOC_PERFC_HZ=200000000U \
    -DMFOC_CORE_M_MAX=0.35f
ifeq ($(AT32F413_MFOC_HALL),1)
    ifeq ($(strip $(AT32F413_HALL_CONFIG_DIR)),)
        $(error Set AT32F413_HALL_CONFIG_DIR to the measured Hall table directory)
    endif
    TARGET_CONFIG_SIGNATURE += -hallcfg_$(AT32F413_HALL_CONFIG_DIR)
    C_DEFS += -DMFOC_FEEDBACK_ENABLE_POSITION=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0 \
        -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_POSITION
else ifeq ($(AT32F413_MFOC_HALL),0)
    C_DEFS += -DMFOC_FEEDBACK_ENABLE_POSITION=0
else
    $(error AT32F413_MFOC_HALL must be 0 or 1)
endif

# CMSIS / peripheral library paths
CHIPLIB_ROOT = vendor/cortex-m/AT32F413_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/at32f413/AT32F413xC_FLASH.ld
STARTUP_S    = $(CMSIS_DEV)/startup/gcc/startup_at32f413.s
SYSTEM_C     = $(CMSIS_DEV)/system_at32f413.c
IT_C         = target/at32f413/at32f413_it.c


# Chip-specific include paths
TARGET_INCLUDES = -I$(DRV_INC) -Itarget/at32f413
ifeq ($(AT32F413_MFOC_HALL),1)
    TARGET_INCLUDES += -I"$(AT32F413_HALL_CONFIG_DIR)"
endif

# ------------------------------------------------------------------
# AT32F413 peripheral driver selection
# ------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_TMR    ?= 1
USE_DRV_USART  ?= 1
USE_DRV_ADC    ?= 1
USE_DRV_DMA    ?= 1
USE_DRV_DEBUG  ?= 1
USE_DRV_EXINT  ?= 0
USE_DRV_SPI    ?= 0
USE_DRV_I2C    ?= 0
USE_DRV_CAN    ?= 0
USE_DRV_WDT    ?= 0
USE_DRV_WWDT   ?= 0
USE_DRV_PWC    ?= 0
USE_DRV_RTC    ?= 0
USE_DRV_BPR    ?= 0
USE_DRV_CRC    ?= 0
USE_DRV_SDIO   ?= 0
USE_DRV_USB    ?= 0
USE_DRV_ACC    ?= 0

DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_flash.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_tmr.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_usart.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_adc.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_dma.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_debug.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_exint.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_i2c.c
endif
ifeq ($(USE_DRV_CAN),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_can.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_wwdt.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f413_pwc.c
endif

CHIP_SOURCES = $(DRV_SOURCES)

include mfoc/mfoc.mk
# Both ADC vectors use the same compile-time bound Motor IRQ body.
MFOC_TARGET_SOURCES = target/stm32g431/mfoc_motor_irq.c \
    peripheral/at32f413/mfoc/mfoc_port.c
CLASS_SOURCES = class/mfoc_app.c class/mfoc_debug.c

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1
