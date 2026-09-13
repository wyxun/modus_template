# =============================================================================
# target/at32f407/target.mk
# AT32F407VGT7 — Cortex-M4F, single-precision FPU (AT-START-F407)
# =============================================================================

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DAT32F407VGT7 -DUSE_STDPERIPH_DRIVER
# CMSIS / peripheral library paths
CHIPLIB_ROOT = vendor/cortex-m/AT32F403A_407_Firmware_Library/libraries
CMSIS_CORE   = vendor/cortex-m/cmsis_core
CMSIS_DEV    = $(CHIPLIB_ROOT)/cmsis/cm4/device_support
DRV_INC      = $(CHIPLIB_ROOT)/drivers/inc
DRV_SRC      = $(CHIPLIB_ROOT)/drivers/src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/at32f407/AT32F407xC_FLASH.ld
STARTUP_S    = $(CMSIS_DEV)/startup/gcc/startup_at32f403a_407.s
SYSTEM_C     = $(CMSIS_DEV)/system_at32f403a_407.c
IT_C         = target/at32f407/at32f407_it.c

# Chip-specific include paths
TARGET_INCLUDES = -I$(DRV_INC) -Itarget/at32f407 \
                  -Ivendor/cortex-m/AT32F403A_407_Firmware_Library/middlewares/usbd_drivers/inc \
                  -Ivendor/cortex-m/AT32F403A_407_Firmware_Library/middlewares/usbd_class/cdc

# ------------------------------------------------------------------
# AT32F407 peripheral driver selection
# ------------------------------------------------------------------
USE_DRV_CRM    ?= 1
USE_DRV_GPIO   ?= 1
USE_DRV_MISC   ?= 1
USE_DRV_FLASH  ?= 1
USE_DRV_USART  ?= 1
USE_DRV_DEBUG  ?= 1
USE_DRV_TMR    ?= 1
USE_DRV_ADC    ?= 0
USE_DRV_DMA    ?= 1
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
USE_DRV_USB    ?= 1
USE_DRV_ACC    ?= 1

DRV_SOURCES =
ifeq ($(USE_DRV_CRM),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_crm.c
endif
ifeq ($(USE_DRV_GPIO),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_gpio.c
endif
ifeq ($(USE_DRV_MISC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_misc.c
endif
ifeq ($(USE_DRV_FLASH),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_flash.c
endif
ifeq ($(USE_DRV_USART),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_usart.c
endif
ifeq ($(USE_DRV_DEBUG),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_debug.c
endif
ifeq ($(USE_DRV_TMR),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_tmr.c
endif
ifeq ($(USE_DRV_ADC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_adc.c
endif
ifeq ($(USE_DRV_DMA),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_dma.c
endif
ifeq ($(USE_DRV_EXINT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_exint.c
endif
ifeq ($(USE_DRV_SPI),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_spi.c
endif
ifeq ($(USE_DRV_I2C),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_i2c.c
endif
ifeq ($(USE_DRV_CAN),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_can.c
endif
ifeq ($(USE_DRV_WDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_wdt.c
endif
ifeq ($(USE_DRV_WWDT),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_wwdt.c
endif
ifeq ($(USE_DRV_PWC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_pwc.c
endif
ifeq ($(USE_DRV_USB),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_usb.c
endif
ifeq ($(USE_DRV_ACC),1)
    DRV_SOURCES += $(DRV_SRC)/at32f403a_407_acc.c
endif

USB_MID_SRC = vendor/cortex-m/AT32F403A_407_Firmware_Library/middlewares
CHIP_SOURCES = $(DRV_SOURCES) \
               $(USB_MID_SRC)/usbd_drivers/src/usbd_core.c \
               $(USB_MID_SRC)/usbd_drivers/src/usbd_int.c \
               $(USB_MID_SRC)/usbd_drivers/src/usbd_sdr.c \
               $(USB_MID_SRC)/usbd_class/cdc/cdc_class.c \
               $(USB_MID_SRC)/usbd_class/cdc/cdc_desc.c

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1

# OpenOCD (AT32 custom v0.11)
OPENOCD_BIN     = $(SW_ROOT)/msys64/mingw64/bin/openocd-at32.exe
OPENOCD_SCRIPTS = $(SW_ROOT)/msys64/mingw64/share/openocd/scripts
