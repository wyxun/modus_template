# =============================================================================
# target/stm32g431/target.mk
# STM32G431xx — Cortex-M4F, single-precision FPU
# =============================================================================

# FOC motor control target (grblHAL is not built on this chip).
# FOC high-frequency ISR is connected via ADC1 JEOS interrupt →
# foc_app_HighFrequencyISR() → motor_HighFrequencyStep() (see
# stm32g4xx_it.c ADC1_2_IRQHandler and haladc.c JEOS/NVIC enable).

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DSTM32G431xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
C_DEFS += -DFOC_TRIG_BACKEND=2
C_DEFS += -DFOC_HF_ISR_HZ=20000U
C_DEFS += -DFOC_DCBUS_SOURCE=2U
# 12 V operation: PB10=0, Q48 off, R30=18 kOhm is the lower divider leg.
C_DEFS += -DHALADC_VBUS_RANGE_48V=0U
C_DEFS += -DFOC_DCBUS_MV_PER_COUNT_NUM=20570U
C_DEFS += -DFOC_DCBUS_MV_PER_COUNT_DEN=2457U
C_DEFS += -DFOC_DCBUS_OFFSET_MILLIVOLT=0
C_DEFS += -DFOC_SUPPORT=1
C_DEFS += -DFOC_ENCODER_STATIC_BINDING=1
C_DEFS += -DMDI_MAIN_STREAM_RESOURCE=board_stream

# CMSIS paths
CMSIS_CORE = vendor/cortex-m/cmsis_core
CMSIS_DEV  = vendor/cortex-m/cmsis_device_g4/Include

# HAL paths
HAL_INC = vendor/cortex-m/stm32g4xx_hal_driver/Inc
HAL_SRC = vendor/cortex-m/stm32g4xx_hal_driver/Src

# Linker script, startup, system, interrupt handler, perf_counter port
LDSCRIPT     = target/stm32g431/STM32G431xB_FLASH.ld
STARTUP_S    = vendor/cortex-m/cmsis_device_g4/Source/Templates/gcc/startup_stm32g431xx.s
SYSTEM_C     = vendor/cortex-m/cmsis_device_g4/Source/Templates/system_stm32g4xx.c
IT_C         = target/stm32g431/stm32g4xx_it.c


# Chip-specific include paths
TARGET_INCLUDES = -I$(HAL_INC) -Itarget/stm32g431

# HAL sources (selective — only what's used by peripherals)
HAL_SOURCES = \
    $(HAL_SRC)/stm32g4xx_hal.c \
    $(HAL_SRC)/stm32g4xx_hal_cortex.c \
    $(HAL_SRC)/stm32g4xx_hal_rcc.c \
    $(HAL_SRC)/stm32g4xx_hal_rcc_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_gpio.c \
    $(HAL_SRC)/stm32g4xx_hal_pwr.c \
    $(HAL_SRC)/stm32g4xx_hal_pwr_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_flash.c \
    $(HAL_SRC)/stm32g4xx_hal_flash_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_dma.c \
    $(HAL_SRC)/stm32g4xx_hal_dma_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_exti.c \
    $(HAL_SRC)/stm32g4xx_hal_fdcan.c \
    $(HAL_SRC)/stm32g4xx_hal_i2c.c \
    $(HAL_SRC)/stm32g4xx_hal_i2c_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_tim.c \
    $(HAL_SRC)/stm32g4xx_hal_tim_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_comp.c \
    $(HAL_SRC)/stm32g4xx_hal_dac.c \
    $(HAL_SRC)/stm32g4xx_hal_dac_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_opamp.c \
    $(HAL_SRC)/stm32g4xx_hal_opamp_ex.c \
    $(HAL_SRC)/stm32g4xx_hal_adc.c \
    $(HAL_SRC)/stm32g4xx_hal_adc_ex.c

# LL sources (selective)
LL_SOURCES = \
    $(HAL_SRC)/stm32g4xx_ll_adc.c \
    $(HAL_SRC)/stm32g4xx_ll_comp.c \
    $(HAL_SRC)/stm32g4xx_ll_dac.c \
    $(HAL_SRC)/stm32g4xx_ll_gpio.c \
    $(HAL_SRC)/stm32g4xx_ll_opamp.c \
    $(HAL_SRC)/stm32g4xx_ll_pwr.c \
    $(HAL_SRC)/stm32g4xx_ll_rcc.c \
    $(HAL_SRC)/stm32g4xx_ll_tim.c \
    $(HAL_SRC)/stm32g4xx_ll_usart.c \
    $(HAL_SRC)/stm32g4xx_ll_utils.c

CHIP_SOURCES = $(HAL_SOURCES) $(LL_SOURCES)

# FOC framework — STM32G431 supports FOC with CORDIC trig backend
include foc/foc.mk

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1
