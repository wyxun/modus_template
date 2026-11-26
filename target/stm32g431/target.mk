# =============================================================================
# target/stm32g431/target.mk
# STM32G431xx — Cortex-M4F, single-precision FPU
# =============================================================================

# FOC motor control target (grblHAL is not built on this chip).
# MFOC initializes stopped; an explicit align command starts current control.
# Calibration polls ADC frames before bridge enable starts the control IRQ.

# CPU architecture (FPU enabled)
CPU_FLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16

# Chip preprocessor defines
C_DEFS += -DSTM32G431xx -DUSE_HAL_DRIVER -DUSE_FULL_LL_DRIVER
# 12 V operation: PB10=0, Q48 off, R30=18 kOhm is the lower divider leg.
C_DEFS += -DHALADC_VBUS_RANGE_48V=0U
C_DEFS += -DMFOC_APP_ENABLED=1
C_DEFS += -DMFOC_USE_PERFC_FSM=1
# Numeric backend is selected once by the top-level MFOC_NUMERIC setting.
C_DEFS += -DMFOC_TRIG_BACKEND=1

# TIM1 PWM and ADC sampling share the current-loop rate.
STM32G431_CONTROL_HZ ?= 20000
TARGET_CONFIG_SIGNATURE = -hz_$(STM32G431_CONTROL_HZ)
C_DEFS += -DMFOC_MOTOR_RUN_FREQ_HZ=$(STM32G431_CONTROL_HZ)U \
    -DMFOC_PERFC_HZ=170000000U -DMFOC_CORE_M_MAX=0.55f

# FLOAT EKF at the normal control rate. Optimize only its covariance update TU.
ifeq ($(MFOC_G431_FEEDBACK),ekf)
    ifneq ($(BUILD),debug)
        MFOC_TARGET_O2_OBJECT := mfoc_ekf
    endif
endif

# FIXED release timing: stagger SMO and speed PI; shorten Core limit peaks.
ifeq ($(MFOC_NUMERIC),fixed)
    C_DEFS += -DMFOC_SMO_STAGGER_SPEED_PI=1 -DMFOC_CORE_LIMIT_DIV32=1
    TARGET_CONFIG_SIGNATURE += -fixed_control
    ifneq ($(BUILD),debug)
        MFOC_TARGET_O2_OBJECT := mfoc_core
    endif
endif

# Timing is enabled for debug builds and compiled out of release builds.
ifeq ($(BUILD),release)
MFOC_ISR_MEASURE ?= 0
else
MFOC_ISR_MEASURE ?= 1
endif
C_DEFS += -DMFOC_ISR_MEASURE=$(MFOC_ISR_MEASURE)
C_DEFS += -DMDI_MAIN_STREAM_RESOURCE=board_stream

# Bench-only probe, with the release optimizer and no waveform sampling.
# TIM1 transfers the three PWM preloads at the top update.
C_DEFS += -DMFOC_G431_PWM_PHASE=2 -DMFOC_PWM_PRELOAD_DELAY=1
TARGET_CONFIG_SIGNATURE += -pwm_phase_2

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
TARGET_INCLUDES = -I$(HAL_INC) -Itarget/stm32g431 -I$(MFOC_PROJECT_BOARD)

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

include mfoc/mfoc.mk
MFOC_TARGET_SOURCES = $(MFOC_ROOT)/port/modus/mfoc_motor_irq.c \
    $(MFOC_PROJECT_BOARD)/mfoc_port.c $(MFOC_PROJECT_BOARD)/mfoc_pwm_phase.c
CLASS_SOURCES = class/mfoc_app.c class/mfoc_debug.c

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1
