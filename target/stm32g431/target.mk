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
STM32G431_EKF ?= 0
ifeq ($(STM32G431_EKF),1)
    ifeq ($(MFOC_NUMERIC),fixed)
        $(error STM32G431_EKF=1 requires MFOC_NUMERIC=float)
    endif
    C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_EKF \
        -DMFOC_FEEDBACK_ENABLE_EKF=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0
    TARGET_CONFIG_SIGNATURE += -ekf
    ifneq ($(BUILD),debug)
        MFOC_TARGET_O2_OBJECT := mfoc_ekf
    endif
else ifneq ($(STM32G431_EKF),0)
    $(error STM32G431_EKF must be 0 or 1)
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
MFOC_P10_TIMING ?= 0
MFOC_P10_PHASE ?= 0
MFOC_P10_SEGMENTS ?= 0
MFOC_P10_PHYSICAL ?= 0
MFOC_P10_HFI_CAPTURE ?= 0
MFOC_P10_KCL ?= 0
MFOC_P10_SYNC_SPEED ?= 0
MFOC_P10_LIMIT ?= 0
MFOC_P10_EXIT ?= 0
MFOC_P10_HFI_MPU ?= 0
ifneq ($(filter-out 0,$(MFOC_P10_PHASE) $(MFOC_P10_SEGMENTS)),)
    ifneq ($(MFOC_P10_TIMING),1)
        $(error P10 phase/segment probes require MFOC_P10_TIMING=1)
    endif
endif
ifeq ($(MFOC_P10_TIMING),1)
    ifneq ($(BUILD)/$(MWAVEFORM_ENABLE)/$(MFOC_ISR_MEASURE),debug-rel/0/0)
        $(error P10 requires BUILD=debug-rel MWAVEFORM_ENABLE=0 MFOC_ISR_MEASURE=0)
    endif
    MFOC_P10_PROFILE ?= $(if $(filter 1,$(STM32G431_EKF)),ekf,flux)
    ifeq ($(STM32G431_EKF),1)
        ifneq ($(MFOC_P10_PROFILE),ekf)
            $(error STM32G431_EKF=1 requires MFOC_P10_PROFILE=ekf)
        endif
    else ifeq ($(MFOC_P10_PROFILE),identification)
        C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_FLUX -DMFOC_IDENTIFICATION_ENABLE=1
    else ifeq ($(MFOC_P10_PROFILE),flux)
        C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_FLUX
    else ifeq ($(MFOC_P10_PROFILE),smo)
        C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_SMO \
            -DMFOC_FEEDBACK_ENABLE_SMO=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0
    else ifeq ($(MFOC_P10_PROFILE),hfi)
        C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_HFI \
            -DMFOC_FEEDBACK_ENABLE_HFI=1 -DMFOC_FEEDBACK_ENABLE_FLUX=0
    else
        $(error MFOC_P10_PROFILE must be flux, smo, hfi, or identification)
    endif
    C_DEFS += -DMFOC_P10_TIMING=1
    ifeq ($(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS),0/0)
        MFOC_P10_SOURCE = target/stm32g431/mfoc_p10_timing.c
    else
        ifeq ($(filter 1/0 1/1 2/0 2/1,$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)),)
            $(error P10 candidates require phase 1 or 2 and segments 0 or 1)
        endif
        C_DEFS += -DMFOC_P10_PHASE=$(MFOC_P10_PHASE) -DMFOC_P10_SEGMENTS=$(MFOC_P10_SEGMENTS) -DMFOC_TRACE_ENABLE=1
        MFOC_P10_SOURCE = target/stm32g431/mfoc_p10_candidate.c
    endif
    TARGET_CONFIG_SIGNATURE += -p10_$(MFOC_P10_PROFILE)_$(MFOC_P10_PHASE)_$(MFOC_P10_SEGMENTS)
endif
ifeq ($(MFOC_P10_PHYSICAL),1)
    ifneq ($(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(or $(MFOC_RUNTIME_BUS),0)/$(or $(MFOC_NUMERIC),float),1/2/0/0/float)
        $(error P10 physical tests require B endpoints, FLOAT and fixed bus)
    endif
    C_DEFS += -DMFOC_P10_PHYSICAL=1
    MFOC_P10_SOURCE += target/stm32g431/mfoc_p10_physical.c
    TARGET_CONFIG_SIGNATURE += -p10physical
else ifneq ($(MFOC_P10_PHYSICAL),0)
    $(error MFOC_P10_PHYSICAL must be 0 or 1)
endif
ifeq ($(MFOC_P10_HFI_CAPTURE),1)
    ifneq ($(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(MFOC_P10_PROFILE)/$(MFOC_P10_PHYSICAL),1/2/0/hfi/0)
        $(error HFI capture requires the B HFI endpoint profile)
    endif
    C_DEFS += -DMFOC_P10_HFI_CAPTURE=1
    TARGET_CONFIG_SIGNATURE += -p10_hfi_capture
else ifneq ($(MFOC_P10_HFI_CAPTURE),0)
    $(error MFOC_P10_HFI_CAPTURE must be 0 or 1)
endif
ifeq ($(MFOC_P10_KCL),1)
    ifeq ($(filter 1/2/0/flux/0/float 1/2/0/smo/0/float,$(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(MFOC_P10_PROFILE)/$(MFOC_P10_PHYSICAL)/$(or $(MFOC_NUMERIC),float)),)
        $(error KCL capture requires the B FLOAT Flux or SMO endpoint profile)
    endif
    C_DEFS += -DMFOC_P10_KCL=1
    MFOC_P10_SOURCE += target/stm32g431/mfoc_p10_kcl.c
    TARGET_CONFIG_SIGNATURE += -p10_kcl
else ifneq ($(MFOC_P10_KCL),0)
    $(error MFOC_P10_KCL must be 0 or 1)
endif
ifeq ($(MFOC_P10_SYNC_SPEED),1)
    ifneq ($(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(MFOC_P10_PROFILE),1/2/0/smo)
        $(error Speed sync requires the B SMO endpoint profile)
    endif
    C_DEFS += -DMFOC_P10_SYNC_SPEED=1
    TARGET_CONFIG_SIGNATURE += -p10_sync_speed
else ifneq ($(MFOC_P10_SYNC_SPEED),0)
    $(error MFOC_P10_SYNC_SPEED must be 0 or 1)
endif
ifeq ($(MFOC_P10_LIMIT),1)
    ifneq ($(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(MFOC_P10_PROFILE)/$(MFOC_P10_SYNC_SPEED)/$(MFOC_RUNTIME_BUS),1/2/0/smo/1/1)
        $(error Limit capture requires B SMO endpoints with speed sync and runtime bus)
    endif
    C_DEFS += -DMFOC_P10_LIMIT=1
    TARGET_CONFIG_SIGNATURE += -p10_limit
else ifneq ($(MFOC_P10_LIMIT),0)
    $(error MFOC_P10_LIMIT must be 0 or 1)
endif
ifneq ($(MFOC_P10_HFI_MPU),0)
    ifneq ($(MFOC_P10_HFI_CAPTURE)/$(MFOC_P10_KCL)/$(MFOC_P10_PHYSICAL),1/0/0)
        $(error HFI amplitude diagnostic requires the B HFI capture profile)
    endif
    ifeq ($(MFOC_P10_HFI_MPU),80)
        C_DEFS += -DMFOC_HFI_VOLTAGE_PU=0.08f
    else ifeq ($(MFOC_P10_HFI_MPU),120)
        C_DEFS += -DMFOC_HFI_VOLTAGE_PU=0.12f
    else ifneq ($(MFOC_P10_HFI_MPU),40)
        $(error MFOC_P10_HFI_MPU must be 40, 80, or 120 for a diagnostic build)
    endif
    C_DEFS += -DMFOC_P10_HFI_MPU=$(MFOC_P10_HFI_MPU)
    TARGET_CONFIG_SIGNATURE += -p10_hfi_mpu_$(MFOC_P10_HFI_MPU)
endif
ifeq ($(MFOC_P10_EXIT),1)
    ifneq ($(MFOC_P10_TIMING)/$(MFOC_P10_PHASE)/$(MFOC_P10_SEGMENTS)/$(MFOC_P10_KCL)/$(MFOC_P10_PHYSICAL),1/2/0/0/0)
        $(error Exit witness requires B endpoints and exclusive TIM2 IRQ)
    endif
    C_DEFS += -DMFOC_P10_EXIT_WITNESS=1
    MFOC_P10_SOURCE += target/stm32g431/mfoc_p10_exit.c
    TARGET_CONFIG_SIGNATURE += -p10_exit
else ifneq ($(MFOC_P10_EXIT),0)
    $(error MFOC_P10_EXIT must be 0 or 1)
endif

# Release B uses the top update and one delayed voltage interval. The
# historical A and immediate-CCR builds remain explicit P10 probe profiles.
G431_PWM_PHASE := 2
ifeq ($(MFOC_P10_TIMING),1)
    G431_PWM_PHASE := $(MFOC_P10_PHASE)
endif
C_DEFS += -DMFOC_G431_PWM_PHASE=$(G431_PWM_PHASE)
TARGET_CONFIG_SIGNATURE += -pwm_phase_$(G431_PWM_PHASE)
ifneq ($(G431_PWM_PHASE),0)
    G431_PWM_PHASE_SOURCE := peripheral/stm32g431/mfoc/mfoc_pwm_phase.c
endif
ifeq ($(G431_PWM_PHASE),2)
    C_DEFS += -DMFOC_PWM_PRELOAD_DELAY=1
endif

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

include mfoc/mfoc.mk
MFOC_TARGET_SOURCES = target/stm32g431/mfoc_motor_irq.c \
    peripheral/stm32g431/mfoc/mfoc_port.c $(G431_PWM_PHASE_SOURCE) \
    $(MFOC_P10_SOURCE)
CLASS_SOURCES = class/mfoc_app.c class/mfoc_debug.c

# 启用 MODUS 默认内置的 perf_counter 移植
MODUS_USE_DEFAULT_PERFC_PORT = 1
