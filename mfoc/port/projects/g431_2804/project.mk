# 2804 on the STM32G431 reference power board.
ifneq ($(TARGET_CHIP),stm32g431)
    $(error MFOC_PROJECT=g431_2804 requires TARGET_CHIP=stm32g431)
endif

MFOC_MOTOR_CONFIG ?= $(MFOC_PROJECT_DIR)/motor_2804.h
C_DEFS += -DMFOC_PROJECT_CONFIG="\"port/projects/g431_2804/mfoc_project_config.h\""

# The legacy EKF switch selects the same project configuration.
STM32G431_EKF ?= 0
ifeq ($(STM32G431_EKF),1)
    MFOC_G431_FEEDBACK ?= ekf
    ifneq ($(MFOC_G431_FEEDBACK),ekf)
        $(error STM32G431_EKF=1 requires MFOC_G431_FEEDBACK=ekf)
    endif
else ifneq ($(STM32G431_EKF),0)
    $(error STM32G431_EKF must be 0 or 1)
endif
MFOC_G431_FEEDBACK ?= flux

MFOC_G431_BACKEND_flux := FLUX
MFOC_G431_BACKEND_smo := SMO
MFOC_G431_BACKEND_hfi := HFI
MFOC_G431_BACKEND_ekf := EKF
MFOC_G431_BACKEND := $(MFOC_G431_BACKEND_$(MFOC_G431_FEEDBACK))
ifeq ($(MFOC_G431_BACKEND),)
    $(error MFOC_G431_FEEDBACK must be flux, smo, hfi or ekf)
endif
ifeq ($(MFOC_G431_FEEDBACK)/$(MFOC_NUMERIC),ekf/fixed)
    $(error MFOC_G431_FEEDBACK=ekf requires MFOC_NUMERIC=float)
endif

C_DEFS += -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_$(MFOC_G431_BACKEND) \
    -DMFOC_FEEDBACK_ENABLE_$(MFOC_G431_BACKEND)=1
ifneq ($(MFOC_G431_FEEDBACK),flux)
    C_DEFS += -DMFOC_FEEDBACK_ENABLE_FLUX=0
endif
MFOC_PROJECT_CONFIG_SIGNATURE += -feedback_$(MFOC_G431_FEEDBACK)
