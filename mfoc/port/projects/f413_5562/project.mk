# 5562 on AT-MOTOR-EVB V1.1, at 24 V with Hall feedback.
ifneq ($(TARGET_CHIP),at32f413)
    $(error MFOC_PROJECT=f413_5562 requires TARGET_CHIP=at32f413)
endif

AT32F413_MFOC_HALL ?= 1
AT32F413_BUS_VOLTAGE ?= 24
AT32F413_HALL_CONFIG_DIR ?= $(MFOC_PROJECT_DIR)
MFOC_MOTOR_CONFIG ?= $(MFOC_PROJECT_DIR)/motor_5562.h
ifneq ($(AT32F413_MFOC_HALL)/$(AT32F413_BUS_VOLTAGE),1/24)
    $(error MFOC_PROJECT=f413_5562 requires Hall and 24 V)
endif

# Preserve the reference PI gains and the 2 A command cap on a 5 A base.
C_DEFS += -DMFOC_APP_5562_HALL=1 \
    -DMFOC_MOTOR_BASE_VOLTAGE_MILLIVOLT=AT32F413_BUS_NOMINAL_MV \
    -DMFOC_APP_PI_REFERENCE_MILLIAMP=3500U \
    -DMFOC_APP_CURRENT_LIMIT_PU=0.4f -DMFOC_APP_SPEED_CURRENT_LIMIT_PU=0.4f

# Opt-in bench firmware; normal Hall control does not compile identification.
ifeq ($(MFOC_RS_DIAGNOSTIC),1)
    MFOC_PROJECT_CONFIG_SIGNATURE += -rs_diagnostic
    C_DEFS += -DMFOC_IDENTIFICATION_ENABLE=1 -DMFOC_APP_RS_DIAGNOSTIC=1
    # This motor reaches the experiment current cap at the library defaults.
    C_DEFS += -DMFOC_IDENTIFICATION_RS_MODULATION_LOW=0.01f \
        -DMFOC_IDENTIFICATION_RS_MODULATION_HIGH=0.025f
endif
