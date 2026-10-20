# Optional library; Motor requires target-defined mfoc_motor_port symbols.
MFOC_ROOT ?= mfoc
MFOC_INCLUDES = -I$(MFOC_ROOT)/base/include -I$(MFOC_ROOT)/core/include           \
    -I$(MFOC_ROOT)/motor/include
MFOC_SOURCES = $(MFOC_ROOT)/base/src/mfoc_numeric.c                              \
    $(MFOC_ROOT)/base/src/mfoc_angle.c                                          \
    $(MFOC_ROOT)/base/src/mfoc_trig.c                                           \
    $(MFOC_ROOT)/base/src/mfoc_math.c                                           \
    $(MFOC_ROOT)/base/src/mfoc_transform.c                                      \
    $(MFOC_ROOT)/base/src/mfoc_modulation.c                                      \
    $(MFOC_ROOT)/base/src/mfoc_pi.c                                             \
    $(MFOC_ROOT)/core/src/mfoc_core.c                                          \
    $(MFOC_ROOT)/motor/src/mfoc_motor.c
