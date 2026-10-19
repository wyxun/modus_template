# Optional pure computation library. Include explicitly; no target is changed.
MFOC_ROOT ?= mfoc
MFOC_INCLUDES = -I$(MFOC_ROOT)/include
MFOC_SOURCES = $(MFOC_ROOT)/src/mfoc_numeric.c                                  \
    $(MFOC_ROOT)/src/mfoc_angle.c                                               \
    $(MFOC_ROOT)/src/mfoc_trig.c                                                \
    $(MFOC_ROOT)/src/mfoc_math.c                                                \
    $(MFOC_ROOT)/src/mfoc_transform.c                                           \
    $(MFOC_ROOT)/src/mfoc_modulation.c
