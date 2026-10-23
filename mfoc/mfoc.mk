# Optional library with MDI data/safety ports; target provides MDI bindings.
MFOC_ROOT ?= mfoc
MFOC_MODUS_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../modus)
MFOC_INCLUDES = -I$(MFOC_ROOT) -I$(MFOC_ROOT)/base/include                    \
    -I$(MFOC_ROOT)/core/include                                                \
    -I$(MFOC_ROOT)/motor/include -I$(MFOC_ROOT)/port/include                    \
    -I$(MFOC_ROOT)/position/include                                             \
    -I$(MFOC_MODUS_ROOT)/lib/perf_counter
MFOC_SOURCES = $(MFOC_ROOT)/base/src/mfoc_numeric.c                             \
    $(MFOC_ROOT)/base/src/mfoc_angle.c                                          \
    $(MFOC_ROOT)/base/src/mfoc_trig.c                                           \
    $(MFOC_ROOT)/base/src/mfoc_math.c                                           \
    $(MFOC_ROOT)/base/src/mfoc_transform.c                                      \
    $(MFOC_ROOT)/base/src/mfoc_modulation.c                                     \
    $(MFOC_ROOT)/base/src/mfoc_pi.c                                             \
    $(MFOC_ROOT)/core/src/mfoc_core.c                                           \
    $(MFOC_ROOT)/motor/src/mfoc_motor.c                                         \
    $(MFOC_ROOT)/motor/src/mfoc_openloop.c                                      \
    $(MFOC_ROOT)/motor/src/mfoc_feedback.c                                      \
    $(MFOC_ROOT)/position/src/mfoc_encoder.c                                    \
    $(MFOC_ROOT)/position/src/mfoc_smo.c                                        \
    $(MFOC_ROOT)/motor/src/mfoc_motor_irq.c                                     \
    $(MFOC_ROOT)/port/src/mfoc_port.c
