# MFOC library sources; the target supplies the Port and interrupt entry.
MFOC_ROOT ?= mfoc
MFOC_MODUS_ROOT ?= $(abspath $(dir $(lastword $(MAKEFILE_LIST)))/../modus)
MFOC_INCLUDES = -I$(MFOC_ROOT) -I$(MFOC_ROOT)/math/include                     \
    -I$(MFOC_ROOT)/control/include                                             \
    -I$(MFOC_ROOT)/core/include                                                \
    -I$(MFOC_ROOT)/motor/include -I$(MFOC_ROOT)/port/include                    \
    -I$(MFOC_ROOT)/position/include                                             \
    -I$(MFOC_ROOT)/identification/include                                       \
    -I$(MFOC_MODUS_ROOT)/lib/perf_counter
MFOC_SOURCES = $(MFOC_ROOT)/math/src/mfoc_numeric.c                              \
    $(MFOC_ROOT)/math/src/mfoc_angle.c                                          \
    $(MFOC_ROOT)/math/src/mfoc_trig.c                                           \
    $(MFOC_ROOT)/math/src/mfoc_math.c                                           \
    $(MFOC_ROOT)/math/src/mfoc_transform.c                                      \
    $(MFOC_ROOT)/control/src/mfoc_modulation.c                                  \
    $(MFOC_ROOT)/control/src/mfoc_pi.c                                          \
    $(MFOC_ROOT)/core/src/mfoc_core.c                                           \
    $(MFOC_ROOT)/motor/src/mfoc_motor.c                                         \
    $(MFOC_ROOT)/motor/src/mfoc_sensorless_start.c                              \
    $(MFOC_ROOT)/motor/src/mfoc_openloop.c                                      \
    $(MFOC_ROOT)/motor/src/mfoc_feedback.c                                      \
    $(MFOC_ROOT)/identification/src/mfoc_identification.c                       \
    $(MFOC_ROOT)/identification/src/mfoc_identification_rs.c                    \
    $(MFOC_ROOT)/identification/src/mfoc_identification_inductance.c            \
    $(MFOC_ROOT)/identification/src/mfoc_identification_flux.c                  \
    $(MFOC_ROOT)/identification/src/mfoc_identification_inertia.c               \
    $(MFOC_ROOT)/position/src/mfoc_encoder.c                                    \
    $(MFOC_ROOT)/position/src/mfoc_hall.c                                       \
    $(MFOC_ROOT)/position/src/mfoc_smo.c                                        \
    $(MFOC_ROOT)/position/src/mfoc_flux.c                                       \
    $(MFOC_ROOT)/position/src/mfoc_ekf.c                                        \
    $(MFOC_ROOT)/position/src/mfoc_hfi.c
