.DEFAULT_GOAL := run
MFOC_ROOT ?= ../..
MFOC_MOTOR_CONFIG ?= $(MFOC_ROOT)/port/template/motor_template.h
include $(MFOC_ROOT)/mfoc.mk

ifeq ($(origin CC),default)
    CC = gcc
endif

NUMERIC ?= FLOAT
BACKEND ?= POSITION
POSITION ?= $(if $(filter POSITION,$(BACKEND)),1,0)
SMO ?= $(if $(filter SMO,$(BACKEND)),1,0)
HFI ?= $(if $(filter HFI,$(BACKEND)),1,0)
FLUX ?= $(if $(filter FLUX,$(BACKEND)),1,0)
EKF ?= $(if $(filter EKF,$(BACKEND)),1,0)
IDENTIFY ?= 0
TRIG ?= LUT
MODEL_UPDATE ?= 0

ifeq ($(filter NONE POSITION SMO HFI FLUX EKF,$(BACKEND)),)
    $(error Unknown BACKEND: $(BACKEND))
endif
ifeq ($(filter LUT LIBM EXTERNAL,$(TRIG)),)
    $(error Unknown TRIG: $(TRIG))
endif

CONFIG_FLAGS = -DMFOC_NUMERIC_$(NUMERIC)=1 \
    -DMFOC_FEEDBACK_BACKEND=MFOC_FEEDBACK_$(BACKEND) \
    -DMFOC_FEEDBACK_ENABLE_POSITION=$(POSITION) \
    -DMFOC_FEEDBACK_ENABLE_SMO=$(SMO) -DMFOC_FEEDBACK_ENABLE_HFI=$(HFI) \
    -DMFOC_FEEDBACK_ENABLE_FLUX=$(FLUX) -DMFOC_FEEDBACK_ENABLE_EKF=$(EKF) \
    -DMFOC_IDENTIFICATION_ENABLE=$(IDENTIFY) \
    -DMFOC_MOTOR_MODEL_UPDATE_ENABLE=$(MODEL_UPDATE) \
    -DMFOC_TRIG_BACKEND=MFOC_TRIG_BACKEND_$(TRIG) $(MFOC_MOTOR_CONFIG_FLAGS)
CFLAGS += -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off
LDLIBS += -lm

ifeq ($(OS),Windows_NT)
    SHELL = cmd.exe
    EXEEXT = .exe
endif

FEATURES = $(POSITION)$(SMO)$(HFI)$(FLUX)$(EKF)
PROFILE = $(NUMERIC)-$(BACKEND)-$(FEATURES)-id$(IDENTIFY)-$(TRIG)-model$(MODEL_UPDATE)
PROGRAM = build/replay-$(PROFILE)$(EXEEXT)
SOURCES = consumer.c $(MFOC_SOURCES)
DEPS = build/replay-$(PROFILE).d
CONFIG_STAMP = build/replay-$(PROFILE).motor-config
MOTOR_CONFIG = $(or $(MFOC_MOTOR_CONFIG_PATH),default)

ifeq ($(OS),Windows_NT)
    CONFIG_STAMP_WIN = $(subst /,\,$(CONFIG_STAMP))
    PREV_CONFIG := $(subst ",,$(shell if exist "$(CONFIG_STAMP_WIN)" type "$(CONFIG_STAMP_WIN)"))
else
    PREV_CONFIG := $(shell cat "$(CONFIG_STAMP)" 2>/dev/null)
endif

ifneq ($(PREV_CONFIG),$(MOTOR_CONFIG))
.PHONY: $(CONFIG_STAMP)
endif

.PHONY: all run
all: $(PROGRAM)

run: $(PROGRAM)
	./$(PROGRAM)

-include $(DEPS)

$(PROGRAM): $(SOURCES) $(CONFIG_STAMP) Makefile $(MFOC_ROOT)/mfoc.mk | build
	$(CC) $(CPPFLAGS) $(CONFIG_FLAGS) $(CFLAGS) $(MFOC_INCLUDES) \
	    $(SOURCES) $(LDLIBS) -o $@
	$(CC) $(CPPFLAGS) $(CONFIG_FLAGS) $(CFLAGS) $(MFOC_INCLUDES) \
	    -MM -MP -MT "$@" $(SOURCES) > "$(DEPS)"

$(CONFIG_STAMP): | build
ifeq ($(OS),Windows_NT)
	@echo "$(MOTOR_CONFIG)">"$(CONFIG_STAMP_WIN)"
else
	@printf '%s\n' "$(MOTOR_CONFIG)" > "$(CONFIG_STAMP)"
endif

build:
	mkdir build
