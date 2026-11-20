.DEFAULT_GOAL := run
MFOC_ROOT ?= ../..
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
    -DMFOC_TRIG_BACKEND=MFOC_TRIG_BACKEND_$(TRIG)
CFLAGS += -std=c11 -O2 -Wall -Wextra -Werror -ffp-contract=off
LDLIBS += -lm

ifeq ($(OS),Windows_NT)
    EXEEXT = .exe
endif

FEATURES = $(POSITION)$(SMO)$(HFI)$(FLUX)$(EKF)
PROFILE = $(NUMERIC)-$(BACKEND)-$(FEATURES)-id$(IDENTIFY)-$(TRIG)-model$(MODEL_UPDATE)
PROGRAM = build/replay-$(PROFILE)$(EXEEXT)
SOURCES = consumer.c $(MFOC_SOURCES)
HEADERS = $(wildcard $(addsuffix /*.h,$(MFOC_INCLUDE_DIRS))) \
    $(wildcard $(addsuffix *.h,$(sort $(dir $(MFOC_SOURCES))))) \
    $(wildcard $(addsuffix *.inc,$(sort $(dir $(MFOC_SOURCES)))))

.PHONY: all run
all: $(PROGRAM)

run: $(PROGRAM)
	./$(PROGRAM)

$(PROGRAM): $(SOURCES) $(HEADERS) Makefile $(MFOC_ROOT)/mfoc.mk | build
	$(CC) $(CPPFLAGS) $(CONFIG_FLAGS) $(CFLAGS) $(MFOC_INCLUDES) \
	    $(SOURCES) $(LDLIBS) -o $@

build:
	mkdir build
