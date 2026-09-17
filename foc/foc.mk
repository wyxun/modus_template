# =============================================================================
# Modus FOC — source and include configuration (minimal single-motor build)
#
# Include this from target/<chip>/target.mk for chips that support FOC.
# Chips that do not support FOC skip the include; FOC_SOURCES stays empty
# (guarded by "FOC_SOURCES ?=" in the top-level makefile) and the FOC
# include paths default to nothing.
#
# 极简构建：编译数学核心 + PID + SVPWM + 编码器 + SMO Observer + App。
# 非首期 Observer 算法不进入构建；本期 SMO 仅在 Encoder 闭环下 Shadow。
# =============================================================================

FOC_INCLUDES = -Ifoc \
               -Ifoc/math -Ifoc/hal -Ifoc/motor \
               -Ifoc/middleware -Ifoc/control \
               -Ifoc/modulation -Ifoc/observer \
               -Ifoc/app

FOC_SOURCES = foc/math/foc_numeric.c \
              foc/math/foc_angle.c \
              foc/math/foc_trig_lut.c \
              foc/math/foc_math.c \
              foc/middleware/foc_core.c \
              foc/control/foc_pid.c \
              foc/modulation/foc_modulation.c \
              foc/observer/foc_encoder.c \
              foc/observer/foc_observer.c \
              foc/observer/foc_smo.c \
              foc/motor/motor.c \
              foc/app/foc_app.c
