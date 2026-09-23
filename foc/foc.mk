# =============================================================================
# Modus FOC — source and include configuration (minimal single-motor build)
#
# Include this from target/<chip>/target.mk for chips that support FOC.
# Chips that do not support FOC skip the include; FOC_SOURCES stays empty
# (guarded by "FOC_SOURCES ?=" in the top-level makefile) and the FOC
# include paths default to nothing.
#
# 编译数学核心 + PID + SVPWM + 编码器 + App。
# SMO 的唯一功能开关在 foc_config.h；未启用时链接器回收未引用代码段。
# =============================================================================

FOC_INCLUDES = -Ifoc \
               -Ifoc/math -Ifoc/hal -Ifoc/motor \
               -Ifoc/middleware -Ifoc/control \
               -Ifoc/modulation -Ifoc/observer -Ifoc/identify \
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
              foc/identify/identify.c \
              foc/identify/identify_resistance.c \
              foc/identify/identify_inductance.c \
              foc/motor/motor_position.c \
              foc/motor/motor.c \
              foc/app/foc_debug.c \
              foc/app/foc_app.c
