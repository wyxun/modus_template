/**
 * @file board_template.h
 * @brief Board settings template and defaults for standalone builds.
 * @author Codex
 * @date 2026-11-21
 *
 * Supply these settings in the target build or in a board header included
 * before mfoc_config.h in every translation unit.
 */
#ifndef MFOC_BOARD_TEMPLATE_H
#define MFOC_BOARD_TEMPLATE_H

/* PWM/ADC control interrupt rate shared by the board and the library. */
#ifndef MFOC_MOTOR_RUN_FREQ_HZ
#define MFOC_MOTOR_RUN_FREQ_HZ 20000U
#endif

/* Frequency of the tick source that perfc actually reads. */
#ifndef MFOC_PERFC_HZ
#define MFOC_PERFC_HZ 170000000U
#endif

/* Maximum voltage-vector magnitude relative to Vdc, in (0, 1/sqrt(3)]. */
#ifndef MFOC_CORE_M_MAX
#define MFOC_CORE_M_MAX 0.55f
#endif

#endif
