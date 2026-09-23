/****************************************************************************
 * @file    foc_port_config.h
 * @brief   Board-level ADC, DC-bus and sampling configuration defaults.
 *
 * Targets override these values from target.mk after validating the board
 * sensing circuit and timing. These values do not describe a motor profile.
 ****************************************************************************/

#ifndef FOC_PORT_CONFIG_H
#define FOC_PORT_CONFIG_H

/* Capabilities supplied by the selected target port. */
#ifndef FOC_PORT_HAS_POSITION
#define FOC_PORT_HAS_POSITION                 0
#endif

/* Current ADC scaling and polarity. */
#ifndef FOC_CURRENT_BASE_MILLIAMP
#define FOC_CURRENT_BASE_MILLIAMP             3500U
#endif

#ifndef FOC_CURRENT_COUNTS_PER_BASE
#define FOC_CURRENT_COUNTS_PER_BASE           1390U
#endif

#ifndef FOC_CURRENT_SAMPLE_INVERTED
#define FOC_CURRENT_SAMPLE_INVERTED           1
#endif

/* DC-bus source and ADC conversion. */
#define FOC_DCBUS_SOURCE_NONE                 (0U)
#define FOC_DCBUS_SOURCE_NOMINAL              (1U)
#define FOC_DCBUS_SOURCE_ADC                  (2U)

#ifndef FOC_DCBUS_SOURCE
#define FOC_DCBUS_SOURCE                      FOC_DCBUS_SOURCE_ADC
#endif

#ifndef FOC_DCBUS_NOMINAL_MILLIVOLT
#define FOC_DCBUS_NOMINAL_MILLIVOLT           (0U)
#endif

#ifndef FOC_DCBUS_MV_PER_COUNT_NUM
#define FOC_DCBUS_MV_PER_COUNT_NUM            (1U)
#endif

#ifndef FOC_DCBUS_MV_PER_COUNT_DEN
#define FOC_DCBUS_MV_PER_COUNT_DEN            (1U)
#endif

#ifndef FOC_DCBUS_OFFSET_MILLIVOLT
#define FOC_DCBUS_OFFSET_MILLIVOLT            (0)
#endif

/* DC-bus sample timing and accepted range. */
#ifndef FOC_DCBUS_SAMPLE_DELAY_CYCLES
#define FOC_DCBUS_SAMPLE_DELAY_CYCLES         (0U)
#endif

#ifndef FOC_DCBUS_MAX_AGE_CYCLES
#define FOC_DCBUS_MAX_AGE_CYCLES              (0U)
#endif

#ifndef FOC_DCBUS_MAX_MILLIVOLT
#define FOC_DCBUS_MAX_MILLIVOLT               (60000U)
#endif

#endif /* FOC_PORT_CONFIG_H */
