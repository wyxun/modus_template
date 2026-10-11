/*******************************************************************************
 * @file    foc_cogging.h
 * @brief   External-table cogging torque compensation
 ******************************************************************************/

#ifndef FOC_COGGING_H
#define FOC_COGGING_H

#include <stdint.h>

#include "foc_angle.h"

typedef struct {
    const foc_scalar_t *pqTable;    /**< 齿槽补偿值查找表 */
    uint16_t hwCount;               /**< 表长度 */
} foc_cogging_t;

/**
 * @brief  初始化齿槽转矩补偿表
 * @param  ptCogging  补偿实例指针
 * @param  pqTable    补偿值查找表
 * @param  hwCount    表长度
 * @return            FOC_RESULT_OK 或错误码
 */
foc_result_t foc_cogging_Init(foc_cogging_t *ptCogging,
                              const foc_scalar_t *pqTable,
                              uint16_t hwCount);
/**
 * @brief  根据机械角度查询齿槽补偿值
 * @param  ptCogging         补偿实例指针
 * @param  tMechanicalAngle  机械角度
 * @param  pqCompensation    输出补偿值
 * @return                   FOC_RESULT_OK 或错误码
 */
foc_result_t foc_cogging_Get(const foc_cogging_t *ptCogging,
                             foc_angle_t tMechanicalAngle,
                             foc_scalar_t *pqCompensation);

#endif /* FOC_COGGING_H */
