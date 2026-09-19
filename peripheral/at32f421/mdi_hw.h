/**
 * @file mdi_hw.h
 * @brief 全局外设资源统一定义 (MDI 硬件池头文件)
 *
 * 为应用层提供统一硬件结构定义，避免应用层暴露芯片私有头文件。
 */

#ifndef __MDI_HW_H__
#define __MDI_HW_H__

#include "mdi/legacy/mdi.h"
#include "mdi/legacy/mdi_static.h"

#define MDI_STREAM_WRITE_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_Write

#define MDI_STREAM_READ_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_Read

#define MDI_STREAM_IS_BUSY_ASSOCIATIONS \
    mdi_stream_t *: mdi_stream_IsBusy

/*============================================================================
 * 项目硬件资源池定义
 *
 * 结构体内声明项目所有的使用到的外设抽象对象
 *===========================================================================*/

typedef struct {
    /* ---------- GPIO ---------- */
    mdi_gpio_t   *ptLedStatus;      /**< 状态指示 LED */

    /* ---------- Stream ---------- */
    mdi_stream_t *ptSerial;       /**< RS232 串口 (PA9/10) */
    
} mdi_hardware_t;

/**
 * @brief 全局统一的硬件资源实例
 * 
 * 真正的实例化发生在外设适配层（如 peripheral/AT32/port_mdi.c 中）
 */
extern const mdi_hardware_t HW;

#endif /* __MDI_HW_H__ */
