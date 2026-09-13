#ifndef __USERCONFIG_H__
#define __USERCONFIG_H__

/*============================ INCLUDES ======================================*/
#include "modus.h"

/*============================ MACROS ========================================*/

/* Debug output: 1 = enable MLOG/RTT output, 0 = disable */
#define USERCONFIG_DEBUG_ENABLE 1

/* Route mshell through serial MDI (UART) instead of default RTT.
 * Enable for chips that don't have a debug probe connected (e.g. CH592). */
#define USERCONFIG_MSHELL_ON_SERIAL 0

/* 最大注册命令数（含内置命令），覆盖 mshell.h 默认的 16 */
#define MSHELL_MAX_CMDS             32

/* MODUS waveform: 1 = enable mwaveform for real-time plotting */
#ifndef MWAVEFORM_ENABLE
#   if defined(AT32F421F8P7)
#       define MWAVEFORM_ENABLE 0
#   else
#       define MWAVEFORM_ENABLE 1
#   endif
#endif

#if MWAVEFORM_ENABLE
/* 10 kHz FOC waveform needs headroom: small buffers wrap in tens of ms and
 * make the OpenOCD RTT read race much more likely (see mwaveform.c wedge
 * recovery). */
#   ifndef MWAVEFORM_RTT_BUFFER_SIZE
#       define MWAVEFORM_RTT_BUFFER_SIZE   4096
#   endif
#   define MWAVEFORM_FIFO_DEPTH        32
#   define MWAVEFORM_MAX_CHANNELS      10
#   define MWAVEFORM_BATCH_ENABLE      1
#   define MWAVEFORM_BATCH_SIZE        64
#   define MWAVEFORM_BATCH_DEPTH       128
#   define MWAVEFORM_BATCH_FLUSH_MS    10
#   define MWAVEFORM_SNAPSHOT_ENABLE   0
#   define MWAVEFORM_SNAPSHOT_DEPTH    16
#endif

/*============================ TYPES =========================================*/

/* MODUS object IDs — extend as new class modules are added */
#define TEMPLATE_CLASS   ((MODUS_ID_MOCK<<8)+1)
#define FOC_APP          ((MODUS_ID_MOCK<<8)+2)


/*============================ MACROFIED FUNCTIONS ===========================*/
/*============================ GLOBAL VARIABLES ==============================*/
/*============================ PROTOTYPES ====================================*/

#endif /* __USERCONFIG_H__ */
