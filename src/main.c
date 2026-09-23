/*============================ INCLUDES ======================================*/
#include "global_define.h"
#include <stdio.h>
#include "peripheral.h"
#if defined(MDI_MAIN_STREAM_RESOURCE)
#include "mdi/instance.h"
#else
#include "mdi_hw.h"
#endif
#include "perf_counter.h"
#include "util_debug.h"
#include "debug_transport.h"

#if MODUS_ENABLE
#include "modus.h"
static modus_t s_tModus = { .ptAppFlash = NULL };
#endif

#if MODUS_ENABLE
#include <string.h>
#include "mdebug/mshell.h"  // 引入 mshell_io_t 和 mshell_SetIO

#if USERCONFIG_MSHELL_ON_SERIAL
static unsigned s_uart_read(char *pchBuf, unsigned hwSize)
{
#if defined(MDI_MAIN_STREAM_RESOURCE)
    int32_t nRead = MDI_STREAM_Read(
        MDI_MAIN_STREAM_RESOURCE, (uint8_t *)pchBuf, (uint32_t)hwSize);
#else
    if (HW.ptSerial == NULL) {
        return 0;
    }
    int32_t nRead = MDI_Stream_Read(
        HW.ptSerial, (uint8_t *)pchBuf, (uint32_t)hwSize);
#endif
    return nRead > 0 ? (unsigned)nRead : 0;
}

static void s_uart_write(const char *pchBuf, unsigned hwSize)
{
#if defined(MDI_MAIN_STREAM_RESOURCE)
    (void)MDI_STREAM_Write(
        MDI_MAIN_STREAM_RESOURCE, (const uint8_t *)pchBuf,
        (uint32_t)hwSize);
#else
    if (HW.ptSerial == NULL) {
        return;
    }
    (void)MDI_Stream_Write(
        HW.ptSerial, (const uint8_t *)pchBuf, (uint32_t)hwSize);
#endif
}

static const mshell_io_t s_tUartIO = {
    .pfcnRead  = s_uart_read,
    .pfcnWrite = s_uart_write,
};
#endif

void user_trace_output(const char *str)
{
#if USERCONFIG_MSHELL_ON_SERIAL
#if defined(MDI_MAIN_STREAM_RESOURCE)
    if (str != NULL) {
        (void)MDI_STREAM_Write(
            MDI_MAIN_STREAM_RESOURCE, (const uint8_t *)str, strlen(str));
    }
#else
    if (str && HW.ptSerial) {
        (void)MDI_Stream_Write(
            HW.ptSerial, (const uint8_t *)str, strlen(str));
    }
#endif
#else
    debug_transport_write_string(str);
#endif
}
#endif

int main(void)
{
    peripheral_Init();
    perfc_init(true);

#if MODUS_ENABLE
    #if MSHELL_ENABLE || !defined(__NO_USE_LOG__)
        debug_transport_init();
        MLOGF(I, "Booting MODUS Template...\r\n");
    #endif
    modus_Init(&s_tModus);
    #if USERCONFIG_MSHELL_ON_SERIAL && MSHELL_ENABLE
    mshell_SetIO(&s_tUartIO);
    #endif
#endif

#if MODUS_ENABLE
    while (1) {
        modus_Run();
    }
#endif

    return 0;
}
