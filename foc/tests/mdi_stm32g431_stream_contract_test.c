#include "mdi_hw.h"

int mdi_stm32g431_stream_contract_test(void)
{
    uint8_t chBuffer[4] = {0};
    int32_t nRead = MDI_Stream_Read(HW.ptSerial, chBuffer, sizeof(chBuffer));
    int32_t nWrite = MDI_Stream_Write(HW.ptSerial, chBuffer, sizeof(chBuffer));
    int32_t nBusy = MDI_Stream_IsBusy(HW.ptSerial);

    return (nRead + nWrite + nBusy) == INT32_MIN;
}
