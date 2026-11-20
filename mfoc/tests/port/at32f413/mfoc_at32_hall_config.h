#ifndef TEST_MFOC_AT32_HALL_CONFIG_H
#define TEST_MFOC_AT32_HALL_CONFIG_H

/* Synthetic values for software testing only. */
#define MFOC_AT32_HALL_SECTORS { \
    {1U, {UINT32_C(0x00000000)}}, {3U, {UINT32_C(0x2aaaaaab)}}, \
    {2U, {UINT32_C(0x55555555)}}, {6U, {UINT32_C(0x80000000)}}, \
    {4U, {UINT32_C(0xaaaaaaab)}}, {5U, {UINT32_C(0xd5555555)}} }

#endif
