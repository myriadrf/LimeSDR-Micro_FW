
#include "la9310_avi_ds.h"

#include "immap.h"
#include "io.h"

#include "log.h"

#include <stddef.h>

typedef struct phytimer_ctrl_m4 {
    uint32_t phytime;
    struct {
        uint32_t cmd : 8; // M4_PHYTIME_CMD_*
        uint32_t timer_id : 8;
        uint32_t trig_value : 2;
    };
} m4_phytimer_t;

extern volatile m4_phytimer_t *vspa_timer_request;

enum M4_PHYTIME_CMD {
    M4_PHYTIME_CMD_IMMEDIATE,
    M4_PHYTIME_CMD_SCHEDULE,
    M4_PHYTIME_CMD_TDD_START,
    M4_PHYTIME_CMD_TDD_END,
};

typedef enum {
    VSPA_MMAP_NONE = 0,
    VSPA_MMAP_L1_TRACE,
    VSPA_MMAP_ADC0,
    VSPA_MMAP_ADC1,
    VSPA_MMAP_ADC2,
    VSPA_MMAP_ADC3,
    VSPA_MMAP_RXDMA_LANE0,
    VSPA_MMAP_RXDMA_LANE1,
    VSPA_MMAP_RXDMA_LANE2,
    VSPA_MMAP_RXDMA_LANE3,
    VSPA_MMAP_DAC,
    VSPA_MMAP_TXDMA_LANE0,
    VSPA_MMAP_STATS,
    VSPA_MMAP_STATS2,
    VSPA_MMAP_TX_TONE_LANE0,
    VSPA_MMAP_VSPA_CORE_STATS,
    VSPA_MMAP_TX_TDD_CONFIG,
    VSPA_MMAP_RXDDR_OUTPUT_LANE0,
    VSPA_MMAP_RXDDR_OUTPUT_LANE1,
    VSPA_MMAP_RXDDR_OUTPUT_LANE2,
    VSPA_MMAP_RXDDR_OUTPUT_LANE3,
    VSPA_MMAP_FW_INFO,
    VSPA_MMAP_PHYTIME_SCHED,
} e_vspa_feature;

typedef struct {
    uint32_t feature;
    uint32_t address;
} vspa_feature_t;

void *vspa_memorymap_find(e_vspa_feature f)
{
    const uint32_t BASE_VSPA0 = 0x20400000; //!< VSPA0 AXI slave interface.

    const vspa_feature_t *row = (const vspa_feature_t *)BASE_VSPA0;
    for (int i = 0; i < 32; ++i)
    {
        if (row[i].feature == VSPA_MMAP_NONE)
            return NULL; // end of table

        if (row[i].feature == f)
            return (void *)(BASE_VSPA0 + (row[i].address << 1)); // row address is in VSPA halfwords, convert to bytes
    }
    return NULL;
}
