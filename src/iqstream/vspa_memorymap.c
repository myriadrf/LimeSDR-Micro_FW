
#include "vspa_memorymap.h"

#include "limesdr_micro/iqplayer_commands.h"
#include "drivers/avi/la9310_avi_ds.h"

#include "immap.h"
#include "io.h"

#include "log.h"

#include <stddef.h>

void *vspa_memorymap_find(e_vspa_feature f)
{
    const uint32_t BASE_VSPA0 = 0x20400000; //!< VSPA-0 AXI slave interface.

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
