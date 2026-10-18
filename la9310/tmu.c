#include "tmu.h"

#include "io.h"

void tmuInit(TmuRegs_t *pTmuHandle)
{
    int idx;
    // TmuRegs_t *pTmuHandle = ( TmuRegs_t * ) TMU_BASE_ADDR;
    out_le32(&pTmuHandle->tmr, TMU_TMR_DISABLE);

    out_le32(&pTmuHandle->ttrcr[0], TMU_TTRCR0_INIT);
    for (idx = 0; idx < TMU_TTRCR0_POINT; idx++)
    {
        out_le32(&pTmuHandle->ttcfgr, (TMU_TTCFGR_INIT0 + (idx * TMU_TTCFGR_DIFF)));
        if (idx % 2 == 0)
        {
            out_le32(&pTmuHandle->tscfgr,
                (TMU_TSCFGR_INIT0 + (((idx / 2) * (TMU_TSCFGR_DIFF0)) + ((idx / 2) * (TMU_TSCFGR_DIFF0 + 1)))));
        }
        else
        {
            out_le32(&pTmuHandle->tscfgr,
                (TMU_TSCFGR_INIT0 + ((((idx + 1) / 2) * (TMU_TSCFGR_DIFF0)) + ((idx / 2) * (TMU_TSCFGR_DIFF0 + 1)))));
        }
    }

    out_le32(&pTmuHandle->ttrcr[1], TMU_TTRCR1_INIT);
    for (idx = 0; idx < TMU_TTRCR1_POINT; idx++)
    {
        out_le32(&pTmuHandle->ttcfgr, (TMU_TTCFGR_INIT1 + (idx * TMU_TTCFGR_DIFF)));
        out_le32(&pTmuHandle->tscfgr, (TMU_TSCFGR_INIT1 + (idx * TMU_TSCFGR_DIFF1)));
    }

    for (idx = 0; idx < TMU_TTRCR2_POINT; idx++)
    {
        out_le32(&pTmuHandle->ttcfgr, (TMU_TTCFGR_INIT2 + (idx * TMU_TTCFGR_DIFF)));
        out_le32(&pTmuHandle->tscfgr, (TMU_TSCFGR_INIT2 + (idx * TMU_TSCFGR_DIFF2)));
    }

    out_le32(&pTmuHandle->ttrcr[3], TMU_TTRCR3_INIT);
    for (idx = 0; idx < TMU_TTRCR3_POINT; idx++)
    {
        out_le32(&pTmuHandle->ttcfgr, (TMU_TTCFGR_INIT3 + (idx * TMU_TTCFGR_DIFF)));
        if (idx == (TMU_TTRCR3_POINT - 1))
        {
            out_le32(&pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 + (idx * TMU_TSCFGR_DIFF3) + 1));
        }
        else
        {
            out_le32(&pTmuHandle->tscfgr, (TMU_TSCFGR_INIT3 + (idx * TMU_TSCFGR_DIFF3)));
        }
    }
    out_le32(&pTmuHandle->teumr[0], TMU_TEUMR0_ENABLE);
    out_le32(&pTmuHandle->tdemar, TMU_TDEMAR_ENABLE);
    out_le32(&pTmuHandle->tmtmir, TMU_TMTMIR_ENABLE);
    out_le32(&pTmuHandle->monitoringSite[0].tmsar, TMU_TMSAR0_INIT);
    out_le32(&pTmuHandle->monitoringSite[1].tmsar, TMU_TMSAR1_INIT);
    out_le32(&pTmuHandle->monitoringSite[2].tmsar, TMU_TMSAR2_INIT);
    out_le32(&pTmuHandle->tmrtrcr, TMU_TMRTRCTR_INIT);
    out_le32(&pTmuHandle->tmftrcr, TMU_TMFTRCTR_INIT);
    out_le32(&pTmuHandle->tsr, TMU_TSR_INIT);

    for (int i = 0; i < 10000; i++)
        ;
    out_le32(&pTmuHandle->tmr, TMU_TMR_ENABLE);
}