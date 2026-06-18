#include <stdint.h>
#include "io.h"
#include "immap.h"
#include "config.h"

#include "la9310_pci.h"

#define VENDOR_ID 0x2058 // Lime Microsystems
#define PRODUCT_ID 0x001D // LimeSDR Micro
// #define VENDOR_ID 0x1957 // NXP
// #define PRODUCT_ID 0x1C12 // la9310
#define REVISION_ID 0x1 // 8bit
#define DEVICE_CLASS 0x0D1000 // 24bit, Wireless Controller / RF Controller / 0

static inline void out_32_masked(uint32_t *addr, uint32_t value, uint32_t mask)
{
    OUT_32(addr, (IN_32(addr) & ~mask) | value);
}

static inline void reset_ser_des(void)
{
    uint32_t *PLL1RSTCTL = (uint32_t *)(CCSR_BASE_ADDR + 0x01EA0000);
    out_32_masked(PLL1RSTCTL, (1 << 31), (1 << 31));
}

static inline void set_pcie_msi_mmc()
{
#define PCIE_MSI_MCR_MMC_MASK 0x000e
#define PCIE_MSI_MCR_MMC_VALUE 0x0006
#define PCIE_MSI_MCR_OFFSET 0x52
    uint16_t *msi_mcr_reg = (uint16_t *)(PCIE_BASE_ADDR + PCIE_MSI_MCR_OFFSET);

    // Set PCIe MSI Multiple Message Capable to 3'b011
    OUT_16(msi_mcr_reg, ((IN_16(msi_mcr_reg) & ~PCIE_MSI_MCR_MMC_MASK) | PCIE_MSI_MCR_MMC_VALUE));
}

static inline void set_pcie_vid_pid()
{
    OUT_16((uint16_t *)(PCIE_BASE_ADDR + 0x0), VENDOR_ID);
    OUT_16((uint16_t *)(PCIE_BASE_ADDR + 0x2), PRODUCT_ID);
}

static inline void set_pcie_class_rev_id()
{
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + 0x8), (DEVICE_CLASS << 8) | REVISION_ID);
}

static inline uint32_t check_pcie_gen2()
{
    uint32_t *PORSR1 = (uint32_t *)(CCSR_BASE_ADDR + 0x01E00000);
    return !((*PORSR1) & (1 << 12));
}

static inline void set_pcie_link_speed()
{
#define PCIE_LINK_CAP_OFFSET 0x7C
#define PCIE_LINK_CTRL2_OFFSET 0xA0

    uint32_t *pcie_link_cap_reg = (uint32_t *)(PCIE_BASE_ADDR + PCIE_LINK_CAP_OFFSET);
    uint16_t *pcie_link_ctrl2_reg = (uint16_t *)(PCIE_BASE_ADDR + PCIE_LINK_CTRL2_OFFSET);

    // Check for PCIe gen2
    if (check_pcie_gen2())
    {
#define TARGET_LINK_SP_MASK 0x000f
#define TARGET_LINK_SP_GEN2 0x2
#define MAX_LINK_SP_MASK 0x0000000f
#define MAX_LINK_SP_GEN2 0x2
        OUT_32(pcie_link_cap_reg, ((IN_32(pcie_link_cap_reg) & ~MAX_LINK_SP_MASK) | MAX_LINK_SP_GEN2));
        OUT_16(pcie_link_ctrl2_reg, ((IN_16(pcie_link_ctrl2_reg) & ~TARGET_LINK_SP_MASK) | TARGET_LINK_SP_GEN2));
    }

    // set max link width
#define Link_Capabilities_Register PCIE_BASE_ADDR + 0x7C
    OUT_32(Link_Capabilities_Register, ((IN_32(Link_Capabilities_Register) & ~(0x3F << 4)) | (1 << 4)));
}

static inline void setup_pcie_atu()
{
    // ATU 0 : INBOUND : map BAR0
    OUT_32((uint32_t *)PCIE_ATU_VIEWPORT_E, (PCIE_ATU_REGION_INBOUND | PCIE_ATU_REGION_INDEX0));
    OUT_32((uint32_t *)PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR0_TARGET);
    OUT_32((uint32_t *)PCIE_ATU_UPPER_TARGET_E, 0x0);
    OUT_32((uint32_t *)PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM);
    OUT_32((uint32_t *)PCIE_ATU_CR2_E, (PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE | PCIE_ATU_BAR_NUM(0)));

    // ATU 1 : INBOUND : map BAR1
    OUT_32((uint32_t *)PCIE_ATU_VIEWPORT_E, (PCIE_ATU_REGION_INBOUND | PCIE_ATU_REGION_INDEX1));
    OUT_32((uint32_t *)PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR1_TARGET);
    OUT_32((uint32_t *)PCIE_ATU_UPPER_TARGET_E, 0x0);
    OUT_32((uint32_t *)PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM);
    OUT_32((uint32_t *)PCIE_ATU_CR2_E, (PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE | PCIE_ATU_BAR_NUM(1)));

    // ATU 2 : INBOUND : map BAR2
    OUT_32((uint32_t *)PCIE_ATU_VIEWPORT_E, (PCIE_ATU_REGION_INBOUND | PCIE_ATU_REGION_INDEX2));
    OUT_32((uint32_t *)PCIE_ATU_LOWER_TARGET_E, PCIE_ATU_BAR2_TARGET);
    OUT_32((uint32_t *)PCIE_ATU_UPPER_TARGET_E, 0x0);
    OUT_32((uint32_t *)PCIE_ATU_CR1_E, PCIE_ATU_TYPE_MEM);
    OUT_32((uint32_t *)PCIE_ATU_CR2_E, (PCIE_ATU_ENABLE | PCIE_ATU_BAR_MODE_ENABLE | PCIE_ATU_BAR_NUM(2)));
}

static inline void setup_pcie_bars()
{
#define COHERENCY_CONTROL_3_OFF 0x8E8
#define COHERENCY_CONTROL_2_OFF 0x8E4
#define COHERENCY_CONTROL_1_OFF 0x8E0
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + COHERENCY_CONTROL_3_OFF), 0x0);
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + COHERENCY_CONTROL_2_OFF), 0x0);
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + COHERENCY_CONTROL_1_OFF), 0x4000000);

    // reduce BAR0 size to 64MB
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + 0x1010), 0x3FFFFFF); // BAR0_MASK
}

#define PCIE_MISC_CONTROL_1_OFF 0x8BC
#define PCIE_DBI_RO_WR_EN 1
#define PCIE_DBI_RO_WR_DIS 0

void pcie_init(void)
{
    reset_ser_des();

    // Set RO write enable register
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_MISC_CONTROL_1_OFF), PCIE_DBI_RO_WR_EN);

    // Set PCIe device ID
    set_pcie_vid_pid();

    // Set PCIe MSI Multiple Message Capable to 3'b011
    set_pcie_msi_mmc();

    // Set PCIe revision ID
    set_pcie_class_rev_id();

    // Set PCIe Link Speed
    set_pcie_link_speed();

    // #ifdef CONFIG_PCIE_EQU_PROG
    //     // Eualization register configuration as requested by verification team.
    //     OUT_32((uint32_t *)0x434008a8, (uint32_t)0x20);
    // #endif

    // Set PCIe BARs size
    setup_pcie_bars();

    // Set PCIe Address translation window
    setup_pcie_atu();

    // Reset RO write enable register
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_MISC_CONTROL_1_OFF), PCIE_DBI_RO_WR_DIS);

    // Errata A-008822
#define PCIE_ABSERR 0x8D0
#define PCIE_ABSERR_SETTING 0x9401
    OUT_32((uint32_t *)(PCIE_BASE_ADDR + PCIE_ABSERR), PCIE_ABSERR_SETTING);

// Set PCIe configuration ready bit and LTSSM enable bit
#define PEX_PF0_CONFIG_OFFSET 0x14
#define PEX_PF0_CFG_READY_MASK 1
#define PEX_PF0_LTSSM_EN_MASK (1 << 12)
    uint32_t val = IN_32((uint32_t *)(PCIE_PEX_PF_CONTROL_BASE + PEX_PF0_CONFIG_OFFSET));
    val |= (PEX_PF0_CFG_READY_MASK | PEX_PF0_LTSSM_EN_MASK);
    OUT_32((uint32_t *)(PCIE_PEX_PF_CONTROL_BASE + PEX_PF0_CONFIG_OFFSET), val);
}
