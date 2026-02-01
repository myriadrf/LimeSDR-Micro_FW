#ifndef LA9310_TMU_H
#define LA9310_TMU_H

#define SITES_MAX       16
#define TMU_TTRCR0_INIT 0x000B0000
#define TMU_TTRCR1_INIT 0x000A0026
#define TMU_TTRCR2_INIT 0x00080048
#define TMU_TTRCR3_INIT 0x00070061
#define TMU_TTRCR0_POINT	12
#define TMU_TTRCR1_POINT	11
#define TMU_TTRCR2_POINT	11
#define TMU_TTRCR3_POINT	8

#define TMU_TTCFGR_INIT0	0x00000000
#define TMU_TSCFGR_INIT0	0x00000025
#define TMU_TSCFGR_DIFF0	0x00000006
#define TMU_TTCFGR_INIT1	0x00010000
#define TMU_TSCFGR_INIT1	0x0000001C
#define TMU_TSCFGR_DIFF1	0x00000008
#define TMU_TTCFGR_INIT2	0x00010000
#define TMU_TSCFGR_INIT2	0x0000001C
#define TMU_TSCFGR_DIFF2	0x00000008
#define TMU_TTCFGR_INIT3	0x00030000
#define TMU_TSCFGR_INIT3	0x0000000E
#define TMU_TSCFGR_DIFF3	0x0000000C
#define TMU_TTCFGR_DIFF		0x00000001

#define TMU_TEUMR0_ENABLE       0x51009C00
#define TMU_TDEMAR_ENABLE       0x0800FFFE
#define TMU_TMTMIR_ENABLE       0x0000000F
#define TMU_TMRTRCTR_INIT	0x0000000A
#define TMU_TMFTRCTR_INIT	0x0000000A
#define TMU_TMSAR0_INIT		0x0000000E
#define TMU_TMSAR1_INIT		0x0000000E
#define TMU_TMSAR2_INIT		0x0000000E

#define TMU_TMR_DISABLE 	0x0
#define TMU_TSR_INIT            0xF0000000
#define TMU_TMR_ENABLE          0x8000C000

typedef struct tmuSiteRegs {
	uint32_t tritsr; /*  Immediate Temperature Site Register */
	uint32_t tratsr; /*  Average Temperature Site Register */
	uint8_t res0[0x8];
} TmuSiteRegs_t;

typedef struct tmuCalibrationSiteRegs {
	uint32_t tscr;  /*  Immediate Temperature Site Register */
	uint32_t tmsar; /*  Average Temperature Site Register */
	uint8_t res0[0x8];
} TmuCalibrationSiteRegs_t;

typedef struct tmuRegsMap {
	uint32_t tmr;           /*  Mode Register */
	uint32_t tsr;           /*  Status Register */
	uint32_t tmtmir;                /*  Temperature measurement interval Register */
	uint8_t res0[0x14];
	uint32_t tier;          /*  Interrupt Enable Register */
	uint32_t tidr;          /*  Interrupt Detect Register */
	uint8_t res1[0x8];
	uint32_t tiiscr;                /*  Interrupt Immediate Site Capture Register */
	uint32_t tiascr;                /*  Interrupt Average Site Capture Register */
	uint32_t ticscr;                /*  Interrupt Critical Site Capture Register */
	uint8_t res2[0x4];
	uint32_t tmhtcrh;               /*  High Temperature Capture Register */
	uint32_t tmhtcrl;               /*  Low Temperature Capture Register */
	uint32_t tmrtrcr;               /*  Low Temperature Capture Register */
	uint32_t tmftrcr;               /*  Low Temperature Capture Register */
	uint32_t tmhtitr;               /*  High Temperature Immediate Threshold Reg */
	uint32_t tmhtatr;               /*  High Temperature Average Threshold Reg */
	uint32_t tmhtactr;              /*  High Temperature Average Crit Threshold Reg */
	uint8_t res3[0x4];
	uint32_t tmltitr;               /*  Low Temperature Immediate Threshold Reg */
	uint32_t tmltatr;               /*  Low Temperature Average Threshold Reg */
	uint32_t tmltactr;              /*  Low Temperature Average Critical Threshold Reg */
	uint8_t res4[0x4];
	uint32_t tmrtrctr;              /*  TMU monitor rising temperature rate critical threshold register */
	uint32_t tmftrctr;              /*  TMU monitor falling temperature rate critical threshold register */
	uint8_t res5[0x8];
	uint32_t ttcfgr;                /*  Temperature configuration register */
	uint32_t tscfgr;                /*  Sensor configuration register */
	uint8_t res6[0x8];
	uint32_t ttcr;          /*  Temperature calibration register */
	uint8_t res7[0x6C];
	TmuSiteRegs_t site[SITES_MAX];   /*  Temp site register */
	uint8_t res8[0x100];
	TmuCalibrationSiteRegs_t monitoringSite[SITES_MAX]; /*  Monitoring Site reg */
	uint8_t res9[0x7F8];
	uint32_t ipbrr0;                 /*  IP Block Revision Register 0 */
	uint32_t ipbrr1;                /*  IP Block Revision Register 1 */
	uint8_t res10[0x300];
	uint32_t teumr[0x3];    /*  Engineering use mode register */
	uint32_t tdemar;                /*  Dynamic element match averaging register */
	uint32_t ttrcr[SITES_MAX];      /*  Temperature Range Control Register */
} TmuRegs_t;

#endif
