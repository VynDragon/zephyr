/*
 * Copyright (c) 2019 Gerson Fernando Budke
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Atmel SAM4L MCU series initialization code
 *
 * This module provides routines to initialize and support board-level hardware
 * for the Atmel SAM4L series processor.
 */

#include <device.h>
#include <init.h>
#include <soc.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

/** Watchdog control register first write keys */
#define WDT_FIRST_KEY     0x55ul
/** Watchdog control register second write keys */
#define WDT_SECOND_KEY    0xAAul
/**
 * \brief Calculate \f$ \left\lceil \frac{a}{b} \right\rceil \f$ using
 * integer arithmetic.
 *
 * \param a An integer
 * \param b Another integer
 *
 * \return (\a a / \a b) rounded up to the nearest integer.
 */
#define div_ceil(a, b)	(((a) + (b) - 1) / (b))

/**
 * @brief Sets the WatchDog Timer Control register to the \a ctrl value thanks
 *        to the WatchDog Timer key.
 *
 * @param ctrl  Value to set the WatchDog Timer Control register to.
 */
static ALWAYS_INLINE void wdt_set_ctrl(u32_t ctrl)
{
	volatile u32_t dly;

	/** Calculate delay for internal synchronization
	 *    see 45.1.3 WDT errata
	 */
	dly = div_ceil(48000000 * 2, 115000);
	dly >>= 3; /* ~8 cycles for one while loop */
	while (dly--) {
		;
	};
	WDT->CTRL = ctrl | WDT_CTRL_KEY(WDT_FIRST_KEY);
	WDT->CTRL = ctrl | WDT_CTRL_KEY(WDT_SECOND_KEY);
}

#define XTAL_FREQ 12000000
#define NR_PLLS 1
#define PLL_MAX_STARTUP_CYCLES (SCIF_PLL_PLLCOUNT_Msk >> SCIF_PLL_PLLCOUNT_Pos)

/**
 * Fcpu = 48MHz
 * Fpll = (Fclk * PLL_mul) / PLL_div
 */
#define CONFIG_PLL0_MUL        (192000000 / XTAL_FREQ)
#define CONFIG_PLL0_DIV         4

static inline bool pll_is_locked(u32_t pll_id)
{
	return !!(SCIF->PCLKSR & (1U << (6 + pll_id)));
}
static inline bool osc_is_ready(u8_t id)
{
	switch (id) {
	case OSC_ID_OSC0:
		return !!(SCIF->PCLKSR & SCIF_PCLKSR_OSC0RDY);
	case OSC_ID_OSC32:
		return !!(BSCIF->PCLKSR & BSCIF_PCLKSR_OSC32RDY);
	case OSC_ID_RC32K:
		return !!(BSCIF->RC32KCR & (BSCIF_RC32KCR_EN));
	case OSC_ID_RC80M:
		return !!(SCIF->RC80MCR & (SCIF_RC80MCR_EN));
	case OSC_ID_RCFAST:
		return !!(SCIF->RCFASTCFG & (SCIF_RCFASTCFG_EN));
	case OSC_ID_RC1M:
		return !!(BSCIF->RC1MCR & (BSCIF_RC1MCR_CLKOE));
	case OSC_ID_RCSYS:
		/* RCSYS is always ready */
		return true;
	default:
		/* unhandled_case(id); */
		return false;
	}
}
/**
 * The PLL options #PLL_OPT_VCO_RANGE_HIGH and #PLL_OPT_OUTPUT_DIV will
 * be set automatically based on the calculated target frequency.
 */
static inline u32_t pll_config_init(u32_t divide, u32_t mul)
{
#define SCIF0_PLL_VCO_RANGE1_MAX_FREQ   240000000
#define SCIF_PLL0_VCO_RANGE1_MIN_FREQ   160000000
#define SCIF_PLL0_VCO_RANGE0_MAX_FREQ   180000000
#define SCIF_PLL0_VCO_RANGE0_MIN_FREQ    80000000
/* VCO frequency range is 160-240 MHz (80-180 MHz if unset) */
#define PLL_OPT_VCO_RANGE_HIGH    0
/* Divide output frequency by two */
#define PLL_OPT_OUTPUT_DIV        1
/* The threshold above which to set the #PLL_OPT_VCO_RANGE_HIGH option */
#define PLL_VCO_LOW_THRESHOLD     ((SCIF_PLL0_VCO_RANGE1_MIN_FREQ \
	+ SCIF_PLL0_VCO_RANGE0_MAX_FREQ) / 2)
#define PLL_MIN_HZ 40000000
#define PLL_MAX_HZ 240000000
#define MUL_MIN    2
#define MUL_MAX    16
#define DIV_MIN    0
#define DIV_MAX    15

	u32_t pll_value;
	u32_t vco_hz;

	/* Calculate internal VCO frequency */
	vco_hz = XTAL_FREQ * mul;
	vco_hz /= divide;

	pll_value = 0;

	/* Bring the internal VCO frequency up to the minimum value */
	if ((vco_hz < PLL_MIN_HZ * 2) && (mul <= 8)) {
		mul *= 2;
		vco_hz *= 2;
		pll_value |= (1U << (SCIF_PLL_PLLOPT_Pos +
				     PLL_OPT_OUTPUT_DIV));
	}

	/* Set VCO frequency range according to calculated value */
	if (vco_hz >= PLL_VCO_LOW_THRESHOLD) {
		pll_value |= 1U << (SCIF_PLL_PLLOPT_Pos +
				    PLL_OPT_VCO_RANGE_HIGH);
	}

	pll_value |= ((mul - 1) << SCIF_PLL_PLLMUL_Pos) |
		      (divide << SCIF_PLL_PLLDIV_Pos) |
		      (PLL_MAX_STARTUP_CYCLES << SCIF_PLL_PLLCOUNT_Pos);

	return pll_value;
}

static inline void flashcalw_set_wait_state(u32_t wait_state)
{
	HFLASHC->FCR = (HFLASHC->FCR & ~FLASHCALW_FCR_FWS) |
			(wait_state ?
			 FLASHCALW_FCR_FWS_1 :
			 FLASHCALW_FCR_FWS_0);
}
static inline bool flashcalw_is_ready(void)
{
	return ((HFLASHC->FSR & FLASHCALW_FSR_FRDY) != 0);
}
static inline void flashcalw_issue_command(u32_t command, int page_number)
{
	u32_t time;

	flashcalw_is_ready();
	time = HFLASHC->FCMD;

	/* Clear the command bitfield. */
	time &= ~FLASHCALW_FCMD_CMD_Msk;
	if (page_number >= 0) {
		time = (FLASHCALW_FCMD_KEY_KEY |
			FLASHCALW_FCMD_PAGEN(page_number) | command);
	} else {
		time |= (FLASHCALW_FCMD_KEY_KEY | command);
	}

	HFLASHC->FCMD = time;
	/*HFLASHC->FSR & (FLASHCALW_FSR_LOCKE | FLASHCALW_FSR_PROGE);*/
	flashcalw_is_ready();
}

/**
 * @brief Setup various clock on SoC at boot time.
 *
 * Setup the SoC clocks according to section 28.12 in datasheet.
 *
 * Setup Slow, Main, PLLA, Processor and Master clocks during the device boot.
 * It is assumed that the relevant registers are at their reset value.
 */
static ALWAYS_INLINE void clock_init(void)
{
	wdt_set_ctrl(WDT->CTRL & ~WDT_CTRL_EN);
	while (WDT->CTRL & WDT_CTRL_EN) {
		;
	};

	/* Enable HCACHE */
	soc_pmc_peripheral_enable(
		PM_CLOCK_MASK(PM_CLK_GRP_HSB, SYSCLK_HRAMC1_DATA));
	soc_pmc_peripheral_enable(
		PM_CLOCK_MASK(PM_CLK_GRP_PBB, SYSCLK_HRAMC1_REGS));

	HCACHE->CTRL = HCACHE_CTRL_CEN_YES;

	while (!(HCACHE->SR & HCACHE_SR_CSTS_EN)) {
		;
	};

#if 1
	/* Enable PLL */
	if (!pll_is_locked(0)) {
		if (!osc_is_ready(OSC_ID_OSC0)) {
			/* This assumes external 12MHz Crystal */
			SCIF->UNLOCK = SCIF_UNLOCK_KEY(0xAAu) |
				       SCIF_UNLOCK_ADDR((u32_t)&SCIF->OSCCTRL0 -
							(u32_t)SCIF);
			SCIF->OSCCTRL0 = SCIF_OSCCTRL0_STARTUP(2) |
					 SCIF_OSCCTRL0_GAIN(3) |
					 SCIF_OSCCTRL0_MODE |
					 SCIF_OSCCTRL0_OSCEN;

			while (!osc_is_ready(OSC_ID_OSC0)) {
				;
			};
		}
		u32_t pll_config = pll_config_init(CONFIG_PLL0_DIV,
						   CONFIG_PLL0_MUL);

		SCIF->UNLOCK = SCIF_UNLOCK_KEY(0xAAu) |
			       SCIF_UNLOCK_ADDR((u32_t)&SCIF->PLL[0] -
						(u32_t)SCIF);
		SCIF->PLL[0] = pll_config | SCIF_PLL_PLLEN;

		while (!pll_is_locked(0)) {
			;
		};
	}

	/** Set a flash wait state depending on the new cpu frequency.
	 */
	flashcalw_set_wait_state(1);
	flashcalw_issue_command(FLASHCALW_FCMD_CMD_HSEN, -1);

	/** Set PLL0 as source clock
	 */
	PM->UNLOCK = PM_UNLOCK_KEY(0xAAu) |
		     PM_UNLOCK_ADDR((u32_t)&PM->MCCTRL - (u32_t)PM);
	PM->MCCTRL = OSC_SRC_PLL0;
#endif
#if 0
	u32_t temp;

	temp = BSCIF->RC1MCR;
	BSCIF->UNLOCK = BSCIF_UNLOCK_KEY(0xAAu) |
			BSCIF_UNLOCK_ADDR((u32_t)&BSCIF->RC1MCR -
					  (u32_t)BSCIF);
	BSCIF->RC1MCR = temp | BSCIF_RC1MCR_CLKOE;

	while (!osc_is_ready(OSC_ID_RC1M)) {
		;
	};

	/* Set a flash wait state depending on the new cpu frequency */
	flashcalw_set_wait_state(0);
	flashcalw_issue_command(FLASHCALW_FCMD_CMD_HSEN, -1);

	/* Set PLL0 as source clock */
	PM->UNLOCK = PM_UNLOCK_KEY(0xAAu) |
		     PM_UNLOCK_ADDR((u32_t)&PM->MCCTRL - (u32_t)PM);
	PM->MCCTRL = OSC_SRC_RC1M;
#endif

	/* Automatically switch to low power mode */
	/* u32_t pmcon = 0; */
	/* Read last PM_CON value */
	/* pmcon =BPM->PMCON; */
	/* Clear last PS Value */
	/* pmcon &= ~BPM_PMCON_PS_Msk; */
	/* Write new PS Value */
	/* pmcon |= BPM_PMCON_PS(0); */
	/* PSCM: without CPU halt */
	/* pmcon |= BPM_PMCON_PSCM; */
	/* Power Scaling Change Request */
	/* pmcon |= BPM_PMCON_PSCREQ; */
	/* Unlock PMCON register */
	/* BPM->UNLOCK = BPM_UNLOCK_KEY(0xAAu) |
	 *		 BPM_UNLOCK_ADDR((u32_t)&BPM->PMCON - (u32_t)BPM);
	 */
	/* Write back PM_CON value */
	/* BPM->PMCON = pmcon; */
	/* while ((BPM->SR & BPM_SR_PSOK) == 0); */
}

/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int atmel_sam4l_init(struct device *arg)
{
	u32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Setup system clocks. */
	clock_init();

	/*
	 * Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise.
	 */
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(atmel_sam4l_init, PRE_KERNEL_1, 0);
