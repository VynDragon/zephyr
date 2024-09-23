/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* PLL configuration data for BL61x */

#ifndef _PLL_DATA__H_
#define _PLL_DATA__H_

typedef struct {
	uint8_t	pllRefdivRatio;
	uint8_t	pllIntFracSw;
	uint8_t	pllIcp1u;
	uint8_t	pllIcp5u;
	uint8_t	pllRz;
	uint8_t	pllCz;
	uint8_t	pllC3;
	uint8_t	pllR4Short;
	uint8_t	pllC4En;
	uint8_t	pllSelSampleClk;
	uint8_t	pllVcoSpeed;
	uint8_t	pllSdmCtrlHw;
	uint8_t	pllSdmBypass;
	uint32_t	pllSdmin;
	uint8_t	aupllPostDiv;
} bl61x_pll_config;

/* XCLK is 32M */
const bl61x_pll_config wifipll_32M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1E00000,
	.aupllPostDiv = 0,
};

/* XCLK is 38.4M */
const bl61x_pll_config wifipll_38P4M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1900000,
	.aupllPostDiv = 0,
};

/* XCLK is 40M */
const bl61x_pll_config wifipll_40M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1800000,
	.aupllPostDiv = 0,
};

/* XCLK is 24M */
const bl61x_pll_config wifipll_24M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1400000,
	.aupllPostDiv = 0,
};

/* XCLK is 26M */
const bl61x_pll_config wifipll_26M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1276276,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_32M_O400M = {
    .pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2580000,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_40M_O400M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1E00000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_38P4M_O400M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1F40000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_24M_O400M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1900000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_26M_O400M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1713B13,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_32M_O480M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2D00000,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_40M_O480M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2400000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_38P4M_O480M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2580000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_24M_O480M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x1E00000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_26M_O480M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1BB13B1,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_32M_O520M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x30C0000,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_40M_O520M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2700000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_38P4M_O520M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x28A0000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_24M_O520M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x2080000,
	.aupllPostDiv = 0,
};

/* UnTested */
const bl61x_pll_config wifipll_26M_O520M = {
	.pllRefdivRatio = 1,
	.pllIntFracSw = 1,
	.pllIcp1u = 1,
	.pllIcp5u = 0,
	.pllRz = 5,
	.pllCz = 2,
	.pllC3 = 2,
	.pllR4Short = 0,
	.pllC4En = 1,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 0,
	.pllSdmBypass = 0,
	.pllSdmin = 0x1DFFFFF,
	.aupllPostDiv = 0,
};

/* Tested */
const bl61x_pll_config wifipll_32M_O512M = {
	.pllRefdivRatio = 2,
	.pllIntFracSw = 0,
	.pllIcp1u = 0,
	.pllIcp5u = 2,
	.pllRz = 3,
	.pllCz = 1,
	.pllC3 = 2,
	.pllR4Short = 1,
	.pllC4En = 0,
	.pllSelSampleClk = 1,
	.pllVcoSpeed = 5,
	.pllSdmCtrlHw = 1,
	.pllSdmBypass = 1,
	.pllSdmin = 0x3000000,
	.aupllPostDiv = 0,
};


const bl61x_pll_config *const bl61x_pll_configs[6] = {
&wifipll_32M, &wifipll_24M, &wifipll_32M, &wifipll_38P4M, &wifipll_40M, &wifipll_26M
};

const bl61x_pll_config *const bl61x_pll_configs_O400M[6] = {
&wifipll_32M_O400M, &wifipll_24M_O400M, &wifipll_32M_O400M, &wifipll_38P4M_O400M, &wifipll_40M_O400M, &wifipll_26M_O400M
};

const bl61x_pll_config *const bl61x_pll_configs_O480M[6] = {
&wifipll_32M_O480M, &wifipll_24M_O480M, &wifipll_32M_O480M, &wifipll_38P4M_O480M, &wifipll_40M_O480M, &wifipll_26M_O480M
};

/* absolute fastness, most likely to be unstable due to 86M bclk instead of 80M */
const bl61x_pll_config *const bl61x_pll_configs_O520M[6] = {
&wifipll_32M_O520M, &wifipll_24M_O520M, &wifipll_32M_O520M, &wifipll_38P4M_O520M, &wifipll_40M_O520M, &wifipll_26M_O520M
};


#endif /* _PLL_DATA__H_ */
