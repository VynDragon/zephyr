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

const bl61x_pll_config *const bl61x_pll_configs[6] = {
&wifipll_32M, &wifipll_24M, &wifipll_32M, &wifipll_38P4M, &wifipll_40M, &wifipll_26M
};

#endif /* _PLL_DATA__H_ */
