/*
 * Copyright (c) 2024 MASSDRIVER EI (massdriver.space)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __CH32V20X_V30X_PINCTRL_H__
#define __CH32V20X_V30X_PINCTRL_H__

#define CH32V20X_V30X_PINMUX_PORT_PA 0
#define CH32V20X_V30X_PINMUX_PORT_PB 1
#define CH32V20X_V30X_PINMUX_PORT_PC 2
#define CH32V20X_V30X_PINMUX_PORT_PD 3
#define CH32V20X_V30X_PINMUX_PORT_PE 4

/*
 * Defines the starting bit for the remap field.
 */
#define CH32V20X_V30X_PINMUX_SPI1_RM    0
#define CH32V20X_V30X_PINMUX_I2C1_RM    1
#define CH32V20X_V30X_PINMUX_USART1_RM  2
#define CH32V20X_V30X_PINMUX_USART2_RM  3
#define CH32V20X_V30X_PINMUX_USART3_RM  4
#define CH32V20X_V30X_PINMUX_TIM1_RM    6
#define CH32V20X_V30X_PINMUX_TIM2_RM    8
#define CH32V20X_V30X_PINMUX_TIM3_RM    10
#define CH32V20X_V30X_PINMUX_TIM4_RM    12
#define CH32V20X_V30X_PINMUX_CAN1_RM    13
#define CH32V20X_V30X_PINMUX_PD01_RM    15
#define CH32V20X_V30X_PINMUX_TIM5CH4_RM 16
#define CH32V20X_V30X_PINMUX_ETH_RM     21
#define CH32V20X_V30X_PINMUX_CAN2_RM    22
#define CH32V20X_V30X_PINMUX_RMII_RM    23
#define CH32V20X_V30X_PINMUX_SDI_RM     24
#define CH32V20X_V30X_PINMUX_SPI3_RM    28

#define CH32V20X_V30X_PINMUX_TIM8_RM    (32 + 2)
#define CH32V20X_V30X_PINMUX_TIM9_RM    (32 + 3)
#define CH32V20X_V30X_PINMUX_TIM10_RM   (32 + 5)
#define CH32V20X_V30X_PINMUX_USART4_RM  (32 + 16)
#define CH32V20X_V30X_PINMUX_USART5_RM  (32 + 18)
#define CH32V20X_V30X_PINMUX_USART6_RM  (32 + 20)
#define CH32V20X_V30X_PINMUX_USART7_RM  (32 + 22)
#define CH32V20X_V30X_PINMUX_USART8_RM  (32 + 24)
#define CH32V20X_V30X_PINMUX_USART1_RM1 (32 + 26)

/* Port number with 0-4 */
#define CH32V20X_V30X_PINCTRL_PORT_SHIFT    0
/* Pin number 0-15 */
#define CH32V20X_V30X_PINCTRL_PIN_SHIFT     3
/* Base remap bit 0-31 */
#define CH32V20X_V30X_PINCTRL_RM_BASE_SHIFT 7
/* Remap Register ID */
#define CH32V20X_V30X_PINCTRL_PCFR_ID_SHIFT 12
/* Function remapping ID 0-3 */
#define CH32V20X_V30X_PINCTRL_RM_SHIFT      13

#define CH32V20X_V30X_PINMUX_DEFINE(port, pin, rm, remapping)                                      \
	((CH32V20X_V30X_PINMUX_PORT_##port << CH32V20X_V30X_PINCTRL_PORT_SHIFT) |                  \
	 (pin << CH32V20X_V30X_PINCTRL_PIN_SHIFT) |                                                \
	 (CH32V20X_V30X_PINMUX_##rm##_RM << CH32V20X_V30X_PINCTRL_RM_BASE_SHIFT) |                 \
	 (remapping << CH32V20X_V30X_PINCTRL_RM_SHIFT))

/* Pin swaps.
 * Warning: Some of those do not apply to all packages.
 * Verify using reference manual and use CH32V20X_V30X_PINMUX_DEFINE directly if needed.
 */

#define USART1_CK_PA8_0   CH32V20X_V30X_PINMUX_DEFINE(PA, 8, USART1, 0)
#define USART1_CK_PA8_1   CH32V20X_V30X_PINMUX_DEFINE(PA, 8, USART1, 1)
#define USART1_CK_PA10_2  CH32V20X_V30X_PINMUX_DEFINE(PA, 10, USART1, 2)
#define USART1_CK_PA5_3   CH32V20X_V30X_PINMUX_DEFINE(PA, 5, USART1, 3)
#define USART1_TX_PA9_0   CH32V20X_V30X_PINMUX_DEFINE(PA, 9, USART1, 0)
#define USART1_TX_PB6_1   CH32V20X_V30X_PINMUX_DEFINE(PB, 6, USART1, 1)
#define USART1_TX_PB15_2  CH32V20X_V30X_PINMUX_DEFINE(PB, 15, USART1, 2)
#define USART1_TX_PA6_3   CH32V20X_V30X_PINMUX_DEFINE(PA, 6, USART1, 3)
#define USART1_RX_PA10_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 10, USART1, 0)
#define USART1_RX_PB7_1   CH32V20X_V30X_PINMUX_DEFINE(PB, 7, USART1, 1)
#define USART1_RX_PA8_2   CH32V20X_V30X_PINMUX_DEFINE(PA, 8, USART1, 2)
#define USART1_RX_PA7_3   CH32V20X_V30X_PINMUX_DEFINE(PA, 7, USART1, 3)
#define USART1_CTS_PA11_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 11, USART1, 0)
#define USART1_CTS_PA11_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 11, USART1, 1)
#define USART1_CTS_PA5_2  CH32V20X_V30X_PINMUX_DEFINE(PA, 5, USART1, 2)
#define USART1_CTS_PC4_3  CH32V20X_V30X_PINMUX_DEFINE(PC, 4, USART1, 3)
#define USART1_RTS_PA12_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 12, USART1, 0)
#define USART1_RTS_PA12_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 12, USART1, 1)
#define USART1_RTS_PA9_2  CH32V20X_V30X_PINMUX_DEFINE(PA, 9, USART1, 2)
#define USART1_RTS_PC5_3  CH32V20X_V30X_PINMUX_DEFINE(PC, 5, USART1, 3)

#define USART2_CK_PA4_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 4, USART2, 0)
#define USART2_CK_PD7_1  CH32V20X_V30X_PINMUX_DEFINE(PD, 7, USART2, 1)
#define USART2_TX_PA2_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 2, USART2, 0)
#define USART2_TX_PD5_1  CH32V20X_V30X_PINMUX_DEFINE(PD, 5, USART2, 1)
#define USART2_RX_PA3_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 3, USART2, 0)
#define USART2_RX_PD6_1  CH32V20X_V30X_PINMUX_DEFINE(PD, 6, USART2, 1)
#define USART2_CTS_PA0_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 0, USART2, 0)
#define USART2_CTS_PD3_1 CH32V20X_V30X_PINMUX_DEFINE(PD, 3, USART2, 1)
#define USART2_RTS_PA1_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 1, USART2, 0)
#define USART2_RTS_PD4_1 CH32V20X_V30X_PINMUX_DEFINE(PD, 4, USART2, 1)

#define USART3_CK_PB12_0  CH32V20X_V30X_PINMUX_DEFINE(PB, 12, USART3, 0)
#define USART3_CK_PC12_1  CH32V20X_V30X_PINMUX_DEFINE(PC, 12, USART3, 1)
#define USART3_CK_PD10_2  CH32V20X_V30X_PINMUX_DEFINE(PD, 10, USART3, 2)
#define USART3_CK_PD10_3  CH32V20X_V30X_PINMUX_DEFINE(PD, 10, USART3, 3)
#define USART3_TX_PB10_0  CH32V20X_V30X_PINMUX_DEFINE(PB, 10, USART3, 0)
#define USART3_TX_PC10_1  CH32V20X_V30X_PINMUX_DEFINE(PC, 10, USART3, 1)
#define USART3_TX_PA13_2  CH32V20X_V30X_PINMUX_DEFINE(PA, 13, USART3, 2)
#define USART3_TX_PD8_3   CH32V20X_V30X_PINMUX_DEFINE(PD, 8, USART3, 3)
#define USART3_RX_PB11_0  CH32V20X_V30X_PINMUX_DEFINE(PB, 11, USART3, 0)
#define USART3_RX_PC11_1  CH32V20X_V30X_PINMUX_DEFINE(PC, 11, USART3, 1)
#define USART3_RX_PA14_2  CH32V20X_V30X_PINMUX_DEFINE(PA, 14, USART3, 2)
#define USART3_RX_PD9_3   CH32V20X_V30X_PINMUX_DEFINE(PD, 9, USART3, 3)
#define USART3_CTS_PB13_0 CH32V20X_V30X_PINMUX_DEFINE(PB, 13, USART3, 0)
#define USART3_CTS_PB13_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 13, USART3, 1)
#define USART3_CTS_PD11_2 CH32V20X_V30X_PINMUX_DEFINE(PD, 11, USART3, 2)
#define USART3_CTS_PD11_3 CH32V20X_V30X_PINMUX_DEFINE(PD, 11, USART3, 3)
#define USART3_RTS_PB14_0 CH32V20X_V30X_PINMUX_DEFINE(PB, 14, USART3, 0)
#define USART3_RTS_PB14_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 14, USART3, 1)
#define USART3_RTS_PD12_2 CH32V20X_V30X_PINMUX_DEFINE(PD, 12, USART3, 2)
#define USART3_RTS_PD12_3 CH32V20X_V30X_PINMUX_DEFINE(PD, 12, USART3, 3)

#define USART4_CK_PB2_0   CH32V20X_V30X_PINMUX_DEFINE(PB, 2, USART4, 0)
#define USART4_CK_PA6_1   CH32V20X_V30X_PINMUX_DEFINE(PA, 6, USART4, 1)
#define USART4_TX_PB0_0   CH32V20X_V30X_PINMUX_DEFINE(PB, 0, USART4, 0)
#define USART4_TX_PA5_1   CH32V20X_V30X_PINMUX_DEFINE(PA, 5, USART4, 1)
#define USART4_RX_PB1_0   CH32V20X_V30X_PINMUX_DEFINE(PB, 1, USART4, 0)
#define USART4_RX_PB5_1   CH32V20X_V30X_PINMUX_DEFINE(PB, 5, USART4, 1)
#define USART4_CTS_PB3_0  CH32V20X_V30X_PINMUX_DEFINE(PB, 3, USART4, 0)
#define USART4_CTS_PA7_1  CH32V20X_V30X_PINMUX_DEFINE(PA, 7, USART4, 1)
#define USART4_RTS_PB4_0  CH32V20X_V30X_PINMUX_DEFINE(PB, 4, USART4, 0)
#define USART4_RTS_PA15_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 15, USART4, 1)

#define USART5_TX_PC12_0 CH32V20X_V30X_PINMUX_DEFINE(PC, 12, USART5, 0)
#define USART5_TX_PB4_1  CH32V20X_V30X_PINMUX_DEFINE(PB, 4, USART5, 1)
#define USART5_TX_PE8_2  CH32V20X_V30X_PINMUX_DEFINE(PE, 8, USART5, 2)
#define USART5_RX_PD2_0  CH32V20X_V30X_PINMUX_DEFINE(PD, 2, USART5, 0)
#define USART5_RX_PB5_1  CH32V20X_V30X_PINMUX_DEFINE(PB, 5, USART5, 1)
#define USART5_RX_PE9_2  CH32V20X_V30X_PINMUX_DEFINE(PE, 9, USART5, 2)

#define USART6_TX_PC0_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 0, USART6, 0)
#define USART6_TX_PB8_1  CH32V20X_V30X_PINMUX_DEFINE(PB, 8, USART6, 1)
#define USART6_TX_PE10_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 10, USART6, 2)
#define USART6_RX_PC1_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 1, USART6, 0)
#define USART6_RX_PB9_1  CH32V20X_V30X_PINMUX_DEFINE(PB, 9, USART6, 1)
#define USART6_RX_PE11_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 11, USART6, 2)

#define USART7_TX_PC2_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 2, USART7, 0)
#define USART7_TX_PA6_1  CH32V20X_V30X_PINMUX_DEFINE(PA, 6, USART7, 1)
#define USART7_TX_PE12_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 12, USART7, 2)
#define USART7_RX_PC3_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 3, USART7, 0)
#define USART7_RX_PA7_1  CH32V20X_V30X_PINMUX_DEFINE(PA, 7, USART7, 1)
#define USART7_RX_PE13_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 13, USART7, 2)

#define USART8_TX_PC4_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 4, USART8, 0)
#define USART8_TX_PA14_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 14, USART8, 1)
#define USART8_TX_PE14_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 14, USART8, 2)
#define USART8_RX_PC5_0  CH32V20X_V30X_PINMUX_DEFINE(PC, 5, USART8, 0)
#define USART8_RX_PA15_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 15, USART8, 1)
#define USART8_RX_PE15_2 CH32V20X_V30X_PINMUX_DEFINE(PE, 15, USART8, 2)

#define SPI1_NSS_PA4_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 4, SPI1, 0)
#define SPI1_NSS_PA15_1 CH32V20X_V30X_PINMUX_DEFINE(PA, 15, SPI1, 1)
#define SPI1_SCK_PA5_0  CH32V20X_V30X_PINMUX_DEFINE(PA, 5, SPI1, 0)
#define SPI1_SCK_PB3_1  CH32V20X_V30X_PINMUX_DEFINE(PB, 3, SPI1, 1)
#define SPI1_MISO_PA6_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 6, SPI1, 0)
#define SPI1_MISO_PB4_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 4, SPI1, 1)
#define SPI1_MOSI_PA7_0 CH32V20X_V30X_PINMUX_DEFINE(PA, 7, SPI1, 0)
#define SPI1_MOSI_PB5_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 5, SPI1, 1)

#define I2C1_SCL_PB6_0 CH32V20X_V30X_PINMUX_DEFINE(PB, 6, I2C1, 0)
#define I2C1_SCL_PB8_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 8, I2C1, 1)
#define I2C1_SDA_PB7_0 CH32V20X_V30X_PINMUX_DEFINE(PB, 7, I2C1, 0)
#define I2C1_SDA_PB9_1 CH32V20X_V30X_PINMUX_DEFINE(PB, 9, I2C1, 1)

#endif /* __CH32V20X_V30X_PINCTRL_H__ */
