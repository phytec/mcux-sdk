/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.0
processor: MKE04Z8xxx4
package_id: MKE04Z8VFK4
mcu_data: ksdk2_0
processor_version: 0.0.4
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '16', peripheral: UART0, signal: RX, pin_signal: PTB0/KBI0_P4/UART0_RX/SPI0_PCS0/PWT_IN1/ADC0_SE4}
  - {pin_num: '15', peripheral: UART0, signal: TX, pin_signal: PTB1/KBI0_P5/UART0_TX/SPI0_MISO/TCLK2/ADC0_SE5}
  - {pin_num: '8', peripheral: SPI0, signal: MISO, pin_signal: PTB4/KBI1_P6/FTM2_CH4/SPI0_MISO/ACMP1_IN2/NMI_b}
  - {pin_num: '13', peripheral: SPI0, signal: MOSI, pin_signal: PTB3/KBI0_P7/SPI0_MOSI/FTM0_CH1/ADC0_SE7}
  - {pin_num: '7', peripheral: SPI0, signal: PCS, pin_signal: PTB5/KBI1_P7/FTM2_CH5/SPI0_PCS0/ACMP1_OUT}
  - {pin_num: '14', peripheral: SPI0, signal: SCK, pin_signal: PTB2/KBI0_P6/SPI0_SCK/FTM0_CH0/ACMP0_IN0/ADC0_SE6}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  PORT_SetPinSelect(kPORT_SPI0, kPORT_SPI0_SCKPTB2_MOSIPTB3_MISOPTB4_PCSPTB5);  /* pin 7,14,8,13 is configured as SPI0_PCS0, SPI0_SCK, SPI0_MISO, SPI0_MOSI */
  PORT_SetPinSelect(kPORT_UART0, kPORT_UART0_RXPTB0_TXPTB1);  /* pin 16,15 is configured as UART0_RX, UART0_TX */
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
