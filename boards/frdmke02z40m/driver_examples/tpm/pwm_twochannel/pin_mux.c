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

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v4.0
processor: MKE02Z64xxx4
package_id: MKE02Z64VQH4
mcu_data: ksdk2_0
processor_version: 0.0.9
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"


/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '52', peripheral: UART1, signal: RX, pin_signal: PTC6/UART1_RX}
  - {pin_num: '51', peripheral: UART1, signal: TX, pin_signal: PTC7/UART1_TX}
  - {pin_num: '32', peripheral: FTM0, signal: 'CH, 0', pin_signal: PTB2/KBI0_P6/SPI0_SCK/FTM0_CH0/ADC0_SE6}
  - {pin_num: '31', peripheral: FTM0, signal: 'CH, 1', pin_signal: PTB3/KBI0_P7/SPI0_MOSI/FTM0_CH1/ADC0_SE7}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* pin 32 is configured as FTM0_CH0 */
    PORT_SetPinSelect(kPORT_FTM0CH0, kPORT_FTM0_CH0_PTB2);
    /* pin 31 is configured as FTM0_CH1 */
    PORT_SetPinSelect(kPORT_FTM0CH1, kPORT_FTM0_CH1_PTB3);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
