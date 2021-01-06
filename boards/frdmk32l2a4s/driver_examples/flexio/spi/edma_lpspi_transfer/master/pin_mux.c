/*
 * Copyright 2019 NXP
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
product: Pins v6.0
processor: K32L2A41xxxxA
package_id: K32L2A41VLL1A
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '62', peripheral: LPUART0, signal: RX, pin_signal: TSI0_CH9/PTB16/LPSPI1_SOUT/LPUART0_RX/TPM0_CLKIN/LPSPI2_PCS3/FXIO0_D16}
  - {pin_num: '63', peripheral: LPUART0, signal: TX, pin_signal: TSI0_CH10/PTB17/LPSPI1_SIN/LPUART0_TX/TPM1_CLKIN/LPSPI2_PCS2/FXIO0_D17}
  - {pin_num: '66', peripheral: LPSPI2, signal: PCS0, pin_signal: PTB20/LPSPI2_PCS0/CMP0_OUT}
  - {pin_num: '67', peripheral: LPSPI2, signal: SCK, pin_signal: PTB21/LPSPI2_SCK/CMP1_OUT}
  - {pin_num: '68', peripheral: LPSPI2, signal: SOUT, pin_signal: PTB22/LPSPI2_SOUT}
  - {pin_num: '69', peripheral: LPSPI2, signal: SIN, pin_signal: PTB23/LPSPI2_SIN}
  - {pin_num: '93', peripheral: FLEXIO0, signal: 'D, 0', pin_signal: PTD0/LLWU_P12/LPSPI0_PCS0/LPUART2_RTS_b/TPM0_CH0/FXIO0_D0}
  - {pin_num: '94', peripheral: FLEXIO0, signal: 'D, 1', pin_signal: ADC0_SE5b/PTD1/LPSPI0_SCK/LPUART2_CTS_b/TPM0_CH1/FXIO0_D1}
  - {pin_num: '95', peripheral: FLEXIO0, signal: 'D, 2', pin_signal: PTD2/LLWU_P13/LPSPI0_SOUT/LPUART2_RX/TPM0_CH2/FXIO0_D2}
  - {pin_num: '96', peripheral: FLEXIO0, signal: 'D, 3', pin_signal: PTD3/LPSPI0_SIN/LPUART2_TX/TPM0_CH3/FXIO0_D3}
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
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);

    /* PORTB16 (pin 62) is configured as LPUART0_RX */
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt3);

    /* PORTB17 (pin 63) is configured as LPUART0_TX */
    PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt3);

    /* PORTB20 (pin 66) is configured as LPSPI2_PCS0 */
    PORT_SetPinMux(PORTB, 20U, kPORT_MuxAlt2);

    /* PORTB21 (pin 67) is configured as LPSPI2_SCK */
    PORT_SetPinMux(PORTB, 21U, kPORT_MuxAlt2);

    /* PORTB22 (pin 68) is configured as LPSPI2_SOUT */
    PORT_SetPinMux(PORTB, 22U, kPORT_MuxAlt2);

    /* PORTB23 (pin 69) is configured as LPSPI2_SIN */
    PORT_SetPinMux(PORTB, 23U, kPORT_MuxAlt2);

    /* PORTD0 (pin 93) is configured as FXIO0_D0 */
    PORT_SetPinMux(PORTD, 0U, kPORT_MuxAlt6);

    /* PORTD1 (pin 94) is configured as FXIO0_D1 */
    PORT_SetPinMux(PORTD, 1U, kPORT_MuxAlt6);

    /* PORTD2 (pin 95) is configured as FXIO0_D2 */
    PORT_SetPinMux(PORTD, 2U, kPORT_MuxAlt6);

    /* PORTD3 (pin 96) is configured as FXIO0_D3 */
    PORT_SetPinMux(PORTD, 3U, kPORT_MuxAlt6);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
