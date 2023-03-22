/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "fsl_tlv320aic3110.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/
#define TLV320AIC3110_CHECK_RET(x, status)  \
    (status) = (x);                  \
    if ((status) != kStatus_Success) \
    {                                \
        return (status);             \
    }

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static status_t TLV320AIC3110_SetInternalPllConfig(
    tlv320aic3110_handle_t *handle, uint32_t inputMclk, uint32_t outputClk, uint32_t sampleRate, uint32_t bitWidth)
{
    status_t ret   = kStatus_Success;

    return ret;
}

static status_t TLV320AIC3110_SetMasterClock(tlv320aic3110_handle_t *handle, uint32_t sysclk, uint32_t sampleRate, uint32_t bitWidth)
{
    status_t ret = kStatus_Success;

    return ret;
}

status_t TLV320AIC3110_Init(tlv320aic3110_handle_t *handle, const tlv320aic3110_config_t *config)
{
    status_t ret = kStatus_Success;

    handle->config  = config;
    uint32_t sysclk = config->format.mclk_HZ;

    /* i2c bus initialization */
    if (CODEC_I2C_Init(handle->i2cHandle, config->i2cConfig.codecI2CInstance, TLV320AIC3110_I2C_BAUDRATE,
                       config->i2cConfig.codecI2CSourceClock) != (status_t)kStatus_HAL_I2cSuccess)
    {
        return kStatus_Fail;
    }

    // Page 0 is selected
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x00);

    // SW Reset
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RESET, 0x01);

    //PLL_clkin = MCLK,codec_clkin = PLL_CLK
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_CLKMUX, 0x03);
    //PLL Power up, P = 1, R = 1
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PLLPR, 0x91);
	//J = 8
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PLLJ, 0x08);
	//D = 0000, D(13:8) = 0
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PLLDMSB, 0x00);
    //D(7:0) = 0
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PLLDLSB, 0x00);
	//mode is i2s,wordlength is 16
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_IFACE1, 0x00);
	// NDAC is powered up and set to 4
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_NDAC, 0x84);
	// MDAC is powered up and set to 4
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_MDAC, 0x84);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_NADC, 0x84);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_MADC, 0x84);
	// DOSR = 128, DOSR(9:8) = 0
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DOSRMSB, 0x00);
	// DOSR(7:0) = 128
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DOSRLSB, 0x80);
	// DAC => volume control thru pin disable
    ret = TLV320AIC3110_WriteReg(handle, 0x74, 0x00);
	// DAC => drc disable, th and hy
    ret = TLV320AIC3110_WriteReg(handle, 0x44, 0x00);
	// DAC => 0 db gain left
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_LDACVOL, 0x00);
	// DAC => 0 db gain right
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RDACVOL, 0x00);

    // Page 1 is selected
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x01);
	// De-pop, Power on = 800 ms, Step time = 4 ms
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_HPPOP, 0x4e);
	// HPL and HPR powered up
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_HPDRIVER, 0xc2);
	// LDAC routed to HPL, RDAC routed to HPR
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMIXERROUTE, 0x88);
	// HPL unmute and gain 1db
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_HPLGAIN, 0x0e);
	// HPR unmute and gain 1db
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_HPRGAIN, 0x0e);
	// No attenuation on HP
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_LANALOGHPL, 0x00);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RANALOGHPR, 0x00);
	// MIC BIAS = AVDD
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_MICBIAS, 0x0b);
	// MICPGA P = MIC 10k
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_MICPGAPI, 0x40);
	// MICPGA M - CM 10k
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_MICPGAMI, 0x40);

    // Page 0 is selected
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x00);
	// select DAC DSP mode 11 & enable adaptive filter
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACPRB, 0x0b);

    // Page 8 is selcted
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x08);
    ret = TLV320AIC3110_WriteReg(handle, 0x01, 0x04);

    // Page 0 is selcted
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x00);
	// POWERUP DAC left and right channels (soft step disable)
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACSETUP, 0xd6);
	// UNMUTE DAC left and right channels
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMUTE, 0x00);
    // POWERUP ADC channel
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_ADCSETUP, 0x80);
	// UNMUTE ADC channel
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_ADCFGA, 0x00);

    // Page 1 is selected
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x01);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMIXERROUTE, 0x08);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMIXERROUTE, 0x00);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_LANALOGSPL, 0x00);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RANALOGSPR, 0x00);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMIXERROUTE, 0x40);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_DACMIXERROUTE, 0x44);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_LANALOGSPL, 0x80);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RANALOGSPR, 0x80);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_SPLGAIN, 0x0d);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_SPRGAIN, 0x0d);

    // Page 1 is selected
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x01);
	// Unmute Class-D Left
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_SPLGAIN, 0x1c);
	// Unmute Class-D Right
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_SPRGAIN, 0x1c);
	// Power-up Class-D drivers
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_SPKAMP, 0xc6);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_LANALOGHPL, 0x30);
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_RANALOGHPR, 0x30);

#if 0
    ret = TLV320AIC3110_WriteReg(handle, AIC31XX_PAGECTL, 0x00);

    // Beep Length MSB
    ret = TLV320AIC3110_WriteReg(handle, 0x49, 0xf);
    // Beep-Length Middle Bits
    ret = TLV320AIC3110_WriteReg(handle, 0x4A, 0xf);
    // Beep-Length LSB
    ret = TLV320AIC3110_WriteReg(handle, 0x4B, 0xf);

    // Left Beep Generator
    ret = TLV320AIC3110_WriteReg(handle, 0x47, 0x8f);
    // Right Beep Generator
    ret = TLV320AIC3110_WriteReg(handle, 0x48, 0x8f);

    // Beep Sin(x) MSB
    //ret = TLV320AIC3110_WriteReg(handle, 0x4C, 0x);
    // Beep Sin(x) LSB
    //ret = TLV320AIC3110_WriteReg(handle, 0x4D, 0x);
    // Beep Cos(x) MSB
    //ret = TLV320AIC3110_WriteReg(handle, 0x4E, 0x);
    // Beep Cos(x) LSB
    //ret = TLV320AIC3110_WriteReg(handle, 0x4F, 0x);
#endif

    return ret;
}

status_t TLV320AIC3110_Deinit(tlv320aic3110_handle_t *handle)
{
    status_t ret = kStatus_Success;

    return ret;
}

void TLV320AIC3110_SetMasterSlave(tlv320aic3110_handle_t *handle, bool master)
{
    if (master)
    {
    }
    else
    {
    }
}

status_t TLV320AIC3110_SetModule(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            break;
        case kTLV320AIC3110_ModuleDAC:
            break;
        case kTLV320AIC3110_ModuleVREF:
            break;
        case kTLV320AIC3110_ModuleLineIn:
            break;
        case kTLV320AIC3110_ModuleLineOut:
            break;
        case kTLV320AIC3110_ModuleMICB:
            break;
        case kTLV320AIC3110_ModuleSpeaker:
            break;
        case kTLV320AIC3110_ModuleOMIX:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_SetDataRoute(tlv320aic3110_handle_t *handle, tlv320aic3110_route_t route)
{
    status_t ret = kStatus_Success;
    switch (route)
    {
        case kTLV320AIC3110_RouteBypass:
            break;
        case kTLV320AIC3110_RoutePlayback:
            break;
        case kTLV320AIC3110_RoutePlaybackandRecord:
            break;
        case kTLV320AIC3110_RouteRecord:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_SetLeftInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input)
{
    status_t ret = kStatus_Success;

    switch (input)
    {
        case kTLV320AIC3110_InputClosed:
            break;
        case kTLV320AIC3110_InputSingleEndedMic:
            break;
        case kTLV320AIC3110_InputDifferentialMicInput2:
            break;
        case kTLV320AIC3110_InputDifferentialMicInput3:
            break;
        case kTLV320AIC3110_InputLineINPUT2:
            break;
        case kTLV320AIC3110_InputLineINPUT3:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }

    return ret;
}

status_t TLV320AIC3110_SetRightInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input)
{
    status_t ret = kStatus_Success;
    uint16_t val = 0;

    switch (input)
    {
        case kTLV320AIC3110_InputClosed:
            break;
        case kTLV320AIC3110_InputSingleEndedMic:
            break;
        case kTLV320AIC3110_InputDifferentialMicInput2:
            break;
        case kTLV320AIC3110_InputDifferentialMicInput3:
            break;
        case kTLV320AIC3110_InputLineINPUT2:
            break;
        case kTLV320AIC3110_InputLineINPUT3:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }

    return ret;
}

status_t TLV320AIC3110_SetProtocol(tlv320aic3110_handle_t *handle, tlv320aic3110_protocol_t protocol)
{

}

status_t TLV320AIC3110_SetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, uint32_t volume)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            break;
        case kTLV320AIC3110_ModuleDAC:
            break;
        case kTLV320AIC3110_ModuleHP:
            break;
        case kTLV320AIC3110_ModuleLineIn:
            break;
        case kTLV320AIC3110_ModuleSpeaker:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

uint32_t TLV320AIC3110_GetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module)
{
    uint16_t vol = 0;

    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            break;
        case kTLV320AIC3110_ModuleDAC:
            break;
        case kTLV320AIC3110_ModuleHP:
            break;
        case kTLV320AIC3110_ModuleLineOut:
            break;
        default:
            break;
    }
    return vol;
}

status_t TLV320AIC3110_SetMute(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled)
{
    status_t ret = kStatus_Success;
    switch (module)
    {
        case kTLV320AIC3110_ModuleADC:
            break;
        case kTLV320AIC3110_ModuleDAC:
            break;
        case kTLV320AIC3110_ModuleHP:
            break;
        case kTLV320AIC3110_ModuleSpeaker:
            break;
        case kTLV320AIC3110_ModuleLineOut:
            break;
        default:
            ret = kStatus_InvalidArgument;
            break;
    }
    return ret;
}

status_t TLV320AIC3110_ConfigDataFormat(tlv320aic3110_handle_t *handle, uint32_t sysclk, uint32_t sample_rate, uint32_t bits)
{
    status_t retval  = kStatus_Success;

    return retval;
}

status_t TLV320AIC3110_SetJackDetect(tlv320aic3110_handle_t *handle, bool isEnabled)
{
    status_t retval = 0;

    return retval;
}

status_t TLV320AIC3110_WriteReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t val)
{
    uint8_t cmd;
    uint8_t buff = (uint8_t)val & 0xFFU;

    /* The register address */
    cmd = reg;

    return CODEC_I2C_Send(handle->i2cHandle, handle->config->slaveAddress, cmd, 1U, &buff, 1U);
}

status_t TLV320AIC3110_ReadReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t *val)
{
    status_t retval = 0;
    uint8_t cmd;
    uint8_t buff = 0U;

    /* The register address */
    cmd = reg;

    retval = CODEC_I2C_Receive(handle->i2cHandle, handle->config->slaveAddress, cmd, 1U, &buff, 1U);

    *val = buff;

    return retval;
}

status_t TLV320AIC3110_ModifyReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t mask, uint16_t val)
{
    status_t retval  = 0;
    uint16_t reg_val = 0;
    retval           = TLV320AIC3110_ReadReg(handle, reg, &reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    reg_val &= (uint16_t)~mask;
    reg_val |= val;
    retval = TLV320AIC3110_WriteReg(handle, reg, reg_val);
    if (retval != kStatus_Success)
    {
        return kStatus_Fail;
    }
    return kStatus_Success;
}

status_t TLV320AIC3110_SetPlay(tlv320aic3110_handle_t *handle, uint32_t playSource)
{
    status_t ret = kStatus_Success;

    return ret;
}
