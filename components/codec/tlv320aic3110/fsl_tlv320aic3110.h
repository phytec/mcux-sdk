/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_TLV320AIC3110_H_
#define _FSL_TLV320AIC3110_H_

#include "fsl_codec_i2c.h"
#include "fsl_common.h"

/*!
 * @addtogroup tlv320aic3110
 * @ingroup codec
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief CLOCK driver version 2.2.1 */
#define FSL_TLV320AIC3110_DRIVER_VERSION (MAKE_VERSION(2, 2, 1))
/*@}*/

/*! @brief tlv320aic3110 handle size */
#ifndef TLV320AIC3110_I2C_HANDLER_SIZE
#define TLV320AIC3110_I2C_HANDLER_SIZE CODEC_I2C_MASTER_HANDLER_SIZE
#endif

/*! @brief Define the register address of TLV320AIC3110. */
#define AIC31XX_PAGECTL		0x0U /* Page Control Register */

/* Page 0 Registers */
#define AIC31XX_RESET		0x1U /* Software reset register */
#define AIC31XX_OT_FLAG		0x3U /* OT FLAG register */
#define AIC31XX_CLKMUX		0x4U /* Clock clock Gen muxing, Multiplexers*/
#define AIC31XX_PLLPR		0x5U /* PLL P and R-VAL register */
#define AIC31XX_PLLJ		0x6U /* PLL J-VAL register */
#define AIC31XX_PLLDMSB		0x7U /* PLL D-VAL MSB register */
#define AIC31XX_PLLDLSB		0x8U /* PLL D-VAL LSB register */
#define AIC31XX_NDAC		0xBU /* DAC NDAC_VAL register*/
#define AIC31XX_MDAC		0xCU /* DAC MDAC_VAL register */
#define AIC31XX_DOSRMSB		0xDU /* DAC OSR setting register 1, MSB value */
#define AIC31XX_DOSRLSB		0xEU /* DAC OSR setting register 2, LSB value */
#define AIC31XX_MINI_DSP_INPOL	0x10U
#define AIC31XX_NADC		0x12U /* Clock setting register 8, PLL */
#define AIC31XX_MADC		0x13U /* Clock setting register 9, PLL */
#define AIC31XX_AOSR		0x14U /* ADC Oversampling (AOSR) Register */
#define AIC31XX_CLKOUTMUX	0x19U /* Clock setting register 9, Multiplexers */
#define AIC31XX_CLKOUTMVAL	0x1AU /* Clock setting register 10, CLOCKOUT M divider value */
#define AIC31XX_IFACE1		0x1BU /* Audio Interface Setting Register 1 */
#define AIC31XX_DATA_OFFSET	0x1CU /* Audio Data Slot Offset Programming */
#define AIC31XX_IFACE2		0x1DU /* Audio Interface Setting Register 2 */
#define AIC31XX_BCLKN		0x1EU /* Clock setting register 11, BCLK N Divider */
#define AIC31XX_IFACESEC1	0x1FU /* Audio Interface Setting Register 3, Secondary Audio Interface */
#define AIC31XX_IFACESEC2	0x20U /* Audio Interface Setting Register 4 */
#define AIC31XX_IFACESEC3	0x21U /* Audio Interface Setting Register 5 */
#define AIC31XX_I2C		0x22U /* I2C Bus Condition */
#define AIC31XX_ADCFLAG		0x24U /* ADC FLAG */
#define AIC31XX_DACFLAG1	0x25U /* DAC Flag Registers */
#define AIC31XX_DACFLAG2	0x26U
#define AIC31XX_OFFLAG		0x27U /* Sticky Interrupt flag (overflow) */
#define AIC31XX_INTRDACFLAG	0x2CU /* Sticy DAC Interrupt flags */
#define AIC31XX_INTRADCFLAG	0x2DU /* Sticy ADC Interrupt flags */
#define AIC31XX_INTRDACFLAG2	0x2EU /* DAC Interrupt flags 2 */
#define AIC31XX_INTRADCFLAG2	0x2FU /* ADC Interrupt flags 2 */
#define AIC31XX_INT1CTRL	0x30U /* INT1 interrupt control */
#define AIC31XX_INT2CTRL	0x31U /* INT2 interrupt control */
#define AIC31XX_GPIO1		0x33U /* GPIO1 control */
#define AIC31XX_DACPRB		0x3CU
#define AIC31XX_ADCPRB		0x3DU /* ADC Instruction Set Register */
#define AIC31XX_DACSETUP	0x3FU /* DAC channel setup register */
#define AIC31XX_DACMUTE		0x40U /* DAC Mute and volume control register */
#define AIC31XX_LDACVOL		0x41U /* Left DAC channel digital volume control */
#define AIC31XX_RDACVOL		0x42U /* Right DAC channel digital volume control */
#define AIC31XX_HSDETECT	0x43U /* Headset detection */
#define AIC31XX_ADCSETUP	0x51U /* ADC Digital Mic */
#define AIC31XX_ADCFGA		0x52U /* ADC Digital Volume Control Fine Adjust */
#define AIC31XX_ADCVOL		0x53U /* ADC Digital Volume Control Coarse Adjust */

/* Page 1 Registers */
#define AIC31XX_HPDRIVER	0x1FU /* Headphone drivers */
#define AIC31XX_SPKAMP		0x20U /* Class-D Speakear Amplifier */
#define AIC31XX_HPPOP		0x21U /* HP Output Drivers POP Removal Settings */
#define AIC31XX_SPPGARAMP	0x22U /* Output Driver PGA Ramp-Down Period Control */
#define AIC31XX_DACMIXERROUTE	0x23U /* DAC_L and DAC_R Output Mixer Routing */
#define AIC31XX_LANALOGHPL	0x24U /* Left Analog Vol to HPL */
#define AIC31XX_RANALOGHPR	0x25U /* Right Analog Vol to HPR */
#define AIC31XX_LANALOGSPL	0x26U /* Left Analog Vol to SPL */
#define AIC31XX_RANALOGSPR	0x27U /* Right Analog Vol to SPR */
#define AIC31XX_HPLGAIN		0x28U /* HPL Driver */
#define AIC31XX_HPRGAIN		0x29U /* HPR Driver */
#define AIC31XX_SPLGAIN		0x2AU /* SPL Driver */
#define AIC31XX_SPRGAIN		0x2BU /* SPR Driver */
#define AIC31XX_HPCONTROL	0x2CU /* HP Driver Control */
#define AIC31XX_MICBIAS		0x2EU /* MIC Bias Control */
#define AIC31XX_MICPGA		0x2FU /* MIC PGA*/
#define AIC31XX_MICPGAPI	0x30U /* Delta-Sigma Mono ADC Channel Fine-Gain Input Selection for P-Terminal */
#define AIC31XX_MICPGAMI	0x31U /* ADC Input Selection for M-Terminal */
#define AIC31XX_MICPGACM	0x32U /* Input CM Settings */

/*! @brief Cache register number */
#define TLV320AIC3110_CACHEREGNUM 66U

/*! @brief TLV320AIC3110 I2C address. */
#define TLV320AIC3110_I2C_ADDR 0x18
/*! @brief TLV320AIC3110 I2C baudrate */
#define TLV320AIC3110_I2C_BAUDRATE (100000U)

/*! @brief Modules in TLV320AIC3110 board. */
typedef enum _tlv320aic3110_module
{
    kTLV320AIC3110_ModuleADC     = 0, /*!< ADC module in TLV320AIC3110 */
    kTLV320AIC3110_ModuleDAC     = 1, /*!< DAC module in TLV320AIC3110 */
    kTLV320AIC3110_ModuleVREF    = 2, /*!< VREF module */
    kTLV320AIC3110_ModuleHP      = 3, /*!< Headphone */
    kTLV320AIC3110_ModuleMICB    = 4, /*!< Mic bias */
    kTLV320AIC3110_ModuleMIC     = 5, /*!< Input Mic */
    kTLV320AIC3110_ModuleLineIn  = 6, /*!< Analog in PGA  */
    kTLV320AIC3110_ModuleLineOut = 7, /*!< Line out module */
    kTLV320AIC3110_ModuleSpeaker = 8, /*!< Speaker module */
    kTLV320AIC3110_ModuleOMIX    = 9, /*!< Output mixer */
} tlv320aic3110_module_t;

/*! @brief tlv320aic3110 play channel
 * @anchor _tlv320aic3110_play_channel
 */
enum
{
    kTLV320AIC3110_HeadphoneLeft  = 1, /*!< tlv320aic3110 headphone left channel */
    kTLV320AIC3110_HeadphoneRight = 2, /*!< tlv320aic3110 headphone right channel */
    kTLV320AIC3110_SpeakerLeft    = 4, /*!< tlv320aic3110 speaker left channel */
    kTLV320AIC3110_SpeakerRight   = 8, /*!< tlv320aic3110 speaker right channel */
};

/*! @brief tlv320aic3110 play source */
typedef enum _tlv320aic3110_play_source
{
    kTLV320AIC3110_PlaySourcePGA   = 1, /*!< tlv320aic3110 play source PGA */
    kTLV320AIC3110_PlaySourceInput = 2, /*!< tlv320aic3110 play source Input */
    kTLV320AIC3110_PlaySourceDAC   = 4, /*!< tlv320aic3110 play source DAC */
} tlv320aic3110_play_source_t;

/*!
 * @brief TLV320AIC3110 data route.
 * Only provide some typical data route, not all route listed.
 * Note: Users cannot combine any routes, once a new route is set, the previous one would be replaced.
 */
typedef enum _tlv320aic3110_route
{
    kTLV320AIC3110_RouteBypass            = 0, /*!< LINEIN->Headphone. */
    kTLV320AIC3110_RoutePlayback          = 1, /*!<  I2SIN->DAC->Headphone. */
    kTLV320AIC3110_RoutePlaybackandRecord = 2, /*!< I2SIN->DAC->Headphone, LINEIN->ADC->I2SOUT. */
    kTLV320AIC3110_RouteRecord            = 5  /*!< LINEIN->ADC->I2SOUT. */
} tlv320aic3110_route_t;

/*!
 * @brief The audio data transfer protocol choice.
 * TLV320AIC3110 only supports I2S format and PCM format.
 */
typedef enum _tlv320aic3110_protocol
{
    kTLV320AIC3110_BusI2S            = 2,           /*!< I2S type */
    kTLV320AIC3110_BusLeftJustified  = 1,           /*!< Left justified mode */
    kTLV320AIC3110_BusRightJustified = 0,           /*!< Right justified mode */
    kTLV320AIC3110_BusPCMA           = 3,           /*!< PCM A mode */
    kTLV320AIC3110_BusPCMB           = 3 | (1 << 4) /*!< PCM B mode */
} tlv320aic3110_protocol_t;

/*! @brief tlv320aic3110 input source */
typedef enum _tlv320aic3110_input
{
    kTLV320AIC3110_InputClosed                = 0, /*!< Input device is closed */
    kTLV320AIC3110_InputSingleEndedMic        = 1, /*!< Input as single ended mic, only use L/RINPUT1 */
    kTLV320AIC3110_InputDifferentialMicInput2 = 2, /*!< Input as differential mic, use L/RINPUT1 and L/RINPUT2 */
    kTLV320AIC3110_InputDifferentialMicInput3 = 3, /*!< Input as differential mic, use L/RINPUT1 and L/RINPUT3*/
    kTLV320AIC3110_InputLineINPUT2            = 4, /*!< Input as line input, only use L/RINPUT2 */
    kTLV320AIC3110_InputLineINPUT3            = 5  /*!< Input as line input, only use L/RINPUT3 */
} tlv320aic3110_input_t;

/*! @brief audio sample rate definition
 * @anchor _tlv320aic3110_sample_rate
 */
enum
{
    kTLV320AIC3110_AudioSampleRate8KHz    = 8000U,   /*!< Sample rate 8000 Hz */
    kTLV320AIC3110_AudioSampleRate11025Hz = 11025U,  /*!< Sample rate 11025 Hz */
    kTLV320AIC3110_AudioSampleRate12KHz   = 12000U,  /*!< Sample rate 12000 Hz */
    kTLV320AIC3110_AudioSampleRate16KHz   = 16000U,  /*!< Sample rate 16000 Hz */
    kTLV320AIC3110_AudioSampleRate22050Hz = 22050U,  /*!< Sample rate 22050 Hz */
    kTLV320AIC3110_AudioSampleRate24KHz   = 24000U,  /*!< Sample rate 24000 Hz */
    kTLV320AIC3110_AudioSampleRate32KHz   = 32000U,  /*!< Sample rate 32000 Hz */
    kTLV320AIC3110_AudioSampleRate44100Hz = 44100U,  /*!< Sample rate 44100 Hz */
    kTLV320AIC3110_AudioSampleRate48KHz   = 48000U,  /*!< Sample rate 48000 Hz */
    kTLV320AIC3110_AudioSampleRate96KHz   = 96000U,  /*!< Sample rate 96000 Hz */
    kTLV320AIC3110_AudioSampleRate192KHz  = 192000U, /*!< Sample rate 192000 Hz */
    kTLV320AIC3110_AudioSampleRate384KHz  = 384000U, /*!< Sample rate 384000 Hz */
};

/*! @brief audio bit width
 * @anchor _tlv320aic3110_audio_bit_width
 */
enum
{
    kTLV320AIC3110_AudioBitWidth16bit = 16U, /*!< audio bit width 16 */
    kTLV320AIC3110_AudioBitWidth20bit = 20U, /*!< audio bit width 20 */
    kTLV320AIC3110_AudioBitWidth24bit = 24U, /*!< audio bit width 24 */
    kTLV320AIC3110_AudioBitWidth32bit = 32U, /*!< audio bit width 32 */
};

/*! @brief tlv320aic3110 sysclk source */
typedef enum _tlv320aic3110_sysclk_source
{
    kTLV320AIC3110_SysClkSourceMclk        = 0U, /*!< sysclk source from external MCLK */
    kTLV320AIC3110_SysClkSourceInternalPLL = 1U, /*!< sysclk source from internal PLL */
} tlv320aic3110_sysclk_source_t;

/*! @brief tlv320aic3110 audio format */
typedef struct _tlv320aic3110_audio_format
{
    uint32_t mclk_HZ;    /*!< master clock frequency */
    uint32_t sampleRate; /*!< sample rate */
    uint32_t bitWidth;   /*!< bit width */
} tlv320aic3110_audio_format_t;

/*! @brief tlv320aic3110 master system clock configuration */
typedef struct _tlv320aic3110_master_sysclk_config
{
    tlv320aic3110_sysclk_source_t sysclkSource; /*!< sysclk source */
    uint32_t sysclkFreq;                 /*!< PLL output frequency value */
} tlv320aic3110_master_sysclk_config_t;

/*! @brief Initialize structure of TLV320AIC3110 */
typedef struct tlv320aic3110_config
{
    tlv320aic3110_route_t route;                      /*!< Audio data route.*/
    tlv320aic3110_protocol_t bus;                     /*!< Audio transfer protocol */
    tlv320aic3110_audio_format_t format;              /*!< Audio format */
    bool master_slave;                         /*!< Master or slave. */
    tlv320aic3110_master_sysclk_config_t masterClock; /*!< master clock configurations */
    bool enableSpeaker;                        /*!< True means enable class D speaker as output, false means no */
    tlv320aic3110_input_t leftInputSource;            /*!< Left input source for TLV320AIC3110 */
    tlv320aic3110_input_t rightInputSource;           /*!< Right input source for tlv320aic3110 */
    tlv320aic3110_play_source_t playSource;           /*!< play source */
    uint8_t slaveAddress;                      /*!< tlv320aic3110 device address */
    codec_i2c_config_t i2cConfig;              /*!< i2c configuration */
} tlv320aic3110_config_t;

/*! @brief tlv320aic3110 codec handler
 */
typedef struct _tlv320aic3110_handle
{
    const tlv320aic3110_config_t *config;              /*!< wm8904 config pointer */
    uint8_t i2cHandle[TLV320AIC3110_I2C_HANDLER_SIZE]; /*!< i2c handle */
} tlv320aic3110_handle_t;
/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief TLV320AIC3110 initialize function.
 *
 * The second parameter is NULL to TLV320AIC3110 in this version. If users want
 * to change the settings, they have to use tlv320aic3110_write_reg() or tlv320aic3110_modify_reg()
 * to set the register value of TLV320AIC3110.
 * Note: If the codec_config is NULL, it would initialize TLV320AIC3110 using default settings.
 * The default setting:
 * codec_config->route = kTLV320AIC3110_RoutePlaybackandRecord
 * codec_config->bus = kTLV320AIC3110_BusI2S
 * codec_config->master = slave
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param config TLV320AIC3110 configuration structure.
 */
status_t TLV320AIC3110_Init(tlv320aic3110_handle_t *handle, const tlv320aic3110_config_t *config);

/*!
 * @brief Deinit the TLV320AIC3110 codec.
 *
 * This function close all modules in TLV320AIC3110 to save power.
 *
 * @param handle TLV320AIC3110 handle structure pointer.
 */
status_t TLV320AIC3110_Deinit(tlv320aic3110_handle_t *handle);

/*!
 * @brief Set audio data route in TLV320AIC3110.
 *
 * This function would set the data route according to route. The route cannot be combined,
 * as all route would enable different modules.
 * Note: If a new route is set, the previous route would not work.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param route Audio data route in TLV320AIC3110.
 */
status_t TLV320AIC3110_SetDataRoute(tlv320aic3110_handle_t *handle, tlv320aic3110_route_t route);

/*!
 * @brief Set left audio input source in TLV320AIC3110.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param input Audio input source.
 */
status_t TLV320AIC3110_SetLeftInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input);

/*!
 * @brief Set right audio input source in TLV320AIC3110.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param input Audio input source.
 */
status_t TLV320AIC3110_SetRightInput(tlv320aic3110_handle_t *handle, tlv320aic3110_input_t input);

/*!
 * @brief Set the audio transfer protocol.
 *
 * TLV320AIC3110 only supports I2S, left justified, right justified, PCM A, PCM B format.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param protocol Audio data transfer protocol.
 */
status_t TLV320AIC3110_SetProtocol(tlv320aic3110_handle_t *handle, tlv320aic3110_protocol_t protocol);

/*!
 * @brief Set TLV320AIC3110 as master or slave.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param master 1 represent master, 0 represent slave.
 */
void TLV320AIC3110_SetMasterSlave(tlv320aic3110_handle_t *handle, bool master);

/*!
 * @brief Set the volume of different modules in TLV320AIC3110.
 *
 * This function would set the volume of TLV320AIC3110 modules. Uses need to appoint the module.
 * The function assume that left channel and right channel has the same volume.
 *
 * Module:kTLV320AIC3110_ModuleADC, volume range value: 0 is mute, 1-255 is -97db to 30db
 * Module:kTLV320AIC3110_ModuleDAC, volume range value: 0 is mute, 1-255 is -127db to 0db
 * Module:kTLV320AIC3110_ModuleHP, volume range value: 0 - 2F is mute, 0x30 - 0x7F is -73db to 6db
 * Module:kTLV320AIC3110_ModuleLineIn, volume range value: 0 - 0x3F is -17.25db to 30db
 * Module:kTLV320AIC3110_ModuleSpeaker, volume range value: 0 - 2F is mute, 0x30 - 0x7F is -73db to 6db
 *
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param module Module to set volume, it can be ADC, DAC, Headphone and so on.
 * @param volume Volume value need to be set.
 */
status_t TLV320AIC3110_SetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, uint32_t volume);

/*!
 * @brief Get the volume of different modules in TLV320AIC3110.
 *
 * This function gets the volume of TLV320AIC3110 modules. Uses need to appoint the module.
 * The function assume that left channel and right channel has the same volume.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param module Module to set volume, it can be ADC, DAC, Headphone and so on.
 * @return Volume value of the module.
 */
uint32_t TLV320AIC3110_GetVolume(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module);

/*!
 * @brief Mute modules in TLV320AIC3110.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param module Modules need to be mute.
 * @param isEnabled Mute or unmute, 1 represent mute.
 */
status_t TLV320AIC3110_SetMute(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled);

/*!
 * @brief Enable/disable expected devices.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param module Module expected to enable.
 * @param isEnabled Enable or disable moudles.
 */
status_t TLV320AIC3110_SetModule(tlv320aic3110_handle_t *handle, tlv320aic3110_module_t module, bool isEnabled);

/*!
 * @brief SET the TLV320AIC3110 play source.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param playSource play source , can be a value combine of kTLV320AIC3110_ModuleHeadphoneSourcePGA,
 * kTLV320AIC3110_ModuleHeadphoneSourceDAC, kTLV320AIC3110_ModulePlaySourceInput, kTLV320AIC3110_ModulePlayMonoRight,
 * kTLV320AIC3110_ModulePlayMonoLeft.
 *
 * @return kStatus_WM8904_Success if successful, different code otherwise..
 */
status_t TLV320AIC3110_SetPlay(tlv320aic3110_handle_t *handle, uint32_t playSource);

/*!
 * @brief Configure the data format of audio data.
 *
 * This function would configure the registers about the sample rate, bit depths.
 *
 * @param handle TLV320AIC3110 handle structure pointer.
 * @param sysclk system clock of the codec which can be generated by MCLK or PLL output.
 * @param sample_rate Sample rate of audio file running in TLV320AIC3110. TLV320AIC3110 now
 * supports 8k, 11.025k, 12k, 16k, 22.05k, 24k, 32k, 44.1k, 48k and 96k sample rate.
 * @param bits Bit depth of audio file (TLV320AIC3110 only supports 16bit, 20bit, 24bit
 * and 32 bit in HW).
 */
status_t TLV320AIC3110_ConfigDataFormat(tlv320aic3110_handle_t *handle, uint32_t sysclk, uint32_t sample_rate, uint32_t bits);

/*!
 * @brief Enable/disable jack detect feature.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param isEnabled Enable or disable moudles.
 */
status_t TLV320AIC3110_SetJackDetect(tlv320aic3110_handle_t *handle, bool isEnabled);

/*!
 * @brief Write register to TLV320AIC3110 using I2C.
 *
 * @param handle TLV320AIC3110 handle structure.
 * @param reg The register address in TLV320AIC3110.
 * @param val Value needs to write into the register.
 */
status_t TLV320AIC3110_WriteReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t val);

/*!
 * @brief Read register from TLV320AIC3110 using I2C.
 * @param reg The register address in TLV320AIC3110.
 * @param val Value written to.
 */
status_t TLV320AIC3110_ReadReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t *val);

/*!
 * @brief Modify some bits in the register using I2C.
 * @param handle TLV320AIC3110 handle structure.
 * @param reg The register address in TLV320AIC3110.
 * @param mask The mask code for the bits want to write. The bit you want to write should be 0.
 * @param val Value needs to write into the register.
 */
status_t TLV320AIC3110_ModifyReg(tlv320aic3110_handle_t *handle, uint8_t reg, uint16_t mask, uint16_t val);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_TLV320AIC3110_H_ */

/*******************************************************************************
 * API
 ******************************************************************************/
