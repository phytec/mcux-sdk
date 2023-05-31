/*
 * Copyright  2021 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_CODEC_TLV320AIC3110_ADAPTER_H_
#define _FSL_CODEC_TLV320AIC3110_ADAPTER_H_

#include "fsl_tlv320aic3110.h"

/*!
 * @addtogroup tlv320aic3110_adapter
 * @ingroup codec_common
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!@brief codec handler size */
#define HAL_CODEC_TLV320AIC3110_HANDLER_SIZE (TLV320AIC3110_I2C_HANDLER_SIZE + 4)
/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif
/*!
 * @brief Codec initilization.
 *
 * @param handle codec handle.
 * @param config codec configuration.
 * @return kStatus_Success is success, else initial failed.
 */
status_t HAL_CODEC_TLV320AIC3110_Init(void *handle, void *config);

/*!
 * @brief Codec de-initilization.
 *
 * @param handle codec handle.
 * @return kStatus_Success is success, else de-initial failed.
 */
status_t HAL_CODEC_TLV320AIC3110_Deinit(void *handle);

/*!
 * @brief set audio data format.
 *
 * @param handle codec handle.
 * @param mclk master clock frequency in HZ.
 * @param sampleRate sample rate in HZ.
 * @param bitWidth bit width.
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetFormat(void *handle, uint32_t mclk, uint32_t sampleRate, uint32_t bitWidth);

/*!
 * @brief set audio codec module volume.
 *
 * @param handle codec handle.
 * @param playChannel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * @param volume volume value, support 0 ~ 100, 0 is mute, 100 is the maximum volume value.
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetVolume(void *handle, uint32_t playChannel, uint32_t volume);

/*!
 * @brief set audio codec module mute.
 *
 * @param handle codec handle.
 * @param playChannel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * @param isMute true is mute, false is unmute.
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetMute(void *handle, uint32_t playChannel, bool isMute);

/*!
 * @brief set audio codec module power.
 *
 * @param handle codec handle.
 * @param module audio codec module.
 * @param powerOn true is power on, false is power down.
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetPower(void *handle, uint32_t module, bool powerOn);

/*!
 * @brief codec set record source.
 *
 * @param handle codec handle.
 * @param recordSource audio codec record source, can be a value or combine value of _codec_record_source.
 *
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetRecord(void *handle, uint32_t recordSource);

/*!
 * @brief codec set record channel.
 *
 * @param handle codec handle.
 * @param leftRecordChannel audio codec record channel, reference _codec_record_channel, can be a value or combine value
 of member in _codec_record_channel.
 * @param rightRecordChannel audio codec record channel, reference _codec_record_channel, can be a value combine of
 member in _codec_record_channel.

 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetRecordChannel(void *handle, uint32_t leftRecordChannel, uint32_t rightRecordChannel);

/*!
 * @brief codec set play source.
 *
 * @param handle codec handle.
 * @param playSource audio codec play source, can be a value or combine value of _codec_play_source.
 *
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_SetPlay(void *handle, uint32_t playSource);

/*!
 * @brief codec module control.
 *
 * This function is used for codec module control, support switch digital interface cmd, can be expand to support codec
 * module specific feature
 *
 * @param handle codec handle.
 * @param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * @param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * @return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_ModuleControl(void *handle, uint32_t cmd, uint32_t data);

/*!
 * brief codec module write reg.
 *
 * param handle codec handle.
 * param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_WriteReg(void *handle, uint8_t reg, uint16_t val);

/*!
 * brief codec module read reg.
 *
 * param handle codec handle.
 * param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_ReadReg(void *handle, uint8_t reg, uint16_t *val);

/*!
 * brief codec module modify reg.
 *
 * param handle codec handle.
 * param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * return kStatus_Success is success, else configure failed.
 */
status_t HAL_CODEC_TLV320AIC3110_ModifyReg(void *handle, uint8_t reg, uint16_t mask, uint16_t val);


#if !(defined CODEC_MULTI_ADAPTERS && CODEC_MULTI_ADAPTERS)
/*!
 * @brief Codec initilization.
 *
 * @param handle codec handle.
 * @param config codec configuration.
 * @return kStatus_Success is success, else initial failed.
 */
static inline status_t HAL_CODEC_Init(void *handle, void *config)
{
    return HAL_CODEC_TLV320AIC3110_Init(handle, config);
}

/*!
 * @brief Codec de-initilization.
 *
 * @param handle codec handle.
 * @return kStatus_Success is success, else de-initial failed.
 */
static inline status_t HAL_CODEC_Deinit(void *handle)
{
    return HAL_CODEC_TLV320AIC3110_Deinit(handle);
}

/*!
 * @brief set audio data format.
 *
 * @param handle codec handle.
 * @param mclk master clock frequency in HZ.
 * @param sampleRate sample rate in HZ.
 * @param bitWidth bit width.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetFormat(void *handle, uint32_t mclk, uint32_t sampleRate, uint32_t bitWidth)
{
    return HAL_CODEC_TLV320AIC3110_SetFormat(handle, mclk, sampleRate, bitWidth);
}

/*!
 * @brief set audio codec module volume.
 *
 * @param handle codec handle.
 * @param playChannel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * @param volume volume value, support 0 ~ 100, 0 is mute, 100 is the maximum volume value.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetVolume(void *handle, uint32_t playChannel, uint32_t volume)
{
    return HAL_CODEC_TLV320AIC3110_SetVolume(handle, playChannel, volume);
}

/*!
 * @brief set audio codec module mute.
 *
 * @param handle codec handle.
 * @param playChannel audio codec play channel, can be a value or combine value of _codec_play_channel.
 * @param isMute true is mute, false is unmute.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetMute(void *handle, uint32_t playChannel, bool isMute)
{
    return HAL_CODEC_TLV320AIC3110_SetMute(handle, playChannel, isMute);
}

/*!
 * @brief set audio codec module power.
 *
 * @param handle codec handle.
 * @param module audio codec module.
 * @param powerOn true is power on, false is power down.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetPower(void *handle, uint32_t module, bool powerOn)
{
    return HAL_CODEC_TLV320AIC3110_SetPower(handle, module, powerOn);
}

/*!
 * @brief codec set record source.
 *
 * @param handle codec handle.
 * @param recordSource audio codec record source, can be a value or combine value of _codec_record_source.
 *
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetRecord(void *handle, uint32_t recordSource)
{
    return HAL_CODEC_TLV320AIC3110_SetRecord(handle, recordSource);
}

/*!
 * @brief codec set record channel.
 *
 * @param handle codec handle.
 * @param leftRecordChannel audio codec record channel, reference _codec_record_channel, can be a value or combine value
 of member in _codec_record_channel.
 * @param rightRecordChannel audio codec record channel, reference _codec_record_channel, can be a value combine of
 member in _codec_record_channel.

 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetRecordChannel(void *handle, uint32_t leftRecordChannel, uint32_t rightRecordChannel)
{
    return HAL_CODEC_TLV320AIC3110_SetRecordChannel(handle, leftRecordChannel, rightRecordChannel);
}

/*!
 * @brief codec set play source.
 *
 * @param handle codec handle.
 * @param playSource audio codec play source, can be a value or combine value of _codec_play_source.
 *
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_SetPlay(void *handle, uint32_t playSource)
{
    return HAL_CODEC_TLV320AIC3110_SetPlay(handle, playSource);
}

/*!
 * @brief codec module control.
 *
 * This function is used for codec module control, support switch digital interface cmd, can be expand to support codec
 * module specific feature
 *
 * @param handle codec handle.
 * @param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * @param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_ModuleControl(void *handle, uint32_t cmd, uint32_t data)
{
    return HAL_CODEC_TLV320AIC3110_ModuleControl(handle, cmd, data);
}

/*!
 * @brief codec module write reg.
 *
 * This function is used for codec module write reg, support switch digital interface cmd, can be expand to support codec
 * module specific feature
 *
 * @param handle codec handle.
 * @param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * @param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_WriteReg(void *handle, uint8_t reg, uint16_t val)
{
    return HAL_CODEC_TLV320AIC3110_WriteReg(handle, reg, val);
}

/*!
 * @brief codec module read reg.
 *
 * This function is used for codec module read reg, support switch digital interface cmd, can be expand to support codec
 * module specific feature
 *
 * @param handle codec handle.
 * @param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * @param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_ReadReg(void *handle, uint8_t reg, uint16_t *val)
{
    return HAL_CODEC_TLV320AIC3110_ReadReg(handle, reg, val);
}

/*!
 * @brief codec module write modify.
 *
 * This function is used for codec module modify reg, support switch digital interface cmd, can be expand to support codec
 * module specific feature
 *
 * @param handle codec handle.
 * @param cmd module control cmd, reference _codec_module_ctrl_cmd.
 * @param data value to write, when cmd is kCODEC_ModuleRecordSourceChannel, the data should be a value combine
 *  of channel and source, please reference macro CODEC_MODULE_RECORD_SOURCE_CHANNEL(source, LP, LN, RP, RN), reference
 *  codec specific driver for detail configurations.
 * @return kStatus_Success is success, else configure failed.
 */
static inline status_t HAL_CODEC_ModifyReg(void *handle, uint8_t reg, uint16_t mask, uint16_t val)
{
    return HAL_CODEC_TLV320AIC3110_ModifyReg(handle, reg, mask, val);
}

#endif

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_WM8904_ADAPTER_H_ */
