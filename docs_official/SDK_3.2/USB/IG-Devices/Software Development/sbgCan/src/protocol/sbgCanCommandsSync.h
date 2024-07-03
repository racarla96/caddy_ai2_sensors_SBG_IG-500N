/*!
 *	\file		sbgCanCommandsSync.h
 *  \author		SBG-Systems (Alexis Guinamard)
 *	\date		08/12/10
 *
 *	\brief		Commands used to configure the IG-500 Synchronization features<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_SYNC_H__
#define __SBG_CAN_COMMANDS_SYNC_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//- Logic inputs defines                                               -//
//----------------------------------------------------------------------//

/*! Bit mask for isolating the sensitivity in the sync in options param ( rising/falling edge) */
#define SBG_CAN_IN_SENSE_MASK			(0x03)

//----------------------------------------------------------------------//
//- Logic inputs types definition                                      -//
//----------------------------------------------------------------------//

/*!
 * Logic and synchronization inputs types
 */
typedef enum _SbgCanLogicInType
{
	SBG_CAN_IN_DISABLED				= 0x00,			/*!< Input channel disabled */
	SBG_CAN_IN_EVENT				= 0x01,			/*!< General purpose event trigger */
	SBG_CAN_IN_TIME_PULSE			= 0x03,			/*!< GPS 4 PPS time input */
	SBG_CAN_IN_ODOMETER				= 0x05,			/*!< Odometer input */
	SBG_CAN_IN_ODOMETER_DIRECTION	= 0x06			/*!< Direction sense for the other odometer channel. <br>
													 *   Compatible with quadrature output encoders (signal B) */
} SbgCanLogicInType;

/*!
 * Logic input sensitivity types
 */
typedef enum _SbgCanLogicInSensitivity
{
	SBG_CAN_IN_FALLING_EDGE			= 0x00,			/*!< The trigger will be activated by a falling edge */
	SBG_CAN_IN_RISING_EDGE			= 0x01,			/*!< The trigger will be activated by a rising edge */
	SBG_CAN_IN_LEVEL_CHANGE			= 0x02			/*!< The trigger is activated by a level change (rising or falling edge) */
} SbgCanLogicInSensitivity;

/*!
 * Logic input channel 0 physical location on IG-500E
 */
typedef enum _SbgCanLogicInLocation
{
	SBG_CAN_IN_STD_LOCATION			= 0x00,			/*!< Sync In ch0 is located on the main connector */
	SBG_CAN_IN_EXT_LOCATION			= 0x04			/*!< Sync In ch0 is located on the extended connector */
} SbgCanLogicInLocation;

/*!
 * Logic output and synchronization  types
 */
typedef enum _SbgCanLogicOutType
{
	SBG_CAN_OUT_DISABLED			= 0x00,			/*!< Input channel disabled */
	SBG_CAN_OUT_MAIN_LOOP_START		= 0x01,			/*!< Main loop starting trigger */
	SBG_CAN_OUT_MAIN_LOOP_DIVIDER	= 0x02,			/*!< Trigger activated at the beginning of each main loop where a continuous output is generated */
	SBG_CAN_OUT_TIME_PULSE_COPY		= 0x03,			/*!< Copy of the GPS time pulse input trigger */
	SBG_CAN_OUT_VIRTUAL_ODO			= 0x05			/*!< Virtual odometer logic output: Enabled each x meters of travel */
} SbgCanLogicOutType;

/*!
 * Logic output polarity
 */
typedef enum _SbgCanLogicOutPolarity
{
	SBG_CAN_OUT_FALLING_EDGE		= 0x00,			/*!< The output pin will generate a falling edge */
	SBG_CAN_OUT_RISING_EDGE			= 0x01,			/*!< The output pin will generate a rising edge */
	SBG_CAN_OUT_TOGGLE				= 0x02			/*!< The pulse is a level change */
} SbgCanLogicOutPolarity;

//----------------------------------------------------------------------//
//- Logic inputs operations                                            -//
//----------------------------------------------------------------------//

/*!
 *	Set a logic input channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[in]	inputType			Type of the logic input, may be <br>
 * 										- SBG_CAN_IN_DISABLED
 *										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *										- SBG_CAN_IN_TIME_PULSE
 * 										- SBG_CAN_IN_BARO
 * 										- SBG_CAN_IN_ODOMETER
 * \param[in]	sensitivity			Sensitivity of the trigger. It may be:<br>
 * 										- SBG_CAN_IN_FALLING_EDGE
 * 										- SBG_CAN_IN_RISING_EDGE
 * 										- SBG_CAN_IN_LEVEL_CHANGE
 * \param[in]	location			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
										- SBG_CAN_IN_STD_LOCATION (default value, leave to this value if not used)
										- SBG_CAN_IN_EXT_LOCATION
 * \param[in]	nsDelay				Delay to be added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are allowed:<br>
 *									This delay is only used when the input is set to:
 * 										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanSetLogicInChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicInType inputType, SbgCanLogicInSensitivity sensitivity,SbgCanLogicInLocation location, uint32 nsDelay);

/*!
 *	Get a logic input channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[out]	pInputType			Type of the logic input, may be <br>
 * 										- SBG_CAN_IN_DISABLED
 *										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *										- SBG_CAN_IN_TIME_PULSE
 * 										- SBG_CAN_IN_BARO
 * 										- SBG_CAN_IN_ODOMETER	
 * \param[out]	pSensitivity		Sensitivity of the trigger. It may be:<br>
 * 										- SBG_CAN_IN_FALLING_EDGE
 * 										- SBG_CAN_IN_RISING_EDGE
 * 										- SBG_CAN_IN_LEVEL_CHANGE
 * \param[out]	pLocation			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
										- SBG_CAN_IN_STD_LOCATION
										- SBG_CAN_IN_EXT_LOCATION
 * \param[out]	pNsDelay			Delay added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are possible:<:<br>
 *									This delay is only valid when the input is set to:
 * 										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanGetLogicInChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicInType *pInputType, SbgCanLogicInSensitivity *pSensitivity, SbgCanLogicInLocation *pLocation, uint32 *pNsDelay);


/*!
 *	Set a logic output channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. Set always 0
 *	\param[in]	outputType			Type of the logic output, may be <br>
 * 										- SBG_CAN_OUT_DISABLED
 *										- SBG_CAN_OUT_MAIN_LOOP_START
 *										- SBG_CAN_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_CAN_OUT_TIME_PULSE_COPY
 *										- SBG_CAN_OUT_EVENT_COPY
 * \param[in]	polarity			Polarity of the out pulse. It may be:
 * 										- SBG_CAN_OUT_FALLING_EDGE
 * 										- SBG_CAN_OUT_RISING_EDGE
 * 										- SBG_CAN_OUT_TOGGLE
 *  \param[in]	duration			When the polarity is set to SBG_CAN_OUT_FALLING_EDGE or SBG_CAN_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *									Leave to 0 if not used.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanSetLogicOutChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicOutType outputType, SbgCanLogicOutPolarity polarity, uint8 duration);

/*!
 *	Get a logic output channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. Leave always to 0
 *	\param[out]	pOutputType			Type of the logic output, may be <br>
 * 										- SBG_CAN_OUT_DISABLED
 *										- SBG_CAN_OUT_MAIN_LOOP_START
 *										- SBG_CAN_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_CAN_OUT_TIME_PULSE_COPY
 *										- SBG_CAN_OUT_EVENT_COPY
 * \param[out]	pPolarity			Polarity of the out pulse. It may be:
 * 										- SBG_CAN_OUT_FALLING_EDGE
 * 										- SBG_CAN_OUT_RISING_EDGE
 * 										- SBG_CAN_OUT_TOGGLE
 *  \param[out]	pDuration			When the polarity is set to SBG_CAN_OUT_FALLING_EDGE or SBG_CAN_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanGetLogicOutChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicOutType *pOutputType, SbgCanLogicOutPolarity *pPolarity, uint8 *pDuration);


/*!
 *	Defines the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	distance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetVirtualOdoConf(SbgCanDeviceHandle deviceHandle, float distance);

/*!
 *	Retrieves the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[out]	pDistance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetVirtualOdoConf(SbgCanDeviceHandle deviceHandle, float *pDistance);

#endif
