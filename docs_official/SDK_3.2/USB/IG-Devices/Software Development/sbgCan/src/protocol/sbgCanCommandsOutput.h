/*!
 *	\file		sbgCanCommandsOutput.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		16/06/10
 *
 *	\brief		Output configuration commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_OUTPUT_H__
#define __SBG_CAN_COMMANDS_OUTPUT_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//  Trigger mask definition                                             //
//----------------------------------------------------------------------//
#define SBG_CAN_TRIGGER_DISABLED					(0x0000)			/*!< Helper used to disable an output using the trigger mechanism. */
#define SBG_CAN_TRIGGER_MAIN_LOOP_DIVIDER			(0x0001)			/*!< Enable trigger on the main loop frequency divider */
#define SBG_CAN_TRIGGER_MAGNETOMETERS				(0x0002)			/*!< Enable trigger on a new magnetometers data */
#define SBG_CAN_TRIGGER_BAROMETER					(0x0004)			/*!< Enable trigger on a new barometer data */
#define SBG_CAN_TRIGGER_GPS_VELOCITY				(0x0008)			/*!< Enable a trigger on a new GPS velocity */
#define SBG_CAN_TRIGGER_GPS_POSITION				(0x0010)			/*!< Enable a trigger on a new GPS position */
#define SBG_CAN_TRIGGER_GPS_COURSE					(0x0020)			/*!< Enable a trigger on a new GPS course (heading) */
#define SBG_CAN_TRIGGER_TIME_PULSE					(0x0040)			/*!< Enable a trigger on an input time pulse */
#define SBG_CAN_TRIGGER_EXT_EVENT					(0x0080)			/*!< Enable a trigger on a new external event */
#define SBG_CAN_TRIGGER_ODO_VELOCITY_0				(0x0100)			/*!< Enable a trigger when a new odometer velocity (channel 0) event occurs*/
#define SBG_CAN_TRIGGER_ODO_VELOCITY_1				(0x0200)			/*!< Enable a trigger when a new odometer velocity (channel 1) event occurs */
#define SBG_CAN_TRIGGER_EXT_TRUE_HEADING			(0x0400)			/*!< Enable a trigger when a new true heading information is available */
#define SBG_CAN_TRIGGER_VIRTUAL_ODOMETER			(0x0800)			/*!< Enable a trigger when the virtual odometer reached the desired distance */

//----------------------------------------------------------------------//
//- Outputs configuration commands                                     -//
//----------------------------------------------------------------------//

/*!
 *	Configures for a specific device output its triggers.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultId						The requested device output default id.
 *	\param[in]	trigger							The configuration of the output trigger.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetOutputConf(SbgCanDeviceHandle deviceHandle, uint16 defaultId, uint16 trigger);

/*!
 *	Returns for a specific device output its triggers.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultId						The requested device output default id.
 *	\param[out] pTrigger						Pointer to the configuration of the output trigger.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOutputConf(SbgCanDeviceHandle deviceHandle, uint16 defaultId, uint16 *pTrigger);

/*!
 *	Configures the device main loop frequency divider.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	divider							The new frequency divider.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetFrequencyDivider(SbgCanDeviceHandle deviceHandle, uint8 divider);

/*!
 *	Retrieves the device main loop frequency divider.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDivider						Pointer to the frequency divider.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetFrequencyDivider(SbgCanDeviceHandle deviceHandle, uint8 *pDivider);

/*!
 *	Retrieves a specific output for the device and return the corresponding CAN message.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	outputId						The requested device output default id.
 *	\param[out] pOutput							Pointer to a structure that hold the output data and default id.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetSpecificOutput(SbgCanDeviceHandle deviceHandle, SbgCanId outputId, SbgCanOutputDataStr *pOutput);

#endif	// __SBG_CAN_COMMANDS_OUTPUT_H__