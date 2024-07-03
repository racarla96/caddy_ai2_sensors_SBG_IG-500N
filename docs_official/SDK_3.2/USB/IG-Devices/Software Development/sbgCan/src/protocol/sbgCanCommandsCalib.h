/*!
 *	\file		sbgCanCommandsCalib.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		20/07/10
 *
 *	\brief		Calibration commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_CALIB_H__
#define __SBG_CAN_COMMANDS_CALIB_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//  Calibration commands arguments                                      //
//----------------------------------------------------------------------//


/*!
 *	For the magnetometers calibration command, defines which action to execute.
 */
typedef enum _SbgCanCalibMagsAction
{
	SBG_CAN_CALIB_MAGS_LOAD_DEFAULT			= 0x00,		/*!< Load into volatile memory the default magnetometers calibration data. */
	SBG_CAN_CALIB_MAGS_SAVE					= 0x05,		/*!< Save the current magnetometers calibration data into FLASH memory. */
	SBG_CAN_CALIB_MAGS_WRONG_PARAMETER		= 0x09,		/*!< Action returned by the device when the input parameter is invalid. */
} SbgCanCalibMagsAction;

/*!
 *	For the gyroscopes calibration command, defined which action to execute
 */
typedef enum _SbgCanCalibGyrosAction
{
	SBG_CAN_CALIB_GYROS_LOAD_DEFAULT		= 0x00,		/*!< Load into volatile memory the default gyroscopes bias data. */
	SBG_CAN_CALIB_GYROS_MEASURE_COARSE		= 0x04,		/*!< Calibrate the zero bias for gyroscopes during 250 ms */
	SBG_CAN_CALIB_GYROS_SAVE				= 0x05,		/*!< Save the current gyroscopes bias data into FLASH memory. */
	SBG_CAN_CALIB_GYROS_MEASURE_MEDIUM		= 0x06,		/*!< Calibrate the zero bias for gyroscopes during 1000 ms */
	SBG_CAN_CALIB_GYROS_MEASURE_FINE		= 0x07,		/*!< Calibrate the zero bias for gyroscopes during 3000 ms */
	SBG_CAN_CALIB_GYROS_WRONG_PARAMETER		= 0x09,		/*!< Action returned by the device when the input parameter is invalid. */
} SbgCanCalibGyrosAction;


//----------------------------------------------------------------------//
//- Calibration commands operations                                    -//
//----------------------------------------------------------------------//

/*!
 * Initiate or terminate the magnetometers calibration procedure.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	argument						Pointer to an int8 used to hold the calibration argument.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanCalibMagProcedure(SbgCanDeviceHandle deviceHandle, SbgCanCalibMagsAction argument);

/*!
 * Set a manual calibration data for magnetometers.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	bias							The magnetometers offset vector.
 * \param[in]	agm								The magnetometers Alignment and Gain Matrix.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetCalibMagManual(SbgCanDeviceHandle deviceHandle, const float bias[3], const float agm[9]);

/*!
 * Retrieve a manual calibration data for magnetometers.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[out]	bias							Float array used to hold the offset vector.
 * \param[out]	agm								Float array used to hold the Alignment and Gain Matrix.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetCalibMagManual(SbgCanDeviceHandle deviceHandle, float bias[3], float agm[9]);

/*!
 * Read the current gyro value to evaluate the gyro bias
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	argument						The argument for gyro calibration.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanCalibGyroBias(SbgCanDeviceHandle deviceHandle, SbgCanCalibGyrosAction argument);

#endif	// __SBG_CAN_COMMANDS_CALIB_H__

