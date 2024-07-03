/*!
 *	\file		sbgCanCommandsIg30.h
 *  \author		SBG Systems (Alexis Guinamard)
 *	\date		10/04/2012
 *
 *	\brief		IG-30 and IG-20 specific commands
 *
 *	\section CodeCopyright Copyright Notice
 *	Copyright (C) 2007-2012, SBG Systems SAS. All rights reserved.
 *
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 */
#ifndef __SBG_CAN_COMMANDS_IG_30_
#define __SBG_CAN_COMMANDS_IG_30_

#include "../sbgCommon.h"
#include "sbgCanDevice.h"
//----------------------------------------------------------------------//
// Masks definitons about kalman filter                                 //
//----------------------------------------------------------------------//

#define SBG_CAN_FILTER_OPTION_ENABLE_ATTITUDE				(0x10)	/*!< Enable the attitude/navigation computation.<br>If disabled, the device can outputs calibrated data at 512 Hz. */

//----------------------------------------------------------------------//
//-  Definitions concerning GPS                                        -//
//----------------------------------------------------------------------//

#define SBG_CAN_GPS_DISABLE_SBAS					(0x00)			/*!< GPS option, disable SBAS and ranging */
#define SBG_CAN_GPS_ENABLE_SBAS_DIFF_CORRECTIONS	(0x01)			/*!< GPS option, enable SBAS corrections */
#define SBG_CAN_GPS_ENABLE_SBAS_RANGING				(0x02)			/*!< GPS option, enable ranging for SBAS */
#define SBG_CAN_GPS_ALTITUDE_ABOVE_MSL				(0x04)

/*!
 *	GPS dynamic platform model enumeration.
 */
typedef enum _SbgCanGpsDynamicModel
{
	SBG_CAN_GPS_MODEL_STATIONARY	= 1,		/*!< Stationary model, low bandwith */
	SBG_CAN_GPS_MODEL_PEDESTRIAN	= 2,		/*!< Pedestrian model, low dynamic, low bandwith */
	SBG_CAN_GPS_MODEL_AUTOMOTIVE	= 3,		/*!< Automotive model, mid dynamic, mid bandwith */
	SBG_CAN_GPS_MODEL_SEA			= 4,		/*!< Sea model, mid dynamic, mid bandwith */
	SBG_CAN_GPS_MODEL_AIRBONE_1G	= 5,		/*!< Airbone model with less than 1g accelerations, mid bandwith */
	SBG_CAN_GPS_MODEL_AIRBONE_2G	= 6,		/*!< Airbone model with less than 2g accelerations, high bandwith */
	SBG_CAN_GPS_MODEL_AIRBONE_4G	= 7			/*!< Airbone model with less than 4g accelerations, very high bandwith and recommanded for most applications */
} SbgCanGpsDynamicModel;

//----------------------------------------------------------------------//
//- Sensor sampling and orientation computation relative commands      -//   
//----------------------------------------------------------------------//

/*!
 *	Set the different cutoff frequencies.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	cutOffGyro						The gyro cut off frequencyin Hz.
 *	\param[in]	cutOffAccel						The accelero cut off frequency in Hz.
 *	\param[in]	cutOffMag						The magneto cut off frequency in Hz.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetFilterFrequencies(SbgCanDeviceHandle deviceHandle, float cutOffGyro, float cutOffAccel, float cutOffMag);

/*!
 *	Get the different cutoff frequencies.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pCutOffGyro						Pointer to an float used to hold the gyro cut off frequency in Hz.
 *	\param[out] pCutOffAccel					Pointer to an float used to hold the accelero cut off frequency in Hz.
 *	\param[out] pCutOffMag						Pointer to an float used to hold the magneto cut off frequency in Hz.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetFilterFrequencies(SbgCanDeviceHandle deviceHandle, float *pCutOffGyro, float *pCutOffAccel, float *pCutOffMag);

/*!
 *	Set the Kalman filter period and mode.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	kalPeriod						The Kalman period in 0.1 ms unit.
 *	\param[in]	kalOptions						The Kalman filter attitude mode.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetKalmanFilter(SbgCanDeviceHandle deviceHandle, uint16 kalPeriod, uint16 kalOptions);

/*!
 *	Get the Kalman filter period and mode.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pKalPeriod						Pointer to an uint16 used to hold the Kalman period in 0.1 ms unit.
 *	\param[out] pKalOptions						Pointer to an uint16 used to hold the Kalman filter attitude mode.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetKalmanFilter(SbgCanDeviceHandle deviceHandle, uint16 *pKalPeriod, uint16 *pKalOptions);

//----------------------------------------------------------------------//
//- GPS and altimeter relative commands                                -//   
//----------------------------------------------------------------------//

/*!
 *	Set the GPS different options.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	model							The gps model.
 *	\param[in]	options							The gps options.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetGpsOptions(SbgCanDeviceHandle deviceHandle, SbgCanGpsDynamicModel model, uint8 options);

/*!
 *	Get the GPS different options.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pModel							Pointer to an uint8 used to hold the gps model.
 *	\param[out]	pOptions						Pointer to an uint8 used to hold the gps options.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetGpsOptions(SbgCanDeviceHandle deviceHandle, SbgCanGpsDynamicModel *pModel, uint8 *pOptions);


#endif