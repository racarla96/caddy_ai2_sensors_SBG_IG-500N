/*!
 *	\file		sbgCanCommandsOdo.h
 *  \author		SBG-Systems (Raphaël Siryani)
 *	\date		08/12/2010
 *
 *	\brief		Odometer parameters on IG-500E.<br>
 *				Copyright 2010 SBG Systems. All rights reserved.
 */

#ifndef __SBG_CAN_COMMANDS_ODO_H__
#define __SBG_CAN_COMMANDS_ODO_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"


//----------------------------------------------------------------------//
//- Odometer definitions                                               -//
//----------------------------------------------------------------------//

#define SBG_CAN_ODO_AUTO_GPS_GAIN		(0x80)		/*!< If set in the GainOpts field, the GPS will automatically enhance odometer's gain */

//----------------------------------------------------------------------//
//- Odometer Types definition                                          -//   
//----------------------------------------------------------------------//

/*!
 *  This enum defines sensitive axis of the corresponding odometer channel
 */
typedef enum _SbgCanOdoAxis
{
	SBG_CAN_ODO_X				= 0x00,				/*!< Odometer sensitive axis is X */
	SBG_CAN_ODO_Y				= 0x01,				/*!< Odometer sensitive axis is Y */
	SBG_CAN_ODO_Z				= 0x02,				/*!< Odometer sensitive axis is Z */
} SbgCanOdoAxis;

/*!
 *  This enum defines the direction of the corresponding odometer channel
 */
typedef enum _SbgCanOdoDirection
{
	SBG_CAN_ODO_DIR_POSITIVE	=	0x00,		/*!< Odometer velocity is always positive */
	SBG_CAN_ODO_DIR_NEGATIVE	=	0x01,		/*!< Odometer velocity is always negative */
} SbgCanOdoDirection;

//----------------------------------------------------------------------//
//- Odometer commands                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Set the main configuration of the external odometer channels
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	axis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_CAN_ODO_X
 *										- SBG_CAN_ODO_Y
 *										- SBG_CAN_ODO_Z
 *  \param[in]	pulsePerMeter		decimal number of pulses per meter
 *  \param[in]	gainError			Error in percent on the previous gain value
 *  \param[in]	gpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain 
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoConfig(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoAxis axis, float pulsesPerMeter, uint8 gainError, bool gpsGainCorrection);

/*!
 *	Get the main configuration of the external odometer channels
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pAxis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_CAN_ODO_X
 *										- SBG_CAN_ODO_Y
 *										- SBG_CAN_ODO_Z
 *  \param[out]	pPulsePerMeter		decimal number of pulses per meter
 *  \param[out]	pGainError			Error in percent on the previous gain value
 *	\param[out]	pGpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoConfig(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoAxis *pAxis, float *pPulsesPerMeter, uint8 *pGainError, bool *pGpsGainCorrection);

/*!
 *  Configures the odometer direction for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoDirection		Direction of the odometer. May be:
 *									- SBG_CAN_ODO_DIR_POSITIVE
 *									- SBG_CAN_ODO_DIR_NEGATIVE
 *									- SBG_CAN_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoDirection(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoDirection odoDirection);

/*!
 *  Get the odometer direction for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pOdoDirection		Direction of the odometer. May be:
 *									- SBG_CAN_ODO_DIR_POSITIVE
 *									- SBG_CAN_ODO_DIR_NEGATIVE
 *									- SBG_CAN_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoDirection(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoDirection *pOdoDirection);

/*!
 *  Configures the odometer lever arm for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoLeverArm			X,Y,Z vector representing the distance between the device and the odometer.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoLeverArm(SbgCanDeviceHandle deviceHandle, uint8 channel, const float odoLeverArm[3]);

/*!
 *  Get the odometer lever arm for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	odoLeverArm			X,Y,Z vector representing the distance between the device and the odometer.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoLeverArm(SbgCanDeviceHandle deviceHandle, uint8 channel, float odoLeverArm[3]);

#endif