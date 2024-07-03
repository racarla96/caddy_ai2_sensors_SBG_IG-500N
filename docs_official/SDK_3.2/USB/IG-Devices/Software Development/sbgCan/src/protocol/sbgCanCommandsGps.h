/*!
 *	\file		sbgCanCommandsGps.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		30/06/10
 *
 *	\brief		Kalman Filter commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_GPS_H__
#define __SBG_CAN_COMMANDS_GPS_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//-  Definitions concerning GPS                                        -//
//----------------------------------------------------------------------//

/*!
 *	GPS Space vehicles information struct.<br>
 *	Angles are in 32/45th of degrees.
 */
typedef struct _SbgCanGpsSVInfo
{
	uint8	satelliteId;					/*!< Id of the followed satellite  */
	uint8	flagsQuality;					/*!< flags and signal quality indicator (quality bits 7,6,5 flags 4,3,2,1,0) */
	uint8	signalStrength;					/*!< Carrier to noise Ratio */
	int8	azimuth;						/*!< Azimuth of the SV(signed) (1LSB = 32/45 degrees) */
	int8	elevation; 						/*!< Elevation of the SV(signed) (1LSB = 32/45 degrees) */
} SbgCanGpsSVInfo;

/*!
 *	Define the aiding source TO use in the Kalman Naviagation filter.
 */
typedef enum _SbgCanAidingNavSrc
{
	SBG_CAN_SRC_GPS				= 0x00,			/*!< We are using the GPS position/velocity as the aiding source for the navigation filter */
	SBG_CAN_SRC_GPS_AND_BARO	= 0x01,			/*!< We are using the GPS position/velocity and the barometric vertical speed as the aiding source for the navigation filter */
	SBG_CAN_SRC_EXTERNAL		= 0x02			/*!< We are using the an external source for position/velocity as the aiding source for the navigation filter */
} SbgCanAidingNavSrc;


//----------------------------------------------------------------------//
//- Gps and reference pressure configuration commands				   -//
//----------------------------------------------------------------------//


/*!
 *	Set the reference pressure for altitude calculation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	refPressure						The reference pressure.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetRefPressure(SbgCanDeviceHandle deviceHandle, uint32 refPressure);

/*!
 *	Get the reference pressure for altitude calculation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pRefPressure					Pointer to an uint32 used to hold the reference pressure.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetRefPressure(SbgCanDeviceHandle deviceHandle, uint32 *pRefPressure);

#endif	// __SBG_CAN_COMMANDS_GPS_H__
