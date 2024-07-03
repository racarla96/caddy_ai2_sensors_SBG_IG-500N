/*!
 *	\file		binaryLogEkf.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		25 February 2013
 *
 *	\brief		This file is used to parse received EKF compued data binary logs.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */
#ifndef __BINARY_LOG_EKF_H__
#define __BINARY_LOG_EKF_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 * EKF computed orientation using euler angles.
 */
typedef struct _SbgLogEkfEulerData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	float	euler[3];				/*!< Roll, Pitch and Yaw angles in rad. */
	float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
} SbgLogEkfEulerData;

/*!
 * EFK computed orientation using quaternion.
 */
typedef struct _SbgLogEkfQuatData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	float	quaternion[4];			/*!< Orientation quaternion stored in W, X, Y, Z form. */
	float	eulerStdDev[3];			/*!< Roll, Pitch and Yaw angles 1 sigma standard deviation in rad. */
} SbgLogEkfQuatData;

/*!
 * EFK computed navigation data.
 */
typedef struct _SbgLogEkfNavData
{
	uint32	timeStamp;				/*!< Time in us since the sensor power up. */
	float	velocity[3];			/*!< North, East, Down velocity in m.s^-1. */
	float	velocityStdDev[3];		/*!< North, East, Down velocity 1 sigma standard deviation in m.s^-1. */
	double	position[3];			/*!< Latitude, Longitude in degrees positive North and East.
										 Altitude above Mean Sea Level in meters. */
	float	undulation;				/*!< Altitude difference between the geoid and the Ellipsoid in meters. */
	float	positionStdDev[3];		/*!< Latitude, longitude and altitude 1 sigma standard deviation in meters. */
} SbgLogEkfNavData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_EKF_EULER message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEkfEulerData(const void *pPayload, uint32 payloadSize, SbgLogEkfEulerData *pOutputData);

/*!
 *	Parse data for the SBG_ECOM_LOG_EKF_QUAT message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEkfQuatData(const void *pPayload, uint32 payloadSize, SbgLogEkfQuatData *pOutputData);

/*!
 *	Parse data for the SBG_ECOM_LOG_EKF_NAV message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEkfNavData(const void *pPayload, uint32 payloadSize, SbgLogEkfNavData *pOutputData);

#endif
