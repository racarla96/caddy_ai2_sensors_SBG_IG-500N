/*!
 *	\file		sbgCanCommandsNav.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		30/06/10
 *
 *	\brief		Navigation commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_NAV_H__
#define __SBG_CAN_COMMANDS_NAV_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//- Definition concerning navigation                                   -//
//----------------------------------------------------------------------//

/*!
 *    Define the position aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgCanAidingPosSrc
{
    SBG_CAN_POS_SRC_GPS             = 0x00,			/*!< We are using the GPS position as the aiding source for the navigation filter */
    SBG_CAN_POS_SRC_GPS_AND_BARO    = 0x01,			/*!< We are using the GPS position and the barometric altitude as the aiding source for the navigation filter */
    SBG_CAN_POS_SRC_USER            = 0x02			/*!< We are using the user provided source for position as the aiding source for the navigation filter */
} SbgCanAidingPosSrc;

/*!
 *    Define the velocity aiding source to use in the Kalman Naviagation filter.
 */
typedef enum _SbgCanAidingVelSrc
{
    SBG_CAN_VEL_SRC_GPS             = 0x00,			/*!< We are using the GPS velocity as the aiding source for the navigation filter */
    SBG_CAN_VEL_SRC_USER            = 0x02,			/*!< We are using the a user provided source for velocity as the aiding source for the navigation filter */
    SBG_CAN_VEL_SRC_ODO             = 0x03,			/*! We are using the external odometer as velocity source for the navigation filter */
} SbgCanAidingVelSrc;

//----------------------------------------------------------------------//
//- Navigation configuration commands								   -//
//----------------------------------------------------------------------//

/*!
 *	Set the source for the velocity and position used by the navigation filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	positionSource					The postion source.
 *	\param[in]	velocitySource					The Velocity source.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetNavSources(SbgCanDeviceHandle deviceHandle, SbgCanAidingPosSrc positionSource, SbgCanAidingVelSrc velocitySource);

/*!
 *	Get the source for the velocity and position used by the navigation filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pPositionSource					Pointer to an enum used to hold the postion source.
 *	\param[out]	pVelocitySource					Pointer to an enum used to hold the velocity source.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetNavSources(SbgCanDeviceHandle deviceHandle, SbgCanAidingPosSrc *pPositionSource, SbgCanAidingVelSrc *pVelocitySource);

/*!
 *	Set the GPS lever arm vector from the device to the antenna. (IG-500N only)
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	gpsLeverArm						X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *												The distance is exressed in meters in the device coordinate system.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetGpsLeverArm(SbgCanDeviceHandle deviceHandle, const float gpsLeverArm[3]);
 
/*!
 *	Get the GPS lever arm vector from the device to the antenna. (IG-500N- only)
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[out]	gpsLeverArm						X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *												The distance is exressed in meters in the device coordinate system.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetGpsLeverArm(SbgCanDeviceHandle deviceHandle, float gpsLeverArm[3]);

/*!
 *	Set the local gravity magnitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	magnitude						The device local magnitude (in m.s-²).
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetGravityMagnitude(SbgCanDeviceHandle deviceHandle, float magnitude);

/*!
 *	Get the local gravity magnitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pMagnitude						Pointer to a float used to hold the device local magnitude.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetGravityMagnitude(SbgCanDeviceHandle deviceHandle, float *pMagnitude);

/*!
 *	Send the velocity aiding information used by the Navigation filter.<br>
 *	The velocity is expressed in meters per second in the device local coordinate system.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	velocity						The aiding velocity respectively along x, y and z axis in m/s in the device coordinate system.
 *	\param[in]	accuracy						The accuracy of the provided velocity in m/s.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterVelocityLocal(SbgCanDeviceHandle deviceHandle, const float velocity[3], float accuracy);

/*!
 *	Send the velocity aiding information used by the Navigation filter.<br>
 *	The velocity is expressed in meters per second in the North East Down (NED) coordinate system.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	velocity						The aiding velocity respectively along North, East and Down axis in m/s in NED coordinate system.
 *	\param[in]	accuracy						The accuracy of the provided velocity in m/s.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterVelocityNED(SbgCanDeviceHandle deviceHandle, const float velocity[3], float accuracy);

/*!
 *	Send the position aiding information used by the Navigation filter.<br>
 *	The position is expressed in WGS84 format in degrees for latitude/longitude and in meters for altitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	position						The desired positions respectively latitude, longitude and height in degrees, degrees and meters.
 *	\param[in]	accuracies						The accuracy of the provided position respectively horizontal and vertical in meters.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterPosition(SbgCanDeviceHandle deviceHandle, const double position[3], const float accuracies[2]);

#endif	// __SBG_CAN_COMMANDS_NAV_H__
