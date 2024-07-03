/*!
 *	\file		sbgCanCommandsFilter.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		16/06/10
 *
 *	\brief		Kalman Filter commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_FILTER_H__
#define __SBG_CAN_COMMANDS_FILTER_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//--------------------------------------------------------------------------------//
//- Definitons about kalman filter                                               -//
//--------------------------------------------------------------------------------//

#define SBG_CAN_MP_MAX_SIZE								(2048)		/*!< Maximum size allowed for a motion profile buffer */
#define SBG_CAN_MP_DEFAULT_SIZE							(0)			/*!< Default size of a motion profile with current DK version */

/*!
 *	List of available sources for heading corrections.
 */
typedef enum _SbgCanHeadingSource
{
	SBG_CAN_HEADING_SOURCE_NONE							=	0x00,	/*!< The heading is calculated using gyroscopes only. */
	SBG_CAN_HEADING_SOURCE_MAGNETOMETERS				=	0x01,	/*!< The heading is based on magnetometers information and gyroscopes. */
	SBG_CAN_HEADING_SOURCE_GPS_COURSE					=	0x02,	/*!< GPS course is used */
	SBG_CAN_HEADING_GPS_ACCELERATIONS					=	0x03,	/*!< GPS + accelerometer heading source */
	SBG_CAN_HEADING_SOURCE_USER							=	0x05,	/*!< Heading is fed by user via the main protocol */
	SBG_CAN_HEADING_SOURCE_REMOTE_MAG					=	0x06,	/*!< Remote magnetometers used as a heading source */
	SBG_CAN_HEADING_SOURCE_REMOTE_TRUE_HEADING			=	0x07	/*!< Remote true heading sensor used (dual antenna)  */
} SbgCanHeadingSource;

//----------------------------------------------------------------------//
//- Kalman filter commands											   -//
//----------------------------------------------------------------------//

/*!
 * Defines the kalman filter motion profile to be used
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pMpBuffer						Motion profile buffer pointer
 * \param[in]	mpSize							Motion profile buffer size
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanSetMotionProfile(SbgCanDeviceHandle deviceHandle, void *pMpBuffer, uint16 mpSize);

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Sends a motion profile buffer part
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pMpBuffer						Motion profile buffer pointer
 * \param[in]	index							Index in the IG-500 motion profile buffer
 * \param[in]	size							Number of bytes to transmit
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanSendMPBuffer(SbgCanDeviceHandle deviceHandle, void *pMpBuffer, uint16 index, uint16 size);

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Once the buffer is fully sent, Validate and use a motion profile buffer
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanValidateMPBuffer(SbgCanDeviceHandle deviceHandle);

/*!
 * Retrives the kalman filter motion profile information
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pId								Motion profile Identifier is returned here
 * \param[in]	pVersion						Motion profile version is returned here
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanGetMotionProfileInfo(SbgCanDeviceHandle deviceHandle, uint32 *pId, uint32 *pVersion);

/*!
 *	Set the external source for heading.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	source							The heading source.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetFilterHeadingSource(SbgCanDeviceHandle deviceHandle, SbgCanHeadingSource source);

/*!
 *	Get the external source for heading.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pSource							Pointer to an uint8 used to hold the heading source.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetFilterHeadingSource(SbgCanDeviceHandle deviceHandle, SbgCanHeadingSource *pSource);

/*!
 *	Set the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	declination						The north magnetic declination in radians.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetMagneticDeclination(SbgCanDeviceHandle deviceHandle, float declination);

/*!
 *	Get the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDeclination					Pointer to a float used to hold the north magnetic declination in radians.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetMagneticDeclination(SbgCanDeviceHandle deviceHandle, float *pDeclination);

/*!
 *	Defines the Heave configuration
 *	\param[in]	deviceHandle			A valid sbgCan library handle.
 *	\param[in]	enableHeave				Set to true if heave has to be computed
 *	\param[in]	heavePeriod				Set the average heave period. Must be between 0.3 and 20.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetHeaveConf(SbgCanDeviceHandle deviceHandle, bool enableHeave, float heavePeriod);

/*!
 *	Returns the heave configuration
 *	\param[in]	deviceHandle			A valid sbgCan library handle.
 *	\param[out]	pEnableHeave			Set to true if heave has to be computed
 *	\param[out]	pHeavePeriod			Average heave period if heave computation is enabled
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetHeaveConf(SbgCanDeviceHandle deviceHandle, bool *pEnableHeave, float *pHeavePeriod);

/*!
 *	Send the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	heading							The desired heading in degrees used by the device as input.
 *	\param[in]	accuracy						The heading accuracy in degrees for the previous datum.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterHeading(SbgCanDeviceHandle deviceHandle, float heading, float accuracy);

#endif	// __SBG_CAN_COMMANDS_FILTER_H__