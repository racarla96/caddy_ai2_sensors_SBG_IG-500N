/*!
 *	\file		sbgCanCommandsOrientation.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		19/07/10
 *
 *	\brief		Orientation commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_ORIENTATION_H__
#define __SBG_CAN_COMMANDS_ORIENTATION_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//  Orientation arguments definitions						            //
//----------------------------------------------------------------------//

/*!
 *	Type of oriention reset to do.
 */
typedef enum _SbgCanOffsetType
{
	SBG_CAN_OFFSET_PRE_ROT_Z_RESET		= 3,	/*!< Calculate a pre rotation on Z axis */
	SBG_CAN_OFFSET_PRE_ROT_XY_RESET		= 4,	/*!< Calculate a pre rotation on X and Y axis */
	SBG_CAN_OFFSET_PRE_ROT_XYZ_RESET	= 5,	/*!< Calculate a pre rotation on X,Y and Z axis */

	SBG_CAN_OFFSET_POST_ROT_Z_RESET		= 6,	/*!< Calculate a post rotation on Z axis */
	SBG_CAN_OFFSET_POST_ROT_XY_RESET	= 7,	/*!< Calculate a post rotation on X and Y axis */
	SBG_CAN_OFFSET_POST_ROT_XYZ_RESET	= 8		/*!< Calculate a post rotation on X,Y and Z axis */
} SbgCanOffsetType;

//----------------------------------------------------------------------//
//- Orientation commands											   -//
//----------------------------------------------------------------------//

/*!
 *	Configures the device to align to the coordinate frame's axis.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	offset							The orientation offset type.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetAutoOrientationOffset(SbgCanDeviceHandle deviceHandle, SbgCanOffsetType offset);

/*!
 *	Set the manual rotation to be applied on sensors input in pre-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetManualPreOrientationOffset(SbgCanDeviceHandle deviceHandle, const float quaternion[4]);

/*!
 *	Get the manual rotation to be applied on sensors input in pre-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetManualPreOrientationOffset(SbgCanDeviceHandle deviceHandle, float quaternion[4]);

/*!
 *	Set the manual rotation to be applied on sensors input in post-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetManualPostOrientationOffset(SbgCanDeviceHandle deviceHandle, const float quaternion[4]);

/*!
 *	Get the manual rotation to be applied on sensors input in post-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetManualPostOrientationOffset(SbgCanDeviceHandle deviceHandle, float quaternion[4]);

#endif	// __SBG_CAN_COMMANDS_ORIENTATION_H__