/*!
 *	\file		sbgCan.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		10/06/10
 *
 *	\brief		Main header file for sbgCom library.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */

/*!
 *	\mainpage sbgCan library documentation
 *	Welcome to the sbgCan library documentation.<br>
 *	This documentation describes all functions implemented in the sbgCan library. 
 */

#ifndef __SBG_CAN_H__
#define __SBG_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgCommon.h"
#include "time/sbgTime.h"
#include "protocol/sbgCanDevice.h"

#include "protocol/sbgCanCommandsCalib.h"
#include "protocol/sbgCanCommandsExt.h"
#include "protocol/sbgCanCommandsFilter.h"
#include "protocol/sbgCanCommandsGps.h"
#include "protocol/sbgCanCommandsIg30.h"
#include "protocol/sbgCanCommandsNav.h"
#include "protocol/sbgCanCommandsOdo.h"
#include "protocol/sbgCanCommandsOrientation.h"
#include "protocol/sbgCanCommandsOutput.h"
#include "protocol/sbgCanCommandsSetting.h"
#include "protocol/sbgCanCommandsSync.h"
#include "protocol/extDevices/sbgCanExtIg.h"
#include "protocol/extDevices/sbgCanExtNmea.h"

//----------------------------------------------------------------------//
//- sbgCam library main operations                                     -//
//----------------------------------------------------------------------//

/*!
 *	Returns an integer representing the version of the sbgCan library.
 *	\return									An integer representing the version of the sbgCan library.<br>
 *											Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgCanGetVersion(void);

/*!
 *	Retreive the sbgCan library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCan library version.
 */
const char *sbgCanGetVersionAsString(void);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode					The errorCode to convert into a string.
 *	\param[out]	errorMsg					String buffer used to hold the error string.
 */
void sbgCanErrorToString(char errorMsg[256], SbgErrorCode errorCode);

//----------------------------------------------------------------------//
//- Footer (close extern C block)                                      -//
//----------------------------------------------------------------------//

#ifdef __cplusplus
}
#endif

#endif	// __SBG_CAN_H__
