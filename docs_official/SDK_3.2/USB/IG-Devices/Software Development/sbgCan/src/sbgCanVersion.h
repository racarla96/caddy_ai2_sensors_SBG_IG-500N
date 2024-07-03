/*!
 *	\file		sbgCanVersion.h
 *  \author		SBG-Systems (Raphael Siryani)
 *	\date		21/06/10
 *
 *	\brief		Version header file for the sbgCan library.<br>
 *				You can find in this file all change logs.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_VERSION_H__
#define __SBG_CAN_VERSION_H__

#include "sbgCommon.h"

//----------------------------------------------------------------------//
//- Version definitions                                                -//
//----------------------------------------------------------------------//

/*!
 *	Change log:<br>
 *	============<br>
 *	<br>
 *	<table border="0">
 *
 *	<tr><td valign="top">22/11/13:</td><td>
 *	Version 2.1.0.0:                
 *	<h2>        New Feature
 *	</h2>
 *	<ul>
 *	<li>[SBGCOM-48] -         Added Heave period configuration
 *	</li>
 *	</ul>
 *	
 *	</td></tr>
 *
 *	<tr><td valign="top">16/54/12:</td><td>
 *	Version 2.0.0.0:     
 *	<h2>        Improvement
 *	</h2>
 *	<ul>
 *	<li>[SBGCAN-11] -         The enum sbgCanCalibAction has been split into two enums, one for magnetometers and one for gyroscopes 
 *	</li>
 *	<li>[SBGCAN-16] -         Changed sbgCanGetVersionAsString method behavior
 *	</li>
 *	<li>[SBGCAN-23] -         Updated PCAN library for better driver support to version 1.2.2.32
 *	</li>
 *	</ul>
 *	    
 *	<h2>        New Feature
 *	</h2>
 *	<ul>
 *	<li>[SBGCAN-1] -         sbgCanSetFilterHeadingSource: GPS + acceleration heading source added
 *	</li>
 *	<li>[SBGCAN-2] -         Motion profiles management
 *	</li>
 *	<li>[SBGCAN-4] -         sbgCanSetAdvancedOptions and sbgCanGetAdvancedOptions added
 *	</li>
 *	<li>[SBGCAN-7] -         sbgCanSetFilterHeadingSource: GPS + acceleration heading source added
 *	</li>
 *	<li>[SBGCAN-9] -         sbgCanSetHeaveConf sbgCanGetHeaveConf added for heave configuration
 *	</li>
 *	<li>[SBGCAN-14] -         sbgCanSetOdoConfig: Automatic odmometer gain calibration by GPS parameter added
 *	</li>
 *	<li>[SBGCAN-15] -         sbgCanSetLowPowerMode now handles a sensor high performance mode
 *	</li>
 *	<li>[SBGCAN-17] -         Add Virtual odometer output trigger event
 *	</li>
 *	<li>[SBGCAN-22] -         NMEA device added option in sbgCanExtNmeaSetOptions: SBG_NMEA_OPT_HDT_AFTER_RMC for HDT frame timestamp management.
 *	</li>
 *	</ul>
 *	   
 *	<h2>        Removed feature
 *	</h2>
 *	<ul>
 *	<li>[SBGCAN-3] -         sbgCanCalibMagProcedure does not support onboard magnetic calibration anymore
 *	</li>
 *	<li>[SBGCAN-5] -         sbgCanSetFilterAttitudeErrors and sbgCanGetFilterAttitudeErrors are obsolete
 *	</li>
 *	<li>[SBGCAN-6] -         sbgCanGetKalmanFilter and sbgCanSetKalmanFilter is obsolete IG-500
 *	</li>
 *	<li>[SBGCAN-8] -         sbgCanSetGpsOptions and sbgCanGetGpsOptions are now obsolete on IG-500
 *	</li>
 *	<li>[SBGCAN-10] -         sbgCanSetVelocityConstraints and sbgCanGetVelocityConstraitns are now obsolete
 *	</li>
 *	<li>[SBGCAN-13] -         sbgCanGetErrorLog command is obsolete
 *	</li>
 *	<li>[SBGCAN-19] -         Remote IG-Device Automatic orientation offset is obsolete
 *	</li>
 *	<li>[SBGCAN-20] -         Remote IG-Device &quot;remote frame&quot; system is obsolete
 *	</li>
 *	</ul>
 *	    
 *	<h2>        Task
 *	</h2>
 *	<ul>
 *	<li>[SBGCAN-12] -         User buffer size changed to 64 bytes
 *	</li>
 *	<li>[SBGCAN-18] -         Update CAN IDs names with a common name between low level protocol specification and sbgCan
 *	</li>
 *	</ul>
 *	
 *	</td></tr>
 *
 *	<tr><td valign="top">15/02/12:</td><td>
 *	Version 1.1.5.0:	Changed sbgCanInterfaceGetBusStatus to return a mask of CAN bus status and error.
 *	</td></tr>
 *
 *	<tr><td valign="top">23/06/11:</td><td>
 *	Version 1.1.0.0:	Changed all definitions to include SBG_CAN instead of SBG_.
 *						Methods: sbgSetUserBuffer, sbgGetUserBuffer, sbgSetDeviceErrorCallback, sbgSetDeviceOutputCallback, sbgGetErrorLog syntax have been fixed.
 *						The new name is sbgCanSetUserBuffer, sbgCanGetUserBuffer, sbgCanSetDeviceErrorCallback, sbgCanSetDeviceOutputCallback, sbgCanGetErrorLog.
 *	</td></tr>
 *
 *	<tr><td valign="top">03/03/11:</td><td>
 *	Initial 1.0.0.0 release.
 *	</td></tr>
 *
 *	<tr><td valign="top">25/02/11:</td><td>
 *	Version 0.9.0.0 RC1 for 1.0.0.0 release.
 *	</td></tr>
 *
 *	</table>
 */

#define SBG_CAN_VERSION		"2.1.0.0"
#define SBG_CAN_VERSION_U	SBG_VERSION(2,1,0,0)
#define SBG_CAN_VERSION_R	"2, 1, 0, 0\0"
#define SBG_CAN_VERSION_W	2,1,0,0

#endif
