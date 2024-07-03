/*!
 *	\file		sbgEComVersion.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		05 February 2013
 *
 *	\brief		Header file that contains all versions related information such as change log.
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
#ifndef __SBG_E_COM_VERSION_H__
#define __SBG_E_COM_VERSION_H__

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
 *	<tr><td valign="top">20/03/14:</td><td  valign="top">
 *	Version 1.2.0.0:
 *	 
 *	<h2>        Improvement
 *	</h2>
 *	<ul>
 *	<li>[SBGECOM-13] -         Updated SBG_ECOM_LOG_SHIP_MOTION_XXX logs to add velocity and status data
 *	</li>
 *  <li>[SBGECOM-16] -         Changed GPS OmniStar solution type to PPP ones for better compatibility with third party GPS
 *  </li>
 *	</ul>
 *	    
 *	<h2>        New Feature
 *	</h2>
 *	<ul>
 *	<li>[SBGECOM-14] -         Added SBG_ECOM_LOG_SHIP_MOTION_HP logs for delayed heave output
 *	</li>
 *	<li>[SBGECOM-15] -         Added sbgInterfaceSerialChangeBaudrate method to change the serial interface baud rate
 *	</li>
 *  <li>[SBGECOM-17] -         Added SBG_ECOM_POS_FIXED / SBG_ECAN_POS_FIXED position type for GPS
 *  </li>
 *	</ul>
 *	   
 *	<h2>        Removed feature
 *	</h2>
 *	<ul>
 *	<li>[SBGECOM-11] -         Removed heave status field from SBG_ECOM_LOG_STATUS log.
 *	</li>
 *	</ul>
 *	</td></tr>
 *
 *	<tr><td valign="top">22/11/13:</td><td  valign="top">
 *	Version 1.1.0.0:
 *	
 *	<h2>        Improvement
 *	</h2>
 *	<ul>
 *	<li>[SBGECOM-2] -         Added pitch information in the SbgLogGpsHdt GPS true heading log
 *	</li>
 *	<li>[SBGECOM-5] -         Now sbgEComProtocolReceive method returns the received command even if the CRC is not valid
 *	</li>
 *	</ul>
 *	    
 *	<h2>        New Feature
 *	</h2>
 *	<ul>
 *	<li>[SBGECOM-1] -         Added output log for DVL support
 *	</li>
 *	<li>[SBGECOM-3] -         Added output for GPS 1 raw data in order to support post processing
 *	</li>
 *	<li>[SBGECOM-4] -         Added event markers logs support
 *	</li>
 *	<li>[SBGECOM-6] -         Added Unix support and build script
 *	</li>
 *	<li>[SBGECOM-8] -         Added sbgEComReceiveAnyCmd method that return any received command that is not an output log
 *	</li>
 *	<li>[SBGECOM-9] -         Added settings import and export command
 *	</li>
 *	</ul>
 *	</td></tr>
 *
 *	<tr><td valign="top">01/04/13:</td><td>
 *	Version 1.0.0.0 initial release.<br>
 *	</td></tr>
 *	</table>
 */
#define SBG_E_COM_VERSION		"1.2.0.0"
#define SBG_E_COM_VERSION_U		SBG_VERSION(1,2,0,0)
#define SBG_E_COM_VERSION_R		"1.2.0.0\0"
#define SBG_E_COM_VERSION_W		1,2,0,0

#endif
