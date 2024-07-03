/*!
 *	\file		sbgEComCmds.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		25 February 2013
 *
 *	\brief		Defines all sbgECom commands identifiers.
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

/*!
 *	\mainpage SBG Systems Enhanced Communication library documentation
 *	Welcome to the sbgECom library documentation.<br>
 *	This documentation describes all functions implemented in the sbgECom library.
 */

#ifndef __SBG_ECOM_CMDS_H__
#define __SBG_ECOM_CMDS_H__

//----------------------------------------------------------------------//
//- Binary LOGS command enum                                           -//
//----------------------------------------------------------------------//

/*!
 *	Enum used to identify each sbgECom command and log.
 */
typedef enum _SbgEComCmdId
{
	SBG_ECOM_LOG_STATUS				= 1,			/*!< Log used to retrieve all system status. */
	SBG_ECOM_LOG_UTC_TIME			= 2,			/*!< UTC time data. */
	SBG_ECOM_LOG_IMU_DATA			= 3,			/*!< Inertial Measurement Unit accelerometers and gyroscopes data. */
	SBG_ECOM_LOG_MAG				= 4,			/*!< Magnetometers data. */
	SBG_ECOM_LOG_MAG_CALIB			= 5,			/*!< This command is used to output magnetometers calibration data. */

	SBG_ECOM_LOG_EKF_EULER			= 6,			/*!< Computed euler angles orientation data. */
	SBG_ECOM_LOG_EKF_QUAT			= 7,			/*!< Computed quaternion orientation data. */
	SBG_ECOM_LOG_EKF_NAV			= 8,			/*!< Computed velocity and position navigation data. */

	SBG_ECOM_LOG_SHIP_MOTION_0		= 9,			/*!< Return ship motion such as surge, sway, heave at the main monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_1		= 10,			/*!< Return ship motion such as surge, sway, heave at the first monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_2		= 11,			/*!< Return ship motion such as surge, sway, heave at the second monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_3		= 12,			/*!< Return ship motion such as surge, sway, heave at the third monitoring point. */

	SBG_ECOM_LOG_GPS1_VEL			= 13,			/*!< GPS 1 velocity log data. */
	SBG_ECOM_LOG_GPS1_POS			= 14,			/*!< GPS 1 position log data. */
	SBG_ECOM_LOG_GPS1_HDT			= 15,			/*!< GPS 1 true heading log data. */

	SBG_ECOM_LOG_GPS2_VEL			= 16,			/*!< GPS 2 velocity log data. */
	SBG_ECOM_LOG_GPS2_POS			= 17,			/*!< GPS 2 position log data. */
	SBG_ECOM_LOG_GPS2_HDT			= 18,			/*!< GPS 2 true heading log data. */

	SBG_ECOM_LOG_ODO_VEL			= 19,			/*!< Measured odometer velocity. */

	SBG_ECOM_LOG_USER_HEADING		= 20,			/*!< User true heading aiding data. */
	SBG_ECOM_LOG_USER_VEL_NED		= 21,			/*!< User velocity aiding data expressed in the NED local frame. */
	SBG_ECOM_LOG_USER_VEL_XYZ		= 22,			/*!< User velocity aiding data expressed in the device local frame. */
	SBG_ECOM_LOG_USER_POS_LLA		= 23,			/*!< User position aiding data expressed in latitude, longitude, altitude frame. */

	SBG_ECOM_LOG_EVENT_A			= 24,			/*!< Event marker for the synchronization A input signal. */
	SBG_ECOM_LOG_EVENT_B			= 25,			/*!< Event marker for the synchronization B input signal. */
	SBG_ECOM_LOG_EVENT_C			= 26,			/*!< Event marker for the synchronization C input signal. */
	SBG_ECOM_LOG_EVENT_D			= 27,			/*!< Event marker for the synchronization D input signal. */
	SBG_ECOM_LOG_EVENT_E			= 28,			/*!< Event marker for the synchronization E input signal. */

	SBG_ECOM_LOG_DVL_BOTTOM_TRACK	= 29,			/*!< Doppler Velocity Log for bottom tracking data. */
	SBG_ECOM_LOG_DVL_WATER_TRACK	= 30,			/*!< Doppler Velocity log for water tracking data. */

	SBG_ECOM_LOG_GPS1_RAW			= 31,			/*!< GPS 1 raw data for post processing. */

	SBG_ECOM_LOG_SHIP_MOTION_HP_0	= 32,			/*!< Return delayed ship motion such as surge, sway, heave at the main monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_1	= 33,			/*!< Return delayed ship motion such as surge, sway, heave at the first monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_2	= 34,			/*!< Return delayed ship motion such as surge, sway, heave at the second monitoring point. */
	SBG_ECOM_LOG_SHIP_MOTION_HP_3	= 35,			/*!< Return delayed ship motion such as surge, sway, heave at the third monitoring point. */

	SBG_ECOM_LOG_DEBUG_0			= 99,			/*!< Debug Log. */

	SBG_ECOM_CMD_ACK				= 100,			/*!< Generic ACK command. */
	SBG_ECOM_CMD_SAVE_SETTINGS		= 101,			/*!< Command used to save the current configuration into FLASH memory. */
	SBG_ECOM_CMD_IMPORT_SETTINGS	= 102,			/*!< Command used to upload a settings file onto the device. */
	SBG_ECOM_CMD_EXPORT_SETTINGS	= 103,			/*!< Command used to export settings from the device. */
	SBG_ECOM_CMD_SET_MAG_CALIB		= 111			/*!< Command used to define the magnetometers calibration data. */
} SbgEComCmdId;

#endif	/* __SBG_ECOM_CMDS_H__ */
