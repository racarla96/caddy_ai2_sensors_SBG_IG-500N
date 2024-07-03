/*!
 *	\file		binaryLogStatus.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		03 April 2013
 *
 *	\brief		This file is used to parse received device status binary logs.
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
#ifndef __BINARY_LOG_STATUS_H__
#define __BINARY_LOG_STATUS_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- General status definitions                                         -//
//----------------------------------------------------------------------//
#define SBG_ECOM_GENERAL_MAIN_POWER_OK		(0x0001u << 0)			/*!< Set to 1 when main power supply is OK. */
#define SBG_ECOM_GENERAL_IMU_POWER_OK		(0x0001u << 1)			/*!< Set to 1 when IMU power supply is OK. */
#define SBG_ECOM_GENERAL_GPS_POWER_OK		(0x0001u << 2)			/*!< Set to 1 when GPS power supply is OK. */
#define SBG_ECOM_GENERAL_SETTINGS_OK		(0x0001u << 3)			/*!< Set to 1 if settings where correctly loaded. */
#define SBG_ECOM_GENERAL_TEMPERATURE_OK		(0x0001u << 4)			/*!< Set to 1 when temperature is within specified limits. */
#define SBG_ECOM_GENERAL_DATALOGGER_OK		(0x0001u << 5)			/*!< Set to 1 when the datalogger is working correctly. */

//----------------------------------------------------------------------//
//- Clock status definitions                                           -//
//----------------------------------------------------------------------//

/*!
 * Clock status and UTC time status definitions.
 */
#define SBG_ECOM_CLOCK_STATUS_SHIFT			(1u)					/*!< Shift used to extract the clock status part. */
#define SBG_ECOM_CLOCK_STATUS_MASK			(0x000Fu)				/*!< Mask used to keep only the clock status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_SHIFT		(6u)					/*!< Shift used to extract the clock UTC status part. */
#define SBG_ECOM_CLOCK_UTC_STATUS_MASK		(0x000Fu)				/*!< Mask used to keep only the clock UTC status part. */

/*!
 * Clock status mask definitions.
 */
#define SBG_ECOM_CLOCK_STABLE_INPUT			(0x0001u << 0)			/*!< Set to 1 if a stable input clock could be used to synchronized the internal clock. */
#define SBG_ECOM_CLOCK_UTC_SYNC				(0x0001u << 5)			/*!< The UTC time is synchronized with a PPS. */

/*!
 * Clock status enum.
 */
typedef enum _SbgEComClockStatus
{
	SBG_ECOM_CLOCK_ERROR			= 0,							/*!< An error has occurred on the clock estimation. */
	SBG_ECOM_CLOCK_FREE_RUNNING		= 1,							/*!< The clock is only based on the internal crystal. */
	SBG_ECOM_CLOCK_STEERING			= 2,							/*!< A PPS has been detected and the clock is converging to it. */
	SBG_ECOM_CLOCK_VALID			= 3								/*!< The clock has converged to the PPS and is within 500ns. */
} SbgEComClockStatus;

/*!
 * Status for the UTC time data.
 */
typedef enum _SbgEComClockUtcStatus
{
	SBG_ECOM_UTC_INVALID			= 0,							/*!< The UTC time is not known, we are just propagating the UTC time internally. */
	SBG_ECOM_UTC_NO_LEAP_SEC		= 1,							/*!< We have received valid UTC time information but we don't have the leap seconds information. */
	SBG_ECOM_UTC_VALID				= 2								/*!< We have received valid UTC time data with valid leap seconds. */
} SbgEComClockUtcStatus;

//----------------------------------------------------------------------//
//- Communication status definitions                                   -//
//----------------------------------------------------------------------//

/*!
 * Communication CAN status definitions.
 */
#define SBG_ECOM_CAN_STATUS_SHIFT			(27u)					/*!< Shift used to access the CAN status part. */
#define SBG_ECOM_CAN_STATUS_MASK			(0x00000007u)			/*!< Mask used to keep only the CAN status part. */

/*!
 * Communication status bit mask definitions.
 */
#define SBG_ECOM_PORTA_VALID				(0x00000001u << 0)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTB_VALID				(0x00000001u << 1)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTC_VALID				(0x00000001u << 2)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTD_VALID				(0x00000001u << 3)		/*!< Set to 0 in case of low level communication error. */
#define SBG_ECOM_PORTE_VALID				(0x00000001u << 4)		/*!< Set to 0 in case of low level communication error. */

#define SBG_ECOM_PORTA_RX_OK				(0x00000001u << 5)		/*!< Set to 0 in case of error on PORT A input. */
#define SBG_ECOM_PORTA_TX_OK				(0x00000001u << 6)		/*!< Set to 0 in case of error on PORT A output. */
#define SBG_ECOM_PORTB_RX_OK				(0x00000001u << 7)		/*!< Set to 0 in case of error on PORT B input. */
#define SBG_ECOM_PORTB_TX_OK				(0x00000001u << 8)		/*!< Set to 0 in case of error on PORT B output. */
#define SBG_ECOM_PORTC_RX_OK				(0x00000001u << 9)		/*!< Set to 0 in case of error on PORT C input. */
#define SBG_ECOM_PORTC_TX_OK				(0x00000001u << 10)		/*!< Set to 0 in case of error on PORT C output. */
#define SBG_ECOM_PORTD_RX_OK				(0x00000001u << 11)		/*!< Set to 0 in case of error on PORT D input. */
#define SBG_ECOM_PORTD_TX_OK				(0x00000001u << 12)		/*!< Set to 0 in case of error on PORT D input. */
#define SBG_ECOM_PORTE_RX_OK				(0x00000001u << 13)		/*!< Set to 0 in case of error on PORT E input. */
#define SBG_ECOM_PORTE_TX_OK				(0x00000001u << 14)		/*!< Set to 0 in case of error on PORT D input. */

#define SBG_ECOM_ETH0_RX_OK					(0x00000001u << 15)		/*!< Set to 0 in case of error on ETH0 input. */
#define SBG_ECOM_ETH0_TX_OK					(0x00000001u << 16)		/*!< Set to 0 in case of error on ETH0 output. */
#define SBG_ECOM_ETH1_RX_OK					(0x00000001u << 17)		/*!< Set to 0 in case of error on ETH1 input. */
#define SBG_ECOM_ETH1_TX_OK					(0x00000001u << 18)		/*!< Set to 0 in case of error on ETH1 output. */
#define SBG_ECOM_ETH2_RX_OK					(0x00000001u << 19)		/*!< Set to 0 in case of error on ETH2 input. */
#define SBG_ECOM_ETH2_TX_OK					(0x00000001u << 20)		/*!< Set to 0 in case of error on ETH2 output. */
#define SBG_ECOM_ETH3_RX_OK					(0x00000001u << 21)		/*!< Set to 0 in case of error on ETH3 input. */
#define SBG_ECOM_ETH3_TX_OK					(0x00000001u << 22)		/*!< Set to 0 in case of error on ETH3 output. */
#define SBG_ECOM_ETH4_RX_OK					(0x00000001u << 23)		/*!< Set to 0 in case of error on ETH4 input. */
#define SBG_ECOM_ETH4_TX_OK					(0x00000001u << 24)		/*!< Set to 0 in case of error on ETH4 output. */

#define SBG_ECOM_CAN_RX_OK					(0x00000001u << 25)		/*!< Set to 0 in case of error on CAN Bus output buffer. */
#define SBG_ECOM_CAN_TX_OK					(0x00000001u << 26)		/*!< Set to 0 in case of error on CAN Bus input buffer. */


/*!
 * Communication status for the CAN Bus.
 */
typedef enum _SbgEComCanBusStatus
{
	SBG_ECOM_CAN_BUS_OFF		= 0,								/*!< Bus OFF operation due to too much errors. */
	SBG_ECOM_CAN_BUS_TX_RX_ERR	= 1,								/*!< Errors on Tx or Rx. */
	SBG_ECOM_CAN_BUS_OK			= 2,								/*!< Bus OK. */
	SBG_ECOM_CAN_BUS_ERROR		= 3									/*!< Bus error. */
} SbgEComCanBusStatus;

//----------------------------------------------------------------------//
//- Aiding status definitions                                          -//
//----------------------------------------------------------------------//
#define SBG_ECOM_AIDING_GPS1_POS_RECV		(0x00000001u << 0)		/*!< Set to 1 when valid GPS 1 position data is received. */
#define SBG_ECOM_AIDING_GPS1_VEL_RECV		(0x00000001u << 1)		/*!< Set to 1 when valid GPS 1 velocity data is received. */
#define SBG_ECOM_AIDING_GPS1_HDT_RECV		(0x00000001u << 2)		/*!< Set to 1 when valid GPS 1 true heading data is received. */
#define SBG_ECOM_AIDING_GPS1_UTC_RECV		(0x00000001u << 3)		/*!< Set to 1 when valid GPS 1 UTC time data is received. */
#define SBG_ECOM_AIDING_GPS2_POS_RECV		(0x00000001u << 4)		/*!< Set to 1 when valid GPS 2 position data is received. */
#define SBG_ECOM_AIDING_GPS2_VEL_RECV		(0x00000001u << 5)		/*!< Set to 1 when valid GPS 2 velocity data is received. */
#define SBG_ECOM_AIDING_GPS2_HDT_RECV		(0x00000001u << 6)		/*!< Set to 1 when valid GPS 2 true heading data is received. */
#define SBG_ECOM_AIDING_GPS2_UTC_RECV		(0x00000001u << 7)		/*!< Set to 1 when valid GPS 2 UTC time data is received. */
#define SBG_ECOM_AIDING_MAG_RECV			(0x00000001u << 8)		/*!< Set to 1 when valid Magnetometer data is received. */
#define SBG_ECOM_AIDING_ODO_RECV			(0x00000001u << 9)		/*!< Set to 1 when Odometer pulse is received. */
#define SBG_ECOM_AIDING_DVL_RECV			(0x00000001u << 10)		/*!< Set to 1 when valid DVL data is received. */
#define SBG_ECOM_AIDING_USBL_RECV			(0x00000001u << 11)		/*!< Set to 1 when valid USBL data is received. */
#define SBG_ECOM_AIDING_EM_LOG_RECV			(0x00000001u << 12)		/*!< Set to 1 when valid EM Log data is received. */
#define SBG_ECOM_AIDING_DEPTH_RECV			(0x00000001u << 13)		/*!< Set to 1 when valid Depth sensor data is received. */
#define SBG_ECOM_AIDING_USER_POS_RECV		(0x00000001u << 14)		/*!< Set to 1 when valid user position data is received. */
#define SBG_ECOM_AIDING_USER_VEL_RECV		(0x00000001u << 15)		/*!< Set to 1 when valid user velocity data is received. */
#define SBG_ECOM_AIDING_USER_HEADING_RECV	(0x00000001u << 16)		/*!< Set to 1 when valid user heading data is received. */

//----------------------------------------------------------------------//
//- Solution status definitions                                        -//
//----------------------------------------------------------------------//

/*!
 * Solution status mode definitions.
 */
#define SBG_ECOM_SOLUTION_MODE_SHIFT		(0u)					/*!< Shift used to extract the clock status part. */
#define SBG_ECOM_SOLUTION_MODE_MASK			(0x0000000Fu)			/*!< Mask used to keep only the clock status part. */

/*!
 * Solution bit masks definitions.
 */
#define SBG_ECOM_SOL_ATTITUDE_VALID			(0x00000001u << 4)		/*!< Set to 1 if attitude data is reliable (Roll/Pitch error < 0,5°). */
#define SBG_ECOM_SOL_HEADING_VALID			(0x00000001u << 5)		/*!< Set to 1 if geading data is reliable (Heading error < 1°). */
#define SBG_ECOM_SOL_VELOCITY_VALID			(0x00000001u << 6)		/*!< Set to 1 if velocity data is reliable (velocity error < 1.5 m/s). */
#define SBG_ECOM_SOL_POSITION_VALID			(0x00000001u << 7)		/*!< Set to 1 if position data is reliable (Position error < 10m). */
#define SBG_ECOM_SOL_VERT_REF_USED			(0x00000001u << 8)		/*!< Set to 1 if vertical reference is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_MAG_REF_USED			(0x00000001u << 9)		/*!< Set to 1 if magnetometer is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_VEL_USED			(0x00000001u << 10)		/*!< Set to 1 if GPS1 velocity is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_POS_USED			(0x00000001u << 11)		/*!< Set to 1 if GPS1 Position is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_COURSE_USED		(0x00000001u << 12)		/*!< Set to 1 if GPS1 Course is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS1_HDT_USED			(0x00000001u << 13)		/*!< Set to 1 if GPS1 True Heading is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_VEL_USED			(0x00000001u << 14)		/*!< Set to 1 if GPS2 velocity is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_POS_USED			(0x00000001u << 15)		/*!< Set to 1 if GPS2 Position is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_COURSE_USED		(0x00000001u << 16)		/*!< Set to 1 if GPS2 Course is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_GPS2_HDT_USED			(0x00000001u << 17)		/*!< Set to 1 if GPS2 True Heading is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_ODO_USED				(0x00000001u << 18)		/*!< Set to 1 if Odometer is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_DVL_BT_USED			(0x00000001u << 19)		/*!< Set to 1 if DVL Bottom Tracking is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_DVL_WT_USED			(0x00000001u << 20)		/*!< Set to 1 if DVL Water Tracking is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_USER_POS_USED			(0x00000001u << 21)		/*!< Set to 1 if user velocity is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_USER_VEL_USED			(0x00000001u << 22)		/*!< Set to 1 if user Position is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_USER_HEADING_USED		(0x00000001u << 23)		/*!< Set to 1 if user Course is used in solution (data used and valid since 3s). */
#define SBG_ECOM_SOL_USBL_USED				(0x00000001u << 24)		/*!< Set to 1 if USBL / LBL is used in solution (data used and valid since 3s). */

/*!
 * Solution filter mode enum.
 */
typedef enum _SbgEComSolutionMode
{
	SBG_ECOM_SOL_MODE_UNINITIALIZED			= 0,					/*!< The Kalman filter is not initialized and the returned data are all invalid. */
	SBG_ECOM_SOL_MODE_VERTICAL_GYRO			= 1,					/*!< The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. */
	SBG_ECOM_SOL_MODE_AHRS					= 2,					/*!< A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. */
	SBG_ECOM_SOL_MODE_NAV_VELOCITY			= 3,					/*!< The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. */
	SBG_ECOM_SOL_MODE_NAV_POSITION			= 4						/*!< Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided. */
} SbgEComSolutionMode;


//----------------------------------------------------------------------//
//- Status definitions                                                 -//
//----------------------------------------------------------------------//

/*!
 * Stores global status data.
 */
typedef struct _SbgLogStatusData
{
	uint32	timeStamp;												/*!< Time in us since the sensor power up. */
	uint16	generalStatus;											/*!< General status bitmask and enums. */
	uint16	clockStatus;											/*!< Clock model and UTC related status. */
	uint32	comStatus;												/*!< Communication status bitmask and enums. */
	uint32	aidingStatus;											/*!< Aiding equipments status bitmask and enums. */
	uint32	solutionStatus;											/*!< Internal filter solution status and indicators. */
	uint16	reserved;												/*!< Reserved status field for future use */
} SbgLogStatusData;

//----------------------------------------------------------------------//
//- Clock status helpers methods                                       -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the clock status from a status field.
 * \param[in]	status				Status field to extract the clock status from it.
 * \return							The extracted clock status.
 */
SBG_INLINE SbgEComClockStatus sbgEComLogStatusGetClockStatus(uint16 status)
{
	return (SbgEComClockStatus)((status >> SBG_ECOM_CLOCK_STATUS_SHIFT) & SBG_ECOM_CLOCK_STATUS_MASK);
}

/*!
 * Method used to read the UTC time status from a clock status field.
 * \param[in]	status				Status field to extract the UTC time status from it.
 * \return							The extracted UTC time status.
 */
SBG_INLINE SbgEComClockUtcStatus sbgEComLogStatusGetClockUtcStatus(uint16 status)
{
	return (SbgEComClockUtcStatus)((status >> SBG_ECOM_CLOCK_UTC_STATUS_SHIFT) & SBG_ECOM_CLOCK_UTC_STATUS_MASK);
}

/*!
 * Method used to write the clock status field.
 * \param[in]	clockStatus			The clock status to set.
 * \param[in]	utcStatus			The UTC time status to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build clock status field.
 */
SBG_INLINE uint16 sbgEComLogStatusBuildClockStatus(SbgEComClockStatus clockStatus, SbgEComClockUtcStatus utcStatus, uint16 masks)
{
	//
	// Create the combined status field
	//
	return	((((uint16)clockStatus)&SBG_ECOM_CLOCK_STATUS_MASK) << SBG_ECOM_CLOCK_STATUS_SHIFT) |
			((((uint16)utcStatus)&SBG_ECOM_CLOCK_UTC_STATUS_MASK) << SBG_ECOM_CLOCK_UTC_STATUS_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Communication status helpers methods                               -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the CAN bus status from a communication status field.
 * \param[in]	status				Status field to extract the CAN bus status from it.
 * \return							The extracted CAN bus status.
 */
SBG_INLINE SbgEComCanBusStatus sbgEComLogStatusGetCanStatus(uint32 status)
{
	return (SbgEComCanBusStatus)((status >> SBG_ECOM_CAN_STATUS_SHIFT) & SBG_ECOM_CAN_STATUS_MASK);
}

/*!
 * Method used to write the CAN bus status field.
 * \param[in]	canStatus			The CAN bus status to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build communication status field.
 */
SBG_INLINE uint32 sbgEComLogStatusBuildCommunicationStatus(SbgEComCanBusStatus canStatus, uint32 masks)
{
	//
	// Create the combined status field
	//
	return	((((uint32)canStatus)&SBG_ECOM_CAN_STATUS_MASK) << SBG_ECOM_CAN_STATUS_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Solution status helpers methods                                    -//
//----------------------------------------------------------------------//

/*!
 * Method used to read the solution mode from a solution status field.
 * \param[in]	status				Status uint32 value to extract the solution mode from it.
 * \return							The extracted solution mode.
 */
SBG_INLINE SbgEComSolutionMode sbgEComLogStatusGetSolutionMode(uint32 status)
{
	return (SbgEComSolutionMode)((status >> SBG_ECOM_SOLUTION_MODE_SHIFT) & SBG_ECOM_SOLUTION_MODE_MASK);
}

/*!
 * Method used to write the solution status field.
 * \param[in]	solutionMode		The solution mode to set.
 * \param[in]	masks				Bit mask to set.
 * \return							The build solution status field.
 */
SBG_INLINE uint32 sbgEComLogStatusBuildSolutionStatus(SbgEComSolutionMode solutionMode, uint32 masks)
{
	//
	// Create the combined status field
	//
	return	((((uint32)solutionMode)&SBG_ECOM_SOLUTION_MODE_MASK) << SBG_ECOM_SOLUTION_MODE_SHIFT) | masks;
}

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_STATUS message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseStatusData(const void *pPayload, uint32 payloadSize, SbgLogStatusData *pOutputData);

#endif
