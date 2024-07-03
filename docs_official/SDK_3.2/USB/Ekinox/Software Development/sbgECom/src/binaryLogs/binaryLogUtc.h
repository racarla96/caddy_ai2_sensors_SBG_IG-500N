/*!
 *	\file		binaryLogUtc.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		20 February 2013
 *
 *	\brief		This file is used to parse received UTC binary logs.
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
#ifndef __BINARY_LOG_UTC_H__
#define __BINARY_LOG_UTC_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Log structure definitions                                          -//
//----------------------------------------------------------------------//

/*!
 *	Structure that stores data for the SBG_ECOM_LOG_UTC_TIME message.
 */
typedef struct _SbgLogUtcData
{
	uint32	timeStamp;					/*!< Time in us since the sensor power up. */
	uint16	reserved;					/*!< Reserved for future use. */
	uint16	year;						/*!< Year for example: 2013. */
	int8	month;						/*!< Month in year [1 .. 12]. */
	int8	day;						/*!< Day in month [1 .. 31]. */
	int8	hour;						/*!< Hour in day [0 .. 23]. */
	int8	minute;						/*!< Minute in hour [0 .. 59]. */
	int8	second;						/*!< Second in minute [0 .. 60]. (60 is used only when a leap second is added) */
	int32	nanoSecond;					/*!< Nanosecond of current second in ns. */
	uint32	gpsTimeOfWeek;				/*!< GPS time of week in ms. */
} SbgLogUtcData;

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_UTC_DATA message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseUtcData(const void *pPayload, uint32 payloadSize, SbgLogUtcData *pOutputData);

#endif
