/*!
 *	\file		sbgCanProtocolOutputMode.h
 *  \author		SBG-Systems (Olivier)
 *	\date		11/10/10
 *
 *	\brief		Output format handling system for protocol use.<br>
 *				This file handles endianness and fixed/float issues.<br>
 *				Using one of the two definitions SBG_PLATFORM_BIG_ENDIAN and SBG_PLATFORM_LITTLE_ENDIAN,<br>
 *				you can define the endianness of the platform you are compiling sbgCom for.<br>
 *				If your platform is in little endian, just define the preprocessor define SBG_PLATFORM_LITTLE_ENDIAN.<br>
 *				If your platform is in big endian, you should define the define SBG_PLATFORM_BIG_ENDIAN.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_PROTOCOL_OUTPUT_MODE_H__
#define __SBG_CAN_PROTOCOL_OUTPUT_MODE_H__

#include "../sbgCommon.h"

//----------------------------------------------------------------------//
//- Methods used to convert data from host to target format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int16 sbgCanHostToTarget16(int16 val);

/*!
 *	Convert a 16 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint16 sbgCanHostToTargetU16(uint16 val);

/*!
 *	Convert a 32 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int32 sbgCanHostToTarget32(int32 val);

/*!
 *	Convert a 32 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint32 sbgCanHostToTargetU32(uint32 val);

/*!
 *	Convert a 64 bits integer from host to target endianness.
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
int64 sbgCanHostToTarget64(int64 val);

/*!
 *	Convert a 64 bits integer from host to target endianness. (unsigned)
 *	\param[in]	val						Value in host endianness.
 *	\return								Value in target endianness.
 */
uint64 sbgCanHostToTargetU64(uint64 val);

/*!
 *	Convert a 32 bits float from host endianness to a float/fixed32 in target endianness
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness float.
 */
uint32 sbgCanHostToTargetFixed32(float val);

/*!
 *	Convert a 64 bits double from host endianness to a fixed64 in target endianness.
 *	\param[in]	val						Value in host endianness in double.
 *	\return								Value in target endianness in double.
 */
uint64 sbgCanHostToTargetFixed64(double val);

/*!
 *	Convert a 32 bits float from host endianness to a frac16 in target endianness
 *	\param[in]	val						Value in host endianness in float.
 *	\return								Value in target endianness frac16.
 */
int16 sbgCanHostToTargetFrac16(float val);

//----------------------------------------------------------------------//
//- Methods used to convert data from target to host format            -//
//----------------------------------------------------------------------//

/*!
 *	Convert a 16 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
int16 sbgCanTargetToHost16(int16 val);

/*!
 *	Convert a unsigned 16 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint16 sbgCanTargetToHostU16(uint16 val);

/*!
 *	Convert a 32 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
int32 sbgCanTargetToHost32(int32 val);

/*!
 *	Convert a unsigned 32 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness.
 *	\return								Value in host endianness.
 */
uint32 sbgCanTargetToHostU32(uint32 val);

/*!
 *	Convert a 64 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
int64 sbgCanTargetToHost64(int64 val);

/*!
 *	Convert a unsigned 64 bits integer from target to host endianness.
 *	\param[in]	val						Value in target endianness (little endian).
 *	\return								Value in host endianness.
 */
uint64 sbgCanTargetToHostU64(uint64 val);

/*!
 *	Convert a 32 bits number from target endianness to a float in host endianness.
 *	\param[in]	val						Value in target endianness in float.
 *	\return								Value in host endianness in float.
 */
float sbgCanTargetToHostFloat(uint32 val);

/*!
 *	Convert a 64 bit number in from target endianness into a double value in host endianness.
 *	\param[in]	targetOutputMode		The output mode we are using.
 *	\param[in]	val						The value in double/fixed in little/big endian to convert.
 *	\return								A 64 bits double value in little endian.
 */
double sbgCanTargetToHostDouble(uint64 val);

/*!
 *	Convert a 16 bits frac16 number from target endianness to a float in host endianness.
 *	\param[in]	val						Value in target endianness in frac16.
 *	\return								Value in host endianness in float.
 */
float sbgCanTargetToHostFrac16(int16 val);

#endif	// __SBG_CAN_PROTOCOL_OUTPUT_MODE_H__
