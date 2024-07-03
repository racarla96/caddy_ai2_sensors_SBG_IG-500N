/*!
 *	\file		sbgCanBus.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		09/07/10
 *
 *	\brief		Implementation of the IG devices can communication on the bus.<br>
 *				You can access low-level communication with the interface.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */

#ifndef __SBG_CAN_BUS_H__
#define __SBG_CAN_BUS_H__

#include "../canWrapper/sbgCanWrapper.h"
#include "../sbgCommon.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//----------------------------------------------------------------------//
//- Bus communication handle definitions							   -//
//----------------------------------------------------------------------//

/*!
 * Identify an invalid bus handle.
 */
#define SBG_CAN_INVALID_BUS_HANDLE			(NULL)

/*! 
 * Type used to list all Can devices attached type definition
 */
typedef void	*SbgCanDeviceList;

/*!
 *	Struct containing all bus handle related data.
 */
typedef struct _SbgCanBusHandleStr
{
	SbgCanInterfaceHandle			 canInterfaceHandle;		/*!< Handle to the hardware interface. */
	SbgCanDeviceList				*pListTail;					/*!< Pointeur to the set of devices associated with the bus. */
	uint32							 listLenght;				/*!< Lenght of the devices list associated with the bus. */
} SbgCanBusHandleStr;

/*!
 *	Handle type used by the bus system.
 */
typedef SbgCanBusHandleStr *SbgCanBusHandle;

/*!
 *	Define the timeout used for reception
 */
#define SBG_CAN_FRAME_RECEPTION_TIME_OUT				(500)

//-----------------------------------------------------------------------//
//- Raw Can Message operations						                    -//
//-----------------------------------------------------------------------//

/*!
 *	Set a can message.
 *	\param[out]	pMsg					Pointer to the can message struct that contain can id, data length and data field.
 *	\param[in]	format					CAN message format (CAN_ID_STANDARD or CAN_ID_EXTENDED).
 *	\param[in]	id						The desired can message id.
 *	\param[in]	length					The data length of the message.
 *	\param[in]	data					The data field of the message.
 *	\return								SBG_NO_ERROR if the message is set correctly.
 */
SbgErrorCode sbgCanSetRawMessage(SbgCanMessageRaw *pMsg, SbgCanMessageFormat format, uint32 id, uint8 length, const uint8 data[8]);

//----------------------------------------------------------------------//
//- Bus communication operations			                           -//
//----------------------------------------------------------------------//

/*!
 *	Init the bus system and return the created handle.
 *	\param[out]	pBusHandle				Pointer used to handle the created sbgCan library.
 *	\param[in]	location				The location of the interface to open.
 *	\param[in]	bitRate					The desired bit rate on the CAN bus.<br>
 *										Possible values are:
 *										<ul>
 *											<li>1 000 kBit/s</li>
 *											<li>  500 kBit/s</li>
 *											<li>  250 kBit/s</li>
 *											<li>  125 kBit/s</li>
 *											<li>  100 kBit/s</li>
 *											<li>   50 kBit/s</li>
 *											<li>   20 kBit/s</li>
 *											<li>   10 kBit/s</li>
 *										</ul>
 *	\return								SBG_NO_ERROR if we have initialised the bus system.
 */
SbgErrorCode sbgCanBusInit(SbgCanBusHandle *pBusHandle, const char *location, uint32 bitRate);

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pBusHandle				Pointer to the sbgCan library handle to close.
 *	\return								SBG_NO_ERROR if we have closed and released the protocol system.
 */
SbgErrorCode sbgCanBusClose(SbgCanBusHandle *pBusHandle);

/*!
 *	Flush all data and the communication with the CAN bus.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\return								SBG_NO_ERROR if the data has been flushed.
 */
SbgErrorCode sbgCanBusFlush(SbgCanBusHandle busHandle);

/*!
 *	Change the bit rate used for the communication with the interface.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	bitRate					New bit rate to use.
 *	\return								SBG_NO_ERROR if we have changed the can bit rate.
 */
SbgErrorCode sbgCanBusChangeBitRate(SbgCanBusHandle busHandle, uint32 bitRate);

/*!
 *	Set a mask to filter incoming frames.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	lowestId				The lowest id to be received by the protocol.
 *	\param[out]	highestId				The highest id to be received by the protocol.
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanBusSetFilter(SbgCanBusHandle busHandle, uint32 lowestId, uint32 highestId);

/*!
 *	Send a can frame over the bus.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	pMsg					Pointer to the can message to send
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanBusSendMessage(SbgCanBusHandle busHandle, const SbgCanMessageRaw *pMsg);

/*!
 *	Try to receive a specific frame from the interface within a timeout.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	pMsg					Pointer to the received message
 *	\param[in]	format					CAN message format (CAN_ID_STANDARD or CAN_ID_EXTENDED).
 *	\param[in]	receiveId				Id to be received.
 *	\param[in]	timeOut					Time in ms to wait before leaving receive mode.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanBusReceiveSpecificMessage(SbgCanBusHandle busHandle, SbgCanMessageRaw *pMsg, SbgCanMessageFormat format, uint32 receiveId, uint16 timeOut);

/*!
 *	Scan the interface while there is some message.<br>
 *	For each received frame, call the continuous frame management function.<br>
 *	This function returns only when no more frames are available in the interface reception buffer.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	pErrorCounter			Pointer to the reception error counter
 *	\return								SBG_NO_ERROR if all the frames has been read sucessfully.
 */
SbgErrorCode sbgCanBusContinuousReceptionHandle(SbgCanBusHandle busHandle, uint32 *pErrorCounter);

#endif	// __SBG_CAN_BUS_H__
