#include "sbgCanBus.h"
#include "sbgCanDevice.h"
#include "../time/sbgTime.h"

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
SbgErrorCode sbgCanSetRawMessage(SbgCanMessageRaw *pMsg, SbgCanMessageFormat format, uint32 id, uint8 length, const uint8 data[8])
{
	SbgErrorCode error;
	
	//
	// Check that we have a output parameter
	//
	if (pMsg)
	{
		//
		// Check that the message length is valid
		//
		if (length <= 8)
		{
			//
			// Fill the CAN message
			//
			pMsg->format = format;
			pMsg->id = id;
			pMsg->length = length;

			//
			// Copy the CAN messsage data field
			//
			memcpy(pMsg->data, data , sizeof(uint8)*length);

			error = SBG_NO_ERROR;
		}
		else
		{
			//
			// The message has an invalid size
			//
			error = SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

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
SbgErrorCode sbgCanBusInit(SbgCanBusHandle *pBusHandle, const char *location, uint32 bitRate)
{
	SbgErrorCode error;
	SbgCanInterfaceHandle canHandle;
	SbgCanBusHandle busHandle;
	
	//
	// Check if we can store the new handle
	//
	if (pBusHandle)
	{
		//
		// First set the returned handle to an invalid handle
		//
		*pBusHandle = SBG_CAN_INVALID_BUS_HANDLE;

		//
		// Try to open the device
		// pointer to location is tested in sbgCanInterfaceOpen
		//
		error = sbgCanInterfaceOpen(&canHandle, location, bitRate);

		//
		// Check if we have opened the device
		//
		if (error == SBG_NO_ERROR)
		{
			//
			// Create a new protocol handle
			//
			busHandle = (SbgCanBusHandle)malloc(sizeof(SbgCanBusHandleStr));

			//
			//	If we have a valid handle
			//
			if (busHandle)
			{
				//
				//	Init the new protocol handle
				//
				busHandle->canInterfaceHandle = canHandle;
			
				//
				//	Create the new devices list
				//
				busHandle->listLenght = 0;
				busHandle->pListTail = NULL;

				//
				//	Return the protocol handle
				//
				*pBusHandle = busHandle;
				error = SBG_NO_ERROR;
			}
			else
			{
				//
				// Allocation of the handle failed
				//
				error = SBG_MALLOC_FAILED;
				sbgCanInterfaceClose(&canHandle);
				free(busHandle);
			}
		}
	}
	else
	{
		//
		//	NULL pointer pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Close the protocol system and release associated memory.
 *	\param[in]	pBusHandle				Pointer to the sbgCan library handle to close.
 *	\return								SBG_NO_ERROR if we have closed and released the protocol system.
 */
SbgErrorCode sbgCanBusClose(SbgCanBusHandle *pBusHandle)
{
	if ( (pBusHandle) && (*pBusHandle != SBG_CAN_INVALID_BUS_HANDLE) )
	{
		//
		//	if there is no device opened
		//
		if ( (*pBusHandle)->pListTail == NULL)
		{
			//
			// Close the bus
			//
			sbgCanInterfaceClose( &((*pBusHandle)->canInterfaceHandle) );

			//
			// Release the bus handle
			//
			free(*pBusHandle);
			*pBusHandle = SBG_CAN_INVALID_BUS_HANDLE;

			return SBG_NO_ERROR;
		}
		else
		{
			//
			// There are still some devices attached to the bus
			//
			return SBG_OPERATION_CANCELLED;
		}
	}
	else
	{
		//
		// Invalid pointer to protocol handle
		//
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Flush all data and the communication with the CAN bus.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\return								SBG_NO_ERROR if the data has been flushed.
 */
SbgErrorCode sbgCanBusFlush(SbgCanBusHandle busHandle)
{
	SbgErrorCode error;

	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		// Flush the can device
		//
		error = sbgCanInterfaceFlush(busHandle->canInterfaceHandle);
	}
	else
	{
		//
		//	NULL handle pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Change the bit rate used for the communication with the interface.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	bitRate					New bit rate to use.
 *	\return								SBG_NO_ERROR if we have changed the can bit rate.
 */
SbgErrorCode sbgCanBusChangeBitRate(SbgCanBusHandle busHandle, uint32 bitRate)
{
	SbgErrorCode error;
	
	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		// Change the bit rate
		//
		error = sbgCanInterfaceChangeBitRate(busHandle->canInterfaceHandle, bitRate);
	}
	else
	{
		//
		//	NULL handle pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Set a mask to filter incoming frames.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	lowestId				The lowest id to be received by the protocol.
 *	\param[out]	highestId				The highest id to be received by the protocol.
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanBusSetFilter(SbgCanBusHandle busHandle, uint32 lowestId, uint32 highestId)
{
	SbgErrorCode error;
	
	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		// Receive a frame from the device
		//
		error = sbgCanInterfaceSetReceiveFilter(busHandle->canInterfaceHandle, lowestId, highestId);
	}
	else
	{
		//
		// NULL handle pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Send a can frame over the bus.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	pMsg					Pointer to the can message to send
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanBusSendMessage(SbgCanBusHandle busHandle, const SbgCanMessageRaw *pMsg)
{
	SbgErrorCode error;

	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		// Send the frame over the device
		//
		error = sbgCanInterfaceWriteFrame(busHandle->canInterfaceHandle, pMsg);
	}
	else
	{
		//
		//	NULL handle pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Try to receive a specific frame from the interface within a timeout.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	pMsg					Pointer to the received message
 *	\param[in]	format					CAN message format (CAN_ID_STANDARD or CAN_ID_EXTENDED).
 *	\param[in]	receiveId				Id to be received.
 *	\param[in]	timeOut					Time in ms to wait before leaving receive mode.
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanBusReceiveSpecificMessage(SbgCanBusHandle busHandle, SbgCanMessageRaw *pMsg, SbgCanMessageFormat format, uint32 receiveId, uint16 timeOut)
{
	SbgCanDeviceHandle deviceHandle;
	uint32 currentTime;
	SbgCanMessageRaw msg;
	SbgErrorCode error = SBG_TIME_OUT;

	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		//	Try to receive a frame within the time out
		//
		currentTime = sbgGetTime();

		while( sbgGetTime() < (currentTime + timeOut) )
		{
			//
			// Receive a frame from a device
			//
			error = sbgCanInterfaceReadFrame(busHandle->canInterfaceHandle, &msg);

			if (error == SBG_NO_ERROR)
			{
				//
				// Check if the frame has the desired frame format and identifier
				//
				if ( (msg.format == format) && (msg.id == receiveId) )
				{
					//
					// If possible, return the received CAN message
					//
					if (pMsg)
					{
						//
						// Copy the received CAN message
						//
						memcpy(pMsg, &msg, sizeof(SbgCanMessageRaw));
					}

					error = SBG_NO_ERROR;
					break;
				}
				else
				{
					//
					// Get the first device
					//
					deviceHandle = (SbgCanDeviceHandle)busHandle->pListTail;

					//
					// Treat the frame for each devices
					//
					while (deviceHandle)
					{
						sbgCanDeviceManageContinuousFrame(deviceHandle, &msg);
						
						//
						// Goto the next device
						//
						deviceHandle = deviceHandle->pPrev;
					}

					error = SBG_INVALID_FRAME;
				}
			}
		}
	}
	else
	{
		//
		//	NULL handle pass as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Scan the interface while there is some message.<br>
 *	For each received frame, call the continuous frame management function.<br>
 *	This function returns only when no more frames are available in the interface reception buffer.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[out]	pErrorCounter			Pointer to the reception error counter
 *	\return								SBG_NO_ERROR if all the frames has been read sucessfully.
 */
SbgErrorCode sbgCanBusContinuousReceptionHandle(SbgCanBusHandle busHandle, uint32 *pErrorCounter)
{
	SbgCanDeviceHandle deviceHandle;
	SbgCanMessageRaw msg;
	SbgErrorCode error;

	//
	// Check input pointers
	//
	if ((busHandle) && (pErrorCounter))
	{
		// Reset error counter
		*pErrorCounter = 0;

		do
		{
			error = sbgCanInterfaceReadFrame(busHandle->canInterfaceHandle, &msg);

			if (error == SBG_NO_ERROR)
			{
				deviceHandle = (SbgCanDeviceHandle) busHandle->pListTail;
				
				while (deviceHandle)
				{
					//
					//	Treat the frame if we have received one
					//
					sbgCanDeviceManageContinuousFrame(deviceHandle, &msg);
				
					deviceHandle = deviceHandle->pPrev;
				}
			}
			else
			{
				*pErrorCounter++;
			}

		}while(error != SBG_NOT_READY);
		
		error = SBG_NO_ERROR;
	}
	else
	{
		//
		//	NULL pointer pass as argument
		//
		error = SBG_NULL_POINTER;
	}
	
	return error;
}
