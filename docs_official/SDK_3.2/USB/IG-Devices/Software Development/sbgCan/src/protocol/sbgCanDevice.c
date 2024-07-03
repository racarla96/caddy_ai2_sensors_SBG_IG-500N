#include "sbgCanDevice.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Devices to bus operations										   -//
//----------------------------------------------------------------------//

/*!
 *	Calcul the receive bus CAN filter according to devices ids.
 *	\param[in]	protocolHandle			The protocol handle.
 *	\return								SBG_NO_ERROR if we have initialised the protocol system.
 */
SbgErrorCode sbgCanCalcFilter(SbgCanBusHandle busHandle)
{
	SbgCanDeviceHandle tmpDevice;
	uint8 i;
	uint32 lowestId;
	uint32 highestId;
	SbgErrorCode error;

	//
	// Check if we have a valid CAN bus handle
	//
	if (busHandle != SBG_CAN_INVALID_BUS_HANDLE)
	{
		//
		// Check if we have some devices in the list
		//
		if (busHandle->pListTail)
		{
			//
			// We will search for the lowest and highest ids for all attached devices
			// We start the search for the first device
			//
			tmpDevice = (SbgCanDeviceHandle) busHandle->pListTail;

			//
			// To start the min/max search, use the max extended CAN message id
			//
			lowestId = SBG_CAN_MAX_EXT_VALID_ID;
			highestId = 0x00000000;

			//
			// Search for the lowest and highest ids of the whole devices list
			//
			while (tmpDevice)
			{
				//
				// For each CAN message id of this device
				//
				for (i=0; i < SBG_CAN_ID_COUNT; i++)
				{
					//
					// First, check that this id isn't a disabled one
					// If it's a disabled id we don't have to use it for the min/max computation
					//
					if (tmpDevice->framesIdList[i].messageId != SBG_CAN_DISABLED_FRAME)
					{
						//
						// Check if the message id is smaller than the current lowest id
						//
						if (tmpDevice->framesIdList[i].messageId  < lowestId)
						{
							//
							// For now, the id is the smallest
							//
							lowestId = tmpDevice->framesIdList[i].messageId ;
						}
						else if (tmpDevice->framesIdList[i].messageId  > highestId)
						{
							//
							// For now, the id is the highest
							//
							highestId = tmpDevice->framesIdList[i].messageId ;
						}
					}
				}

				//
				// Goto the next device
				//
				tmpDevice = tmpDevice->pPrev;
			}

			//
			//	We have finish to search the lowest and highest ids
			//
			error = sbgCanInterfaceSetReceiveFilter(busHandle->canInterfaceHandle, lowestId, highestId);
		}
		else
		{
			//
			// There is no device attached to this CAN bus
			//
			error = SBG_DEVICE_NOT_FOUND;
		}
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

//----------------------------------------------------------------------//
//- Device communication operations									   -//
//----------------------------------------------------------------------//

/*!
 *	Add a device to the sbgCan library handle.
 *	\param[out]	pDeviceHandle			Pointer to the new device handle.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	userId					The list of user ids used for the device.
 *	\return								SBG_NO_ERROR if the device has been added properly.
 */
SbgErrorCode sbgCanAddDevice(SbgCanDeviceHandle *pDeviceHandle, SbgCanBusHandle busHandle, const SbgCanFramesIdList userId)
{
	SbgCanDeviceHandle deviceHandle;
	SbgCanDeviceHandle cpyDeviceHandle;
	SbgErrorCode error = SBG_NO_ERROR;
	uint32 maxCanId;
	uint32 canId;
	uint32 i;
	uint32 j;

	//
	// Check input parameters
	//
	if ( (pDeviceHandle) && (busHandle != SBG_CAN_INVALID_BUS_HANDLE) )
	{
		//
		// First set the returned handle to an invalid handle
		//
		*pDeviceHandle = SBG_CAN_INVALID_DEVICE_HANDLE;
		
		//
		// Create the new device handle
		//
		deviceHandle = (SbgCanDeviceHandle)malloc(sizeof(SbgCanDeviceHandleStr));

		//
		// Check if we were able to allocate memory for the new device handle
		//
		if (deviceHandle)
		{
			//
			// Initialize callback and user args to NULL
			//
			deviceHandle->pUserHandlerDeviceError = NULL;
			deviceHandle->pUserHandlerDeviceOutput = NULL;
			deviceHandle->pUserArgDeviceError = NULL;
			deviceHandle->pUserArgDeviceOutput = NULL;

			//
			// Set the pointer to the bus handle
			//
			deviceHandle->canBusHandle = busHandle;

			//
			// The new device isn't linked to any entry
			//
			deviceHandle->pNext = NULL;
			deviceHandle->pPrev = NULL;

			//
			// Insert the device into the chained devices list
			// We start by checking if the chained list already have some entries
			//
			if (busHandle->pListTail)
			{
				//
				// The list contains some entries so insert the new device at the tail
				// First, we backup the the device that was at the tail and we add the new entry at the tail
				//
				cpyDeviceHandle = (SbgCanDeviceHandleStr*)busHandle->pListTail;
				busHandle->pListTail = (SbgCanDeviceList*)deviceHandle;

				//
				// Update the old device to point to the new one
				//
				cpyDeviceHandle->pNext = deviceHandle;

				//
				// Update the new device to point to the old device
				//
				deviceHandle->pPrev = cpyDeviceHandle;
			}
			else
			{
				//
				//	If the list is empty add the device directly at the tail.
				//
				busHandle->pListTail = (SbgCanDeviceList*)deviceHandle;
			}

			//
			// We have added a new device so increase the list size by one
			//
			busHandle->listLenght++;

			//
			// Reset the pointer to the bus handle
			//
			deviceHandle->canBusHandle = busHandle;

			//
			// Set the ids list for the device
			//
			if (userId)
			{
				//
				// For each CAN frame, check it's id validity (except for inputs commands)
				//
				for (i=0; i<SBG_CAN_SEND_ID_FIRST; i++)
				{
					//
					// Get the CAN id for this frame
					//
					canId = userId[i].messageId;

					//
					// Get the max CAN id according to the CAN format
					//
					if (userId[i].messageFormat == SBG_CAN_ID_STANDARD)
					{
						maxCanId = SBG_CAN_MAX_STD_VALID_ID;
					}
					else
					{
						maxCanId = SBG_CAN_MAX_EXT_VALID_ID;
					}

					//
					// Check if the CAN id is valid according to CAN frame format (standard or extended)
					//
					if (canId<=maxCanId)
					{
						//
						// Check that we don't have two CAN frame with the same id (except for the inputs commands)
						//
						for (j=i+1; j<SBG_CAN_SEND_ID_FIRST ; j++)
						{
							//
							// Check that the canId isn't already present in the list
							//
							if (canId == userId[j].messageId)
							{
								error = SBG_INVALID_PARAMETER;
								break;
							}
						}
					}
					else
					{
						//
						// Invalid CAN id
						//
						error = SBG_INVALID_PARAMETER;
						break;
					}

					//
					// Check if we have to end the loop (ie we have an error)
					//
					if (error != SBG_NO_ERROR)
					{
						break;
					}
				}
				
				//
				// Check if we have an error during the initialization
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// Copy user ids
					//
					memcpy(deviceHandle->framesIdList, userId, sizeof(SbgCanFramesIdList));

					//
					//	Calcul the new filter
					//
					error = sbgCanCalcFilter(busHandle);

					//
					// Check if we were able to initialize the receive filter
					//
					if (error == SBG_NO_ERROR)
					{
						//
						// We have a valid device handle so return it
						//
						*pDeviceHandle = deviceHandle;
					}
				}
			}
			else
			{
				//
				//	Use default ids
				//
				for (i = 0; i < SBG_CAN_ID_COUNT; i++)
				{
					deviceHandle->framesIdList[i].messageFormat = SBG_CAN_ID_STANDARD;
					deviceHandle->framesIdList[i].messageId = i;
				}

				//
				//	Calcul the new filter
				//
				error = sbgCanCalcFilter(busHandle);

				//
				// Check if we were able to initialize the receive filter
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// We have a valid device handle so return it
					//
					*pDeviceHandle = deviceHandle;
				}
			}

			//
			// The device handle has been allocated and added to the chained list so check if we have an error
			// and if so, remove this device handle
			//
			if (error != SBG_NO_ERROR)
			{
				//
				// Free the device handle
				//
				sbgCanRemoveDevice(&deviceHandle);
				*pDeviceHandle = SBG_CAN_INVALID_DEVICE_HANDLE;
			}
		}
		else
		{
			//
			// Allocation of the device handle failed
			//
			error = SBG_MALLOC_FAILED;
			free(deviceHandle);
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
 *	Remove a device of the sbgCan library handle.
 *	\param[in]	pDeviceHandle			Pointer to the device handle to remove.	
 *	\return								SBG_NO_ERROR if the device has been removed properly.
 */
SbgErrorCode sbgCanRemoveDevice(SbgCanDeviceHandle *pDeviceHandle)
{
	SbgCanDeviceHandle tailDeviceEntry;
	SbgCanDeviceHandle deviceHandle;
	SbgCanBusHandle busHandle;
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid device handle pointer
	//
	if (pDeviceHandle)
	{
		//
		// Get the device handle to remove and the bus handle
		//
		deviceHandle = *pDeviceHandle;

		//
		// Check if the device handle is valid
		//
		if (deviceHandle)
		{
			//
			// Get the bus handle
			//
			busHandle = deviceHandle->canBusHandle;

			//
			// Check if we have a valid bus handle and if the list size is not equals to 0
			//
			if ( (busHandle) && (busHandle->listLenght>0) )
			{
				//
				// Get the tail device entry
				//
				tailDeviceEntry = (SbgCanDeviceHandle)busHandle->pListTail;

				//
				// Check if the device we would like to remove is the tail one or not
				//
				if (deviceHandle == tailDeviceEntry)
				{
					//
					// The device we would like to remove is a tail one.
					// So directly assign the previous device has the tail one
					//
					busHandle->pListTail = (SbgCanDeviceList*)deviceHandle->pPrev;

					//
					// If we had a previous device, it doesn't point to the device Handle anymore
					//
					if (deviceHandle->pPrev)
					{
						deviceHandle->pPrev->pNext = NULL;
					}
				}
				else
				{
					//
					// We are removing a device in the middle of the chained list
					// Check if the device has a previous entry
					//
					if (deviceHandle->pPrev)
					{
						//
						// The previous entry directly points to the device next entry
						//
						deviceHandle->pPrev->pNext = deviceHandle->pNext;
					}

					//
					// Check if the device has a next entry
					//
					if (deviceHandle->pNext)
					{
						//
						// The next entry directly points to the device previous entry
						//
						deviceHandle->pNext->pPrev = deviceHandle->pPrev;
					}
				}

				//
				// We have removed one item in the list so decrement it's size by one
				//
				busHandle->listLenght--;

				//
				// Release the device
				//
				free(deviceHandle);
				*pDeviceHandle = SBG_CAN_INVALID_DEVICE_HANDLE;

				//
				// Update the new reception filter
				//
				sbgCanCalcFilter(busHandle);

				//
				// We have no error
				//
				error = SBG_NO_ERROR;
			}
			else
			{
				//
				// Invalid bus handle or the chained list is empty!
				//
				error = SBG_INVALID_PARAMETER;
			}
		}
		else
		{
			//
			// Invalid device handle
			//
			error = SBG_NULL_POINTER;
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
 *	Defines the handle function to call when we have an error on a continuous frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	callback				Pointer to the error handler function. (could be NULL to remove the callback)
 *	\param[in]	pUserArg				User argument to pass to the error handler function.
 *	\return								SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgCanSetDeviceErrorCallback(SbgCanDeviceHandle deviceHandle, DeviceErrorCallback callback, void *pUserArg)
{
	//
	// Check if we have both a valid handle and a valid callback function
	//
	if (deviceHandle)
	{
		deviceHandle->pUserHandlerDeviceError = callback;
		deviceHandle->pUserArgDeviceError = pUserArg;
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Defines the handle function to call when we have received a valid continuous frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	callback				Pointer to the error handler function. (could be NULL to remove the callback)
 *	\param[in]	pUserArg				User argument to pass to the error handler function.
 *	\return								SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgCanSetDeviceOutputCallback(SbgCanDeviceHandle deviceHandle, DeviceOutputCallback callback, void *pUserArg)
{
	//
	// Check if we have both a valid handle and a valid callback function
	//
	if (deviceHandle)
	{
		deviceHandle->pUserHandlerDeviceOutput = callback;
		deviceHandle->pUserArgDeviceOutput = pUserArg;
		return SBG_NO_ERROR;
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Send a specific CAN message.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				Generique id used to identify which CAN message to send.
 *	\param[in]	length					CAN message data field length.
 *	\param[in]	pData					Data contained in the CAN message.
 *	\return								SBG_NO_ERROR if we were able to send the CAN message.
 */
SbgErrorCode sbgCanDeviceSendSpecificMessage(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, uint8 length, const uint8 *pData)
{
	SbgCanMessageRaw msg;

	//
	// Check that we have a valid device handle
	//
	if (deviceHandle)
	{
		//
		// Check that the default CAN id is valid
		//
		if (defaultId < SBG_CAN_ID_COUNT)
		{
			//
			// Build the CAN message
			//
			sbgCanSetRawMessage(&msg, deviceHandle->framesIdList[defaultId].messageFormat, deviceHandle->framesIdList[defaultId].messageId, length, pData);

			//
			// Send the CAN message
			//
			return sbgCanBusSendMessage(deviceHandle->canBusHandle, &msg);
		}
		else
		{
			return SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		return SBG_NULL_POINTER;
	}
}

/*!
 *	Receive a specific CAN message.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				Generique id used to identify which CAN message we should receive.
 *	\param[in]	timeOut					Time in ms to wait before leaving receive mode.
 *	\param[out]	pLength					Received CAN message length.
 *	\param[out]	pData					Receive CAN message data.
 *	\return								SBG_NO_ERROR if 
 */
SbgErrorCode sbgCanDeviceReceiveSpecificMessage(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, uint16 timeOut, uint8 *pLength, uint8 pData[8])
{
	SbgCanMessageRaw msg;
	SbgErrorCode errorCode;

	//
	// Check that we have a valid device handle
	//
	if (deviceHandle)
	{
		//
		// Check that the default CAN id is valid
		//
		if (defaultId < SBG_CAN_ID_COUNT)
		{
			//
			// Build the CAN message
			//
			errorCode = sbgCanBusReceiveSpecificMessage(deviceHandle->canBusHandle, &msg, deviceHandle->framesIdList[defaultId].messageFormat, deviceHandle->framesIdList[defaultId].messageId, timeOut);

			//
			// Check if we were able to receive the CAN message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// If possible, copy the length
				//
				if (pLength)
				{
					*pLength = msg.length;
				}
				
				//
				// If possible, copy the data
				//
				if ( (pData) && (msg.length>0) )
				{
					memcpy(pData, msg.data, msg.length*sizeof(uint8));
				}
			}
		}
		else
		{
			errorCode = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Internal Device operations										   -//
//----------------------------------------------------------------------//

/*!
 *	For a specific frame, get its user id and frame format.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				The default frame idbto get the user id and frame format.
 *	\param[out]	pFormat					The CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[out]	pUserId					The user id of the frame.
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanDeviceGetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, SbgCanMessageFormat *pFormat, uint32 *pUserId)
{
	SbgErrorCode error;

	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Check if the default id is valid
		//
		if (defaultId < SBG_CAN_ID_COUNT)
		{
			//
			// If possible, return the user id
			//
			if (pUserId)
			{
				*pUserId = deviceHandle->framesIdList[defaultId].messageId;
			}

			//
			// If possible, return the CAN format
			//
			if (pFormat)
			{
				*pFormat = deviceHandle->framesIdList[defaultId].messageFormat;
			}

			error = SBG_NO_ERROR;
		}
		else
		{
			//
			// The default id isn't a valid one
			//
			error = SBG_INVALID_PARAMETER;
		}
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
 *	Set a frame format and user id for a specific frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				The default id of frame for which to set the id.
 *	\param[in]	format					CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[in]	userId					The new user id of the frame.			
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanDeviceSetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, SbgCanMessageFormat format, uint32 userId)
{
	SbgErrorCode error = SBG_NO_ERROR;
	uint8 i;
	
	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Check if the default id is valid
		//
		if (defaultId < SBG_CAN_ID_COUNT)
		{
			//
			// Check if the user id isn't a disabled a frame
			//
			if (userId != SBG_CAN_DISABLED_FRAME)
			{
				//
				// Check if the user id is valid according to the frame format
				//
				if (format == SBG_CAN_ID_STANDARD)
				{
					if (userId>SBG_CAN_MAX_STD_VALID_ID)
					{
						//
						// The user id isn't a valid standard CAN id
						//
						return SBG_INVALID_PARAMETER;
					}
				}
				else
				{
					if (userId>SBG_CAN_MAX_EXT_VALID_ID)
					{
						//
						// The user id isn't a valid extended CAN id
						//
						return SBG_INVALID_PARAMETER;
					}
				}

				//
				// Check that the user id isn't already in use for an other frame
				//
				for (i=0; i<SBG_CAN_ID_COUNT ; i++)
				{
					//
					// Check that this id isn't already used by an other frame
					//
					if ( (defaultId != i) && (userId == deviceHandle->framesIdList[i].messageId) )
					{
						//
						// A frame has already this id
						//
						return SBG_INVALID_PARAMETER;
					}
				}
			}

			//
			// Set the new id value
			//
			deviceHandle->framesIdList[defaultId].messageId = userId;
			deviceHandle->framesIdList[defaultId].messageFormat = format;

			//
			// Compute new filter
			//
			sbgCanCalcFilter(deviceHandle->canBusHandle);
		}
		else
		{
			//
			// The default id isn't a valid one
			//
			error = SBG_INVALID_PARAMETER;
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
 *	Treat a received continuous frame and call the user device output callback if the received frame is valid.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	pMsg					Pointer to the received can message
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanDeviceManageContinuousFrame(SbgCanDeviceHandle deviceHandle, const SbgCanMessageRaw *pMsg)
{
	SbgErrorCode error = SBG_NO_ERROR;
	SbgCanOutputDataStr output;

	if ( (deviceHandle->pUserHandlerDeviceOutput) && (deviceHandle->pUserHandlerDeviceError) )
	{
		//
		// Test the message id with all the possible output frames id
		//
		error = sbgCanDeviceHandleOutputs(deviceHandle, pMsg, &output);

		//
		// Use callback functions
		//
		if (error == SBG_NO_ERROR)
		{
			deviceHandle->pUserHandlerDeviceOutput(deviceHandle, &output, deviceHandle->pUserArgDeviceOutput);
		}
		else if (error != SBG_NOT_CONTINUOUS_FRAME)
		{
			deviceHandle->pUserHandlerDeviceError(deviceHandle, error, deviceHandle->pUserArgDeviceError);
		}
	}
	else
	{
		error = SBG_OPERATION_CANCELLED;
	}
	return error;
}

/*!
 *	Check if a received message id and format are the same as the corresponding message.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	pMsg					Received message to check.
 *	\param[in]	defaultId				Default id within the SbgCanId list to check the message against.
 *	\return								TRUE if the received message corresponds to the one we have in the SbgCanId list.
 */
bool sbgCanCheckReceivedId(SbgCanDeviceHandle deviceHandle, const SbgCanMessageRaw *pMsg, SbgCanId defaultId)
{
	//
	// Check if the input parameters are valid
	//
	if ( (pMsg) && (defaultId < SBG_CAN_ID_COUNT) )
	{
		//
		// Check both the message id and the message type (extended or standard).
		//
		if ( (pMsg->id == deviceHandle->framesIdList[defaultId].messageId) && (pMsg->format == deviceHandle->framesIdList[defaultId].messageFormat) )
		{
			return TRUE;
		}
	}

	return FALSE;
}

/*!
 *	Treat the output according to their default id and convert the data if needed.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	pMsg					Pointer to the device output message.
 *	\param[out]	pOutput					Pointer to a structure that hold the output data and id
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanDeviceHandleOutputs(SbgCanDeviceHandle deviceHandle, const SbgCanMessageRaw *pMsg, SbgCanOutputDataStr *pOutput)
{
	SbgErrorCode error = SBG_NO_ERROR;
	uint64 timeUs;
	uint64 remainder;

	if (pOutput)
	{		
		//
		// Test the message id to see if it matches with one of the user id configured for the device
		//
		if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_TIMESTAMP_TRIGGER))
		{
			if (pMsg->length == sizeof(uint32)+sizeof(uint16))
			{
				//
				// Fill the time since reset structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_TIMESTAMP_TRIGGER;
				pOutput->outputData.timestampTrigger.timeSinceReset = sbgCanTargetToHostU32( *(uint32*)(pMsg->data) );
				pOutput->outputData.timestampTrigger.triggerMask = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_DEVICE_STATUS))
		{
			if (pMsg->length == sizeof(uint32))
			{
				//
				// Fill the device status
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_DEVICE_STATUS;
				pOutput->outputData.deviceStatusMask = sbgCanTargetToHostU32( *(uint32*)(pMsg->data) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_UTC_TIME))
		{
			if (pMsg->length == 3*sizeof(uint8)+5*sizeof(uint8))
			{
				//
				// Get the current UTC date
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_UTC_TIME;
				pOutput->outputData.utcTime.utcTimeYear = pMsg->data[0];
				pOutput->outputData.utcTime.utcTimeMonth = pMsg->data[1];
				pOutput->outputData.utcTime.utcTimeDay = pMsg->data[2];

				//
				// Get the time in micro seconds encoded into 5 bytes
				//
				timeUs = (((uint64)pMsg->data[3]) << 4*8) + (((uint64)pMsg->data[4]) << 3*8) + (((uint64)pMsg->data[5]) << 2*8) + (((uint64)pMsg->data[6]) << 8) + pMsg->data[7];

				//
				// Get hours from the time in micro seconds
				//
				pOutput->outputData.utcTime.utcTimeHours = (uint8)(timeUs/(1000000LL*3600));
				remainder = timeUs%(1000000LL*3600);

				//
				// Get minutes from the time in micro seconds
				//
				pOutput->outputData.utcTime.utcTimeMinute = (uint8)(remainder/(1000000*60));
				remainder = remainder%(1000000*60);
				
				//
				// Get the seconds from the time in micro seconds
				//
				pOutput->outputData.utcTime.utcTimeSecond = (uint8)(remainder/1000000);
				remainder = remainder%1000000;

				//
				// Get nano seconds from the time in micro seconds
				//
				pOutput->outputData.utcTime.utcTimeNanoSecond = (uint32)remainder*1000;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_QUATERNION))
		{
			if (pMsg->length == sizeof(uint16)*4)
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_QUATERNION;
				pOutput->outputData.quaternion[0] = sbgCanTargetToHostFrac16( *(int16*)(pMsg->data + 0*sizeof(int16)) );
				pOutput->outputData.quaternion[1] = sbgCanTargetToHostFrac16( *(int16*)(pMsg->data + 1*sizeof(int16)) );
				pOutput->outputData.quaternion[2] = sbgCanTargetToHostFrac16( *(int16*)(pMsg->data + 2*sizeof(int16)) );
				pOutput->outputData.quaternion[3] = sbgCanTargetToHostFrac16( *(int16*)(pMsg->data + 3*sizeof(int16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_EULER))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the euler angles from rad in 1e-4 to rad
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_EULER;
				pOutput->outputData.eulerAngle[0] = ( (float)sbgCanTargetToHost16( *(int16*)(pMsg->data + 0*sizeof(int16)) ) )*1e-4f;
				pOutput->outputData.eulerAngle[1] = ( (float)sbgCanTargetToHost16( *(int16*)(pMsg->data + 1*sizeof(int16)) ) )*1e-4f;
				pOutput->outputData.eulerAngle[2] = ( (float)sbgCanTargetToHost16( *(int16*)(pMsg->data + 2*sizeof(int16)) ) )*1e-4f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_HEADING))
		{
			if (pMsg->length == sizeof(uint32)*2)
			{
				//
				// Read both the Kalman enhanced heading and heading accuracy
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_HEADING;
				pOutput->outputData.eulerYaw.heading = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.eulerYaw.headingAcc = sbgCanTargetToHost32( *(uint32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GYROSCOPES))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the gyroscopes values from rad/s in 1e-3 to rad/s.
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GYROSCOPES;
				pOutput->outputData.gyroscopes[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.gyroscopes[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.gyroscopes[2] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) ) )*1e-3f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_DELTA_ANGLES))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the gyroscopes values from rad/s in 1e-3 to rad/s.
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_DELTA_ANGLES;
				pOutput->outputData.deltaAngle[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.deltaAngle[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.deltaAngle[2] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) ) )*1e-3f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_ACCELEROMETERS))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the accelerometers values from m/s^-2 in 1e-2 to m/s^-2
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_ACCELEROMETERS;
				pOutput->outputData.accelerometers[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-2f;
				pOutput->outputData.accelerometers[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-2f;
				pOutput->outputData.accelerometers[2] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) ) )*1e-2f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_MAGNETOMETERS))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the magnetomters values from A.U. in 1e-3 to A.U. 1e-3
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_MAGNETOMETERS;
				pOutput->outputData.magnetometers[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.magnetometers[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-3f;
				pOutput->outputData.magnetometers[2] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) ) )*1e-3f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_TEMPERATURES))
		{
			if (pMsg->length == sizeof(uint16)*2)
			{
				//
				// Convert the temperatures values from °C in 1e-2 to °C
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_TEMPERATURES;
				pOutput->outputData.temperatures[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-2f;
				pOutput->outputData.temperatures[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-2f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES))
		{
			if (pMsg->length == sizeof(uint16)*3)
			{
				//
				// Convert the gyroscopes temperatures values from °C in 1e-2 to °C
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES;
				pOutput->outputData.gyroTemperatures[0] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 0*sizeof(uint16)) ) )*1e-2f;
				pOutput->outputData.gyroTemperatures[1] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 1*sizeof(uint16)) ) )*1e-2f;
				pOutput->outputData.gyroTemperatures[2] = ( (float)sbgCanTargetToHost16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) ) )*1e-2f;
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_POSITION_1))
		{
			if (pMsg->length == 2*sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_POSITION_1;
				pOutput->outputData.position1.latitude = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.position1.longitude = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_POSITION_2))
		{
			if (pMsg->length == sizeof(int32)+2*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_POSITION_2;
				pOutput->outputData.position2.altitude = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.position2.horAccuracy = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32)) );
				pOutput->outputData.position2.vertAccuracy = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32) + sizeof (uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_VELOCITY_1))
		{
			if (pMsg->length == 2*sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_VELOCITY_1;
				pOutput->outputData.velocity1.Vx = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.velocity1.Vy = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_VELOCITY_2))
		{
			if (pMsg->length == sizeof(int32)+sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_VELOCITY_2;
				pOutput->outputData.velocity2.Vz = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.velocity2.Vacc = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GYROSCOPES_RAW))
		{
			if (pMsg->length == 3*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GYROSCOPES_RAW;
				pOutput->outputData.rawGyroValues[0] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data) );
				pOutput->outputData.rawGyroValues[1] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint16)) );
				pOutput->outputData.rawGyroValues[2] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_ACCELEROMETERS_RAW))
		{
			if (pMsg->length == 3*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_ACCELEROMETERS_RAW;
				pOutput->outputData.rawAcceleroValues[0] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data) );
				pOutput->outputData.rawAcceleroValues[1] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint16)) );
				pOutput->outputData.rawAcceleroValues[2] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_MAGNETOMETERS_RAW))
		{
			if (pMsg->length == 3*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_MAGNETOMETERS_RAW;
				pOutput->outputData.rawMagnetoValues[0] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data) );
				pOutput->outputData.rawMagnetoValues[1] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint16)) );
				pOutput->outputData.rawMagnetoValues[2] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_TEMPERATURES_RAW))
		{
			if (pMsg->length == 2*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_TEMPERATURES_RAW;
				pOutput->outputData.rawTemperatures[0] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data) );
				pOutput->outputData.rawTemperatures[1] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES_RAW))
		{
			if (pMsg->length == 3*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES_RAW;
				pOutput->outputData.rawGyroTemperatures[0] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data) );
				pOutput->outputData.rawGyroTemperatures[1] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(uint16)) );
				pOutput->outputData.rawGyroTemperatures[2] = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + 2*sizeof(uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_BAROMETER))
		{
			if (pMsg->length == sizeof(uint32)+sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_BAROMETER;
				pOutput->outputData.barometer.pressure = sbgCanTargetToHostU32( *(uint32*)(pMsg->data) );
				pOutput->outputData.barometer.altitude = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(uint32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_MAG_CALIB_DATA))
		{
			if (pMsg->length == 6*sizeof(uint8))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_MAG_CALIB_DATA;
				memcpy(pOutput->outputData.magCalibData, pMsg->data, 6*sizeof(uint8));
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_ODOMETER_VELOCITIES))
		{
			if (pMsg->length == sizeof(int32)*2)
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_ODOMETER_VELOCITIES;
				pOutput->outputData.odoRawVelocity[0] = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.odoRawVelocity[1] = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_INFO))
		{
			if (pMsg->length == sizeof(uint32)+2*sizeof(uint8))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_INFO;
				pOutput->outputData.gpsInfo.gpsTime =  sbgCanTargetToHostU32( *(uint32*)(pMsg->data) );
				pOutput->outputData.gpsInfo.gpsFlag =  *(uint8*)(pMsg->data + sizeof(uint32));
				pOutput->outputData.gpsInfo.gpsSatNum = *(uint8*)(pMsg->data + sizeof(uint32) + sizeof(uint8));
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_SVINFO))
		{
			if (pMsg->length == 4*sizeof(uint8)+2*sizeof(int8))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_SVINFO;
				pOutput->outputData.gpsSvInfo.gpsChannel		= pMsg->data[0];
				pOutput->outputData.gpsSvInfo.gpsId				= pMsg->data[1];
				pOutput->outputData.gpsSvInfo.gpsQualityFlag	= pMsg->data[2];
				pOutput->outputData.gpsSvInfo.sigStrength		= pMsg->data[3];
				pOutput->outputData.gpsSvInfo.azimuth			= pMsg->data[4];
				pOutput->outputData.gpsSvInfo.elevation			= pMsg->data[5];
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_POSITION_1))
		{
			if (pMsg->length == 2*sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_POSITION_1;
				pOutput->outputData.gpsRawPosition1.latitude = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsRawPosition1.longitude = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_POSITION_2))
		{
			if (pMsg->length == sizeof(int32)+2*sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_POSITION_2;
				pOutput->outputData.gpsRawPosition2.altitude = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsRawPosition2.horAccuracy = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32)) );
				pOutput->outputData.gpsRawPosition2.vertAccuracy = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32) + sizeof (uint16)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_VELOCITY_1))
		{
			if (pMsg->length == 2*sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_VELOCITY_1;
				pOutput->outputData.gpsRawVelocity1.Vx = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsRawVelocity1.Vy = sbgCanTargetToHost32( *(int32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_VELOCITY_2))
		{
			if (pMsg->length == sizeof(int32)+sizeof(uint16))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_VELOCITY_2;
				pOutput->outputData.gpsRawVelocity2.Vz = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsRawVelocity2.Vacc = sbgCanTargetToHostU16( *(uint16*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_COURSE))
		{
			if (pMsg->length == sizeof(int32)+sizeof(uint32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_COURSE;
				pOutput->outputData.gpsRawCourse.heading = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsRawCourse.headingAcc  = sbgCanTargetToHostU32( *(uint32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_GPS_TRUE_HEADING))
		{
			if (pMsg->length == sizeof(int32)+sizeof(uint32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_GPS_TRUE_HEADING;
				pOutput->outputData.gpsTrueHeading.heading = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
				pOutput->outputData.gpsTrueHeading.headingAcc  = sbgCanTargetToHostU32( *(uint32*)(pMsg->data + sizeof(int32)) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else if (sbgCanCheckReceivedId(deviceHandle, pMsg, SBG_CAN_ID_OUTPUT_HEAVE))
		{
			if (pMsg->length == sizeof(int32))
			{
				//
				// return the output structure
				//
				pOutput->outputId = SBG_CAN_ID_OUTPUT_HEAVE;
				pOutput->outputData.heave = sbgCanTargetToHost32( *(int32*)(pMsg->data) );
			}
			else
			{
				error = SBG_INVALID_FRAME;
			}
		}
		else
		{
			error = SBG_NOT_CONTINUOUS_FRAME;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}
