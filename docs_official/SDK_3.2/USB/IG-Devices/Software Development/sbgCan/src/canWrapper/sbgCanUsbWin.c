#include "sbgCanWrapper.h"
#include <windows.h>
#include "PCANBasic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//------------------------------------------------------------------------------//
//- SBG PCAN Interface convertion		                                       -//
//------------------------------------------------------------------------------//

/*!
 *	Convert the bit rate into a valid PCAN device bit rate.
 *	\param[out]	pCanBitRate			The corresponding PCAN bit rate.
 *	\param[in]	bitRate				The bit rate to convert.
 *	\return							SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanGetBitRate(TPCANBaudrate *pCanBitRate, uint32 bitRate)
{
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid output parameter
	//
	if (pCanBitRate)
	{	
		//
		//Get the corresponding PCAN bit rate
		//
		switch (bitRate)
		{
		case 1000: 
			*pCanBitRate = PCAN_BAUD_1M;
			break;
		case 500:
			*pCanBitRate = PCAN_BAUD_500K;
			break;
		case 250:
			*pCanBitRate = PCAN_BAUD_250K;
			break;
		case 125:
			*pCanBitRate = PCAN_BAUD_125K;
			break;
		case 100:
			*pCanBitRate = PCAN_BAUD_100K;
			break;
		case 50:
			*pCanBitRate = PCAN_BAUD_50K;
			break;
		case 20:
			*pCanBitRate = PCAN_BAUD_20K;
			break;
		case 10:
			*pCanBitRate = PCAN_BAUD_10K;
			break;
		default:
			error = SBG_INVALID_PARAMETER;
			break;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Convert the CAN device location into a valid PCAN usb channel.
 *	\param[out]	pCanUsbChannel		The corresponding PCAN channel.
 *	\param[in]	pLocation			The CAN channel to convert.
 *	\return							SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanGetPCanChannel(TPCANHandle *pCanUsbChannel, const char *pLocation)
{
	uint32 canChannel;
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid output parameter
	//
	if ( (pCanUsbChannel) && (pLocation) )
	{	
		//
		//Get the corresponding PCAN bit rate
		//
		if (sscanf(pLocation, "CAN%i", &canChannel) == 1)
		{
			switch (canChannel)
			{
			case 0: 
				*pCanUsbChannel = PCAN_USBBUS1;
				break;
			case 1:
				*pCanUsbChannel = PCAN_USBBUS2;
				break;
			case 2:
				*pCanUsbChannel = PCAN_USBBUS3;
				break;
			case 3:
				*pCanUsbChannel = PCAN_USBBUS4;
				break;
			case 4:
				*pCanUsbChannel = PCAN_USBBUS5;
				break;
			case 5:
				*pCanUsbChannel = PCAN_USBBUS6;
				break;
			case 6:
				*pCanUsbChannel = PCAN_USBBUS7;
				break;
			case 7:
				*pCanUsbChannel = PCAN_USBBUS8;
				break;
			default:
				*pCanUsbChannel = PCAN_NONEBUS;
				error = SBG_INVALID_PARAMETER;
				break;
			}
		}
		else
		{
			//
			// Unable to extract the CAN channel
			//
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Convert a PCAN usb channel into a CAN interface location.
 *	\param[out]	pLocation			The CAN interface path.
 *	\param[in]	canUsbChannel		The PCAN channel to convert.
 *	\return							SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanGetCanLocation(char *pLocation, TPCANHandle canUsbChannel)
{
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid output parameter
	//
	if (pLocation)
	{	
		//
		//Get the corresponding PCAN channel
		//
		switch (canUsbChannel)
		{
		case PCAN_USBBUS1: 
			sprintf(pLocation, "CAN0");
			break;
		case PCAN_USBBUS2:
			sprintf(pLocation, "CAN1");
			break;
		case PCAN_USBBUS3:
			sprintf(pLocation, "CAN2");
			break;
		case PCAN_USBBUS4:
			sprintf(pLocation, "CAN3");
			break;
		case PCAN_USBBUS5:
			sprintf(pLocation, "CAN4");
			break;
		case PCAN_USBBUS6:
			sprintf(pLocation, "CAN5");
			break;
		case PCAN_USBBUS7:
			sprintf(pLocation, "CAN6");
			break;
		case PCAN_USBBUS8:
			sprintf(pLocation, "CAN7");
			break;
		default:
			sprintf(pLocation, "NONE");
			error = SBG_INVALID_PARAMETER;
			break;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Convert the sbgCan message format into a valid PCAN message type.
 *	\param[out]	pCanMessageType			The corresponding PCAN message type.
 *	\param[in]	format					The CAN message format to convert.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanGetPCanMessageType(TPCANMessageType *pCanMessageType, SbgCanMessageFormat format)
{
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid output parameter
	//
	if (pCanMessageType)
	{
		//
		// Defines the PCAN message type
		//
		switch (format)
		{
		case SBG_CAN_ID_STANDARD:
			*pCanMessageType = PCAN_MESSAGE_STANDARD;
			break;
		case SBG_CAN_ID_EXTENDED:
			*pCanMessageType = PCAN_MESSAGE_EXTENDED;
			break;
		default:
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Convert the PCAN message type into a valid sbgCan message format.
 *	\param[out]	pFormat					The corresponding sbgCan message format.
 *	\param[in]	canMessageType			The PCAN message type to convert.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanGetCanMessageFormat(SbgCanMessageFormat *pFormat, TPCANMessageType canMessageType)
{
	SbgErrorCode error = SBG_NO_ERROR;

	//
	// Check if we have a valid output parameter
	//
	if (pFormat)
	{
		//
		// Defines the PCAN message type
		//
		switch (canMessageType)
		{
		case PCAN_MESSAGE_STANDARD:
			*pFormat = SBG_CAN_ID_STANDARD;
			break;
		case PCAN_MESSAGE_EXTENDED:
			*pFormat = SBG_CAN_ID_EXTENDED;
			break;
		default:
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

//------------------------------------------------------------------------------//
//- SBG Interface operations                                                   -//
//------------------------------------------------------------------------------//

/*!
 *	Open a PC usb interface at a specific location.
 *	\param[out]	pInterfaceHandle				Pointer to the interface handle.
 *	\param[in]	pLocation						The access to the interface to be initialized (from 0 to n).
 *	\param[in]	bitRate							The desired bit rate on the CAN bus.<br>
 *												Possible values are:
 *												<ul>
 *													<li>1 000 kBit/s</li>
 *													<li>  500 kBit/s</li>
 *													<li>  250 kBit/s</li>
 *													<li>  125 kBit/s</li>
 *													<li>  100 kBit/s</li>
 *													<li>   50 kBit/s</li>
 *													<li>   20 kBit/s</li>
 *													<li>   10 kBit/s</li>
 *												</ul>
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceOpen(SbgCanInterfaceHandle *pInterfaceHandle, const char *pLocation, uint32 bitRate)
{
	SbgCanInterfaceHandle canInterfaceHandle;
	TPCANHandle canUsbChannel;
	TPCANBaudrate canBitRate;
	SbgErrorCode error;

	//
	// Check if we can store the created CAN handle
	//
	if ( (pInterfaceHandle) && (pLocation) )
	{
		//
		// First set the returned handle to invalid
		//
		*pInterfaceHandle = SBG_CAN_INVALID_HANDLE;
		
		//
		// Convert both the input location into a valid PCAN channel and the bit rate into a PCAN bit rate
		//
		if ( (sbgCanGetPCanChannel(&canUsbChannel, pLocation) == SBG_NO_ERROR) && (sbgCanGetBitRate(&canBitRate, bitRate) == SBG_NO_ERROR) )
		{
			//
			// Check if the channel is available and not already initialized
			//
			if (CAN_GetStatus(canUsbChannel) == PCAN_ERROR_INITIALIZE)
			{
				//
				// Initialize the usb CAN channel
				//
				if (CAN_Initialize(canUsbChannel, canBitRate, 0, 0, 0) == PCAN_ERROR_OK)
				{
					
					//
					// Create the interface handle
					//
					canInterfaceHandle = (SbgCanInterfaceHandle)malloc(sizeof(SbgCanInterfaceHandleStr));

					if (canInterfaceHandle)
					{
						//
						//	Init the interface handle
						//
						canInterfaceHandle->hardwareHandle = malloc(sizeof(TPCANHandle));

						if (canInterfaceHandle->hardwareHandle)
						{
							*(TPCANHandle*)(canInterfaceHandle->hardwareHandle) = canUsbChannel;
							canInterfaceHandle->bitRate = bitRate;

							//
							// For now, set both the lowest and highest ids to 0
							//
							canInterfaceHandle->lowestId = 0;
							canInterfaceHandle->highestId = 0;

							//
							//	Wa have a valid interface handle so return it
							//
							*pInterfaceHandle = canInterfaceHandle;
							error = SBG_NO_ERROR;
						}
						else
						{
							//
							// Unable to create the interface handle
							//
							error = SBG_MALLOC_FAILED;
							free(canInterfaceHandle->hardwareHandle);
							free(canInterfaceHandle);
						}
					}
					else
					{
						//
						// Handle allocation failed
						//
						error = SBG_MALLOC_FAILED;
						free(canInterfaceHandle);
					}
				}
				else
				{
					//
					// Unable to initialize the interface
					//
					error = SBG_ERROR;
				}
			}
			else
			{
				//
				// Interface not available
				//
				error = SBG_DEVICE_NOT_FOUND;
			}
		}
		else
		{
			//
			// Wrong location or bit rate
			//
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// NULL pointer passed as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Close a PC usb interface.
 *	\param[in]	pInterfaceHandle		Pointer to the hanlde of the interface to be close.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceClose(SbgCanInterfaceHandle *pInterfaceHandle)
{
	TPCANHandle canHandle;
	SbgErrorCode error;

	//
	// First check if we have a valid interface handle
	//
	if ( (pInterfaceHandle) && (*pInterfaceHandle != SBG_CAN_INVALID_HANDLE) )
	{
		//
		//	Convert the pointer into a PCAN handle
		//
		canHandle = *(TPCANHandle*)( (*pInterfaceHandle)->hardwareHandle );

		//
		// Check if the PCAN device is initialized
		//
		if (CAN_GetStatus(canHandle) != PCAN_ERROR_INITIALIZE)
		{
			//
			// The PCAN channel is initialized so close it
			//
			if (CAN_Uninitialize(canHandle) == PCAN_ERROR_OK)
			{
				//
				//	Reset the pointer to the interface
				//
				free((*pInterfaceHandle)->hardwareHandle);
				free(*pInterfaceHandle);
				*pInterfaceHandle = SBG_CAN_INVALID_HANDLE;

				//
				// PCAN channel sucessfully uninitialized
				//
				error = SBG_NO_ERROR;
			}
			else
			{
				//
				// Unable to uninitialize the PCAN channel
				//
				error = SBG_ERROR;
			}
		}
		else
		{
			//
			// The PCAN channel isn't initialized
			//
			error = SBG_DEVICE_NOT_FOUND;
		}
	}
	else
	{
		//
		// Invalid pointer to interface handle
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Flush the interface rx and tx buffers (remove all old data).
 *	\param[in]	interfaceHandle			Hanlde of the interface to flush.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceFlush(SbgCanInterfaceHandle interfaceHandle)
{
	TPCANHandle canUsbChannel;
	SbgErrorCode error;

	//
	// Check if we have a valid interface handle
	//
	if (interfaceHandle != SBG_CAN_INVALID_HANDLE)
	{
		//
		// Convert the handle into a valid PCAN channel
		//
		canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

		//
		// Flush the interface corresponding to the channel
		//
		if (CAN_Reset(canUsbChannel) == PCAN_ERROR_OK)
		{
			//
			// Device has been successfully flushed
			//
			error = SBG_NO_ERROR;
		}
		else
		{
			//
			// Unable to reset the PCAN channel
			//
			error = SBG_ERROR;
		}
	}
	else
	{
		//
		// NULL handle passed as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Change the bit rate of the opened interface.
 *	\param[in]	interfaceHandle			Hanlde of the interface.
 *	\param[in]	bitRate					The new desired bit rate.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceChangeBitRate(SbgCanInterfaceHandle interfaceHandle, uint32 bitRate)
{
	TPCANHandle canUsbChannel;
	SbgErrorCode errorCode;
	WORD canBitRate;

	//
	// Check if we have a valid interface handle
	//
	if (interfaceHandle != SBG_CAN_INVALID_HANDLE)
	{
		//
		// Check that the current bitrate isn't already equal to the new bitrate
		//
		if (interfaceHandle->bitRate != bitRate)
		{
			//
			// Get the PCAN channel
			//
			canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

			//
			// Get the PCAN bitrate
			//
			if (sbgCanGetBitRate(&canBitRate, bitRate) == SBG_NO_ERROR)
			{
				//
				// Check if the PCAN device is initialized
				//
				if (CAN_GetStatus(canUsbChannel) != PCAN_ERROR_INITIALIZE)
				{
					//
					// Close the PCAN channel
					//
					if (CAN_Uninitialize(canUsbChannel) == PCAN_ERROR_OK)
					{
						//
						// For now, we have an uninitialized PCAN channel
						//
						*(TPCANHandle*)(interfaceHandle->hardwareHandle) = PCAN_NONEBUS;

						//
						// Re-open the PCAN channel with the new bitrate
						//
						if (CAN_Initialize(canUsbChannel, canBitRate, 0, 0, 0) == PCAN_ERROR_OK)
						{
							//
							// We have successfully re-opened the PCAN channel so update the interface settings
							//
							interfaceHandle->bitRate = bitRate;
							*(TPCANHandle*)(interfaceHandle->hardwareHandle) = canUsbChannel;

							//
							// We have successfully changed the bitrate
							//
							errorCode = SBG_NO_ERROR;
						}
						else
						{
							//
							// Unable to initialize the PCAN channel
							//
							errorCode = SBG_ERROR;
						}					
					}
					else
					{
						//
						// Unable to close the PCAN channel
						//
						errorCode = SBG_ERROR;
					}
				}
				else
				{
					//
					// The PCAN channel isn't initialized
					//
					errorCode = SBG_ERROR;
				}
			}
			else
			{
				//
				// We should have an invalid bitrate
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
		else
		{
			//
			// The new bitrate is equal to the current one so just return
			//
			errorCode = SBG_NO_ERROR;
		}
	}
	else
	{
		//
		// NULL handle passed as argument
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Get the status of the CAN bus.
 *	\param[in]	interfaceHandle					Handle of the interface.
 *	\return										A CAN status mask that represents all errors or 0 if no error.
 */
uint32 sbgCanInterfaceGetBusStatus(SbgCanInterfaceHandle interfaceHandle)
{
	TPCANHandle canUsbChannel;

	//
	// Check if we have a valid interface handle
	//
	if (interfaceHandle != SBG_CAN_INVALID_HANDLE)
	{
		//
		// Convert the handle into a valid PCAN channel
		//
		canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

		//
		// Get the bus error status
		//
		return (uint32)CAN_GetStatus(canUsbChannel);
	}
	else
	{
		//
		// NULL handle passed as argument
		//
		return SBG_CAN_ERROR_ILLNET;
	}
}

/*!
 *	Set a mask to filter received frames.
 *	\param[in]	interfaceHandle			Handle of the interface to read.
 *	\param[in]	lowestId				The lowest id to be received.
 *	\param[in]	highestId				The highest id to be received.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceSetReceiveFilter(SbgCanInterfaceHandle interfaceHandle, uint32 lowestId, uint32 highestId)
{
	TPCANHandle canUsbChannel;
	uint32 pcanFilter;
	SbgErrorCode error;

	//
	// Check if we have a valid interface handle
	//
	if (interfaceHandle != SBG_CAN_INVALID_HANDLE)
	{
		//
		// Check if we have a valid lowest and highest ids
		//
		if (lowestId <= highestId)
		{
			//
			//	Convert the handle into a valid PCAN channel
			//
			canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

			//
			// Close the previous mask
			//
			pcanFilter = PCAN_FILTER_CLOSE;

			if (CAN_SetValue(canUsbChannel, PCAN_MESSAGE_FILTER, &pcanFilter, sizeof(pcanFilter)) == PCAN_ERROR_OK)
			{
				//
				// Set the new message mask for the device
				// TODO: check standard/extended CAN format
				//
				if (CAN_FilterMessages(canUsbChannel, lowestId, highestId, PCAN_MODE_EXTENDED) == PCAN_ERROR_OK)
				{
					//
					// Filter mask set successfully
					//
					interfaceHandle->lowestId = lowestId;
					interfaceHandle->highestId = highestId;

					error = SBG_NO_ERROR;
				}
				else
				{
					//
					// Unable to set the desired filter mask
					//
					error = SBG_OPERATION_CANCELLED;
				}				
			}
			else
			{
				//
				// Unable to close the filter mask
				//
				error = SBG_OPERATION_CANCELLED;
			}
		}
		else
		{
			//
			// Lowest is is greater than highest id
			//
			error = SBG_INVALID_PARAMETER;
		}
	}
	else
	{
		//
		// NULL handle passed as argument
		//
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 *	Transmit a frame over the CAN bus.
 *	\param[in]	interfaceHandle			Handle of the interface to transmit.
 *	\param[in]	pMsg					Pointer to the can message to be sent.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceWriteFrame(SbgCanInterfaceHandle interfaceHandle, const SbgCanMessageRaw *pMsg)
{
	TPCANHandle canUsbChannel;
	TPCANMsg canMessage;
	TPCANStatus canError;
	SbgErrorCode error;

	//
	// Check if we have a valid interface handle and a valid message structure
	//
	if ( (interfaceHandle != SBG_CAN_INVALID_HANDLE) && (pMsg) )
	{
		//
		// Convert the handle into a valid PCAN channel
		//
		canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

		//
		// Check if the message length is less than 8 bytes
		//
		if (pMsg->length<=8)
		{
			//
			// Create the PCAN message
			//
			canMessage.ID = pMsg->id;
			canMessage.LEN = pMsg->length;

			//
			// Copy the PCAN message data
			//
			memcpy(canMessage.DATA, pMsg->data, sizeof(uint8)*pMsg->length);

			//
			// Convert the can format into a valid PCAN message type
			//
			error = sbgCanGetPCanMessageType(&canMessage.MSGTYPE, pMsg->format);

			//
			// Check if we were able to get a valid PCAN message type
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Send the CAN message
				//
				canError = CAN_Write(canUsbChannel, &canMessage);

				//
				// Check if the CAN message has been sent
				//
				if (canError == PCAN_ERROR_OK)
				{
					//
					// The message has been sent
					//
					error = SBG_NO_ERROR;
				}
				else if ( (canError == PCAN_ERROR_XMTFULL) || (canError == PCAN_ERROR_BUSOFF) )
				{
					//
					// Interface unable to transmit due to a full Tx buffer or a 'Bus Off' state
					//
					error = SBG_NOT_READY;
				}
				else
				{
					//
					// We have an unknow error
					//
					error = SBG_ERROR;
				}
			}
		}
		else
		{
			//
			// The CAN message length is too long
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

/*!
 *	Read a frame from the CAN bus.
 *	\param[in]	interfaceHandle			Hanlde of the interface to read.
 *	\param[out]	pMsg					Pointer to the can message to be read.
 *	\return								SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceReadFrame(SbgCanInterfaceHandle interfaceHandle, SbgCanMessageRaw *pMsg)
{
	TPCANHandle canUsbChannel;
	TPCANMsg canMessage;
	TPCANTimestamp canTimeStamp;
	TPCANStatus canError;
	SbgErrorCode error;

	//
	// Check if we have a valid interface handle and a valid message structure
	//
	if ( (interfaceHandle != SBG_CAN_INVALID_HANDLE) && (pMsg) )
	{
		//
		//	Convert the handle into a valid PCAN channel
		//
		canUsbChannel = *(TPCANHandle*)(interfaceHandle->hardwareHandle);

		//
		// Read the PCAN messages
		//
		canError = CAN_Read(canUsbChannel, &canMessage, &canTimeStamp);

		//
		// Check if the CAN message has been received
		//
		if (canError == PCAN_ERROR_OK)
		{
			//
			// Check if the received frame is valid
			//
			if (canMessage.LEN <= 8)
			{
				//
				// Fill the sbgCan message
				//
				pMsg->id = canMessage.ID;
				pMsg->length = canMessage.LEN;

				//
				// Copy the data buffer
				//
				memcpy(pMsg->data, canMessage.DATA, sizeof(uint8)*canMessage.LEN);

				//
				// Convert the received CAN message type into an sbgCan message format
				//
				if (sbgCanGetCanMessageFormat(&pMsg->format, canMessage.MSGTYPE) == SBG_NO_ERROR)
				{
					//
					// We have received a valid frame
					//
					error = SBG_NO_ERROR;
				}
				else
				{
					//
					// We have received a frame with an unsuported frame type
					//
					error = SBG_INVALID_FRAME;
				}
			}
			else
			{
				//
				// The received message has an invalid size
				//
				error = SBG_BUFFER_OVERFLOW;
			}
		}
		else if (canError == PCAN_ERROR_QRCVEMPTY)
		{
			//
			// Interface unable to read due to an empty Rx buffer
			//
			error = SBG_NOT_READY;
		}
		else
		{
			//
			// We have an unknow error
			//
			error = SBG_ERROR;
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}