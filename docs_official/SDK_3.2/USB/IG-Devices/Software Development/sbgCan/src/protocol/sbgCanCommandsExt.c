#include "sbgCanCommandsExt.h"
#include "sbgCanProtocolOutputMode.h"
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
//- External device specific commands definitions                      -//
//----------------------------------------------------------------------//
#define SBG_CAN_EXT_DEVICE_CONF_SIZE				(0xFE)					/*!< Used with SBG_CAN_EXT_DEVICE_CONF command to initiate a new command transmission and pass the command size. */
#define SBG_CAN_EXT_DEVICE_CONF_END					(0xFF)					/*!< Used with SBG_CAN_EXT_DEVICE_CONF command to indicate that we have finished the command transmission. */
#define SBG_CAN_EXT_DEVICE_CONF_INVALID_SIZE		(0xFFFF)				/*!< Invalid frame size for SBG_CAN_EXT_DEVICE_CONF_SIZE sub command. */
#define SBG_CAN_EXT_DEVICE_CONF_MAX_DATA_LENGTH		(500)					/*!< Maximum size of the data part of the frame. */

//----------------------------------------------------------------------//
//- External device commands                                           -//
//----------------------------------------------------------------------//

/*!
 *	Select the external device connected to the IG-500E and its secondary UART configuration
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	deviceType						The device type connected
 *  \param[in]	baudRate						The baud rate used to communicate with the device at initialization
 *  \param[in]	uartOptions						Some uart Options needed to communicate with the device
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetExtDevice(SbgCanDeviceHandle deviceHandle, SbgCanExtDeviceType deviceType, uint32 baudRate, uint16 uartOptions)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)+sizeof(uint32)+sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	*(dataBuffer) = (uint8)deviceType;
	*(uint32*)(dataBuffer+sizeof(uint8)) = sbgCanHostToTarget32(baudRate);
	*(uint16*)(dataBuffer+sizeof(uint8)+sizeof(uint32)) = sbgCanHostToTarget16(uartOptions);

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it, it could be long to change the device type so let 1.5 seconds for command completion
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE, 1500, &receivedLength, receivedData);

		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength != sizeof(dataBuffer))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, sizeof(dataBuffer)) != 0)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Get the external device connected to the IG-500E and its secondary UART configuration
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[out]	pDeviceType						The device type connected
 *  \param[out]	pBaudRate						The baud rate used to communicate with the device at initialization
 *  \param[out]	pUartOptions					Some uart Options used to communicate with the device
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetExtDevice(SbgCanDeviceHandle deviceHandle, SbgCanExtDeviceType *pDeviceType, uint32 *pBaudRate, uint16 *pUartOptions)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Set and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE, 0, NULL);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(uint8)+sizeof(uint32)+sizeof(uint16))
			{
				//
				// Fill the device type if needed
				//
				if (pDeviceType)
				{
					*pDeviceType = (SbgCanExtDeviceType)receivedData[0];
				}

				//
				// Fill the baud rate if needed
				//
				if (pBaudRate)
				{
					*pBaudRate = sbgCanTargetToHost32(*(uint32*)(receivedData+sizeof(uint8)));
				}

				//
				// Fill the uart options if needed
				//
				if (pUartOptions)
				{
					*pUartOptions = sbgCanTargetToHost16(*(uint16*)(receivedData+sizeof(uint8)+sizeof(uint32)));
				}
			}
			else
			{
				//
				//	Invalid frame was received
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Send to the external device a specific configuration.<br>
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	pCommand						Command sent to the external device.
 *  \param[in]	commandSize						Size of the command to send.
 *  \param[out]	pAnswer							Answer of the external device.
 *  \param[out]	pAnswerSize						Size of the device answer.
 *  \param[in]	answerMaxSize					Maximum allowed size of the answer.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtDeviceConfig(SbgCanDeviceHandle deviceHandle, const uint8 *pCommand, uint16 commandSize, uint8 *pAnswer, uint16 *pAnswerSize, uint16 answerMaxSize)
{
	//
	// This command require three steps:
	//	- Send SBG_CAN_EXT_DEVICE_CONF with SBG_CAN_EXT_DEVICE_CONF_SIZE and the size of the command
	//	- Send SBG_CAN_EXT_DEVICE_CONF with the command payload (max 7 bytes per CAN message).
	//	- Send SBG_CAN_EXT_DEVICE_CONF with SBG_CAN_EXT_DEVICE_CONF_END to indicate that the command has been transmitted
	//	  and we are ready to receive the answer.
	//	- Wait for the answer that uses the same process.
	//
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[8];
	SbgErrorCode errorCode;
	uint8 *pTempAnswerBuffer;
	uint32 numBlocks;
	uint32 numBytesLeft;
	uint16 sizeToReceive;
	uint16 receivedSize;
	int16 lastReceivedIndex;
	uint8 receivedCmdIndex;
	bool receptionComplete;
	uint32 i;

	//
	// Check that we have a valid command
	//
	if (pCommand)
	{
		//
		// Check that the buffer to send has a valid length
		//
		if (commandSize <= SBG_CAN_EXT_DEVICE_CONF_MAX_DATA_LENGTH)
		{
			//
			// Fill the CAN message data buffer for SBG_CAN_EXT_DEVICE_CONF_SIZE
			//
			*(uint8*)(dataBuffer) = SBG_CAN_EXT_DEVICE_CONF_SIZE;
			*(uint16*)(dataBuffer+sizeof(uint8)) = sbgCanHostToTarget16(commandSize);

			//
			// Create the CAN message and send it
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE_CONF, sizeof(uint8)+sizeof(uint16), dataBuffer);

			//
			// Check that the CAN message has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Compute how many blocks we will have to send and the number of bytes left in the last incomplete CAN message
				//
				numBlocks = commandSize/7;
				numBytesLeft = commandSize-numBlocks*7;

				//
				// Send all the command using block of 7 bytes
				//
				for (i=0; i<numBlocks; i++)
				{
					//
					// Fill the CAN message, the first uint8 is the block index
					//
					*(uint8*)(dataBuffer) = (uint8)i;

					//
					// Fill the payload
					//
					memcpy(dataBuffer+sizeof(uint8), pCommand+i*7*sizeof(uint8), 7*sizeof(uint8));
					
					//
					// Create and send the command
					//
					errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE_CONF, sizeof(dataBuffer), dataBuffer);

					//
					// Check if we were able to send the command
					//
					if (errorCode != SBG_NO_ERROR)
					{
						return errorCode;
					}
				}

				//
				// Check if we have some bytes left to send
				//
				if (numBytesLeft>0)
				{
					//
					// Fill the CAN message, the first uint8 is the block index
					//
					*(uint8*)(dataBuffer) = (uint8)numBlocks;

					//
					// Fill the payload
					//
					memcpy(dataBuffer+sizeof(uint8), pCommand+numBlocks*7*sizeof(uint8), numBytesLeft*sizeof(uint8));
					
					//
					// Create and send the command
					//
					errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE_CONF, sizeof(uint8)+(uint8)numBytesLeft*sizeof(uint8), dataBuffer);

					//
					// Check if we were able to send the command
					//
					if (errorCode != SBG_NO_ERROR)
					{
						return errorCode;
					}
				}

				//
				// Fill the CAN message data buffer for SBG_CAN_EXT_DEVICE_CONF_END
				//
				*(uint8*)(dataBuffer) = SBG_CAN_EXT_DEVICE_CONF_END;

				//
				// Create the CAN message and send it
				//
				errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE_CONF, sizeof(uint8), dataBuffer);

				//
				// Check if we were able to send the CAN message
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Initialize the reception size
					//
					sizeToReceive = SBG_CAN_EXT_DEVICE_CONF_INVALID_SIZE;
					pTempAnswerBuffer = NULL;
					lastReceivedIndex = -1;
					receivedSize = 0;
					receptionComplete = FALSE;
					
					//
					// We will try to receive all the answer frames
					//
					do
					{
						//
						// Wait for the answer
						//
						errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_EXT_DEVICE_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

						//
						// Check if we have received a valid frame
						//
						if (errorCode == SBG_NO_ERROR)
						{
							//
							// Check that the received frame is at least 1 byte long (the smallest frame we could receive)
							//
							if (receivedLength>=sizeof(uint8))
							{
								//
								// Get the frame command or index
								//
								receivedCmdIndex = *(uint8*)(receivedData);

								//
								// Check if we are trying to receive the size frame
								//
								if (sizeToReceive == SBG_CAN_EXT_DEVICE_CONF_INVALID_SIZE)
								{
									//
									// We are trying to receive the size so check if the message is valid
									//
									if (receivedLength == sizeof(uint8)+sizeof(uint16))
									{
										//
										// Extract the size of the answer
										//
										sizeToReceive = *(uint16*)(dataBuffer+sizeof(uint8)) = sbgCanTargetToHost16(*(uint16*)(receivedData+sizeof(uint8)));

										//
										// Check if the received command and size are valid
										//
										if ( (receivedCmdIndex == SBG_CAN_EXT_DEVICE_CONF_SIZE) && (sizeToReceive <= SBG_CAN_EXT_DEVICE_CONF_MAX_DATA_LENGTH) )
										{
											//
											// Allocate a buffer to hold the answer
											//
											pTempAnswerBuffer = (uint8*)malloc(sizeToReceive*sizeof(uint8));
										}
										else
										{
											errorCode = SBG_INVALID_FRAME;
										}
									}
									else
									{
										errorCode = SBG_INVALID_FRAME;
									}
								}
								else
								{
									//
									// Check the frame size to know if it's a DATA frame or a END frame
									//
									if ( (receivedLength > sizeof(uint8)) && (receivedCmdIndex != SBG_CAN_EXT_DEVICE_CONF_SIZE) && (receivedCmdIndex != SBG_CAN_EXT_DEVICE_CONF_END) )
									{
										//
										// We have received a DATA frame so check if the index is coherent
										//
										if (receivedCmdIndex == lastReceivedIndex+1)
										{
											//
											// We have received the next data block so update both the received size and last received index
											//
											receivedSize += receivedLength-sizeof(uint8);
											lastReceivedIndex = receivedCmdIndex;

											//
											// Check that the received size isn't greater than the buffer
											//
											if (receivedSize<=sizeToReceive)
											{
												memcpy(pTempAnswerBuffer+receivedCmdIndex*7*sizeof(uint8), receivedData+sizeof(uint8), receivedLength-sizeof(uint8));
											}
										}
										else if (receivedCmdIndex == lastReceivedIndex)
										{
											//
											// We have already received this data block so just re-write this chunck of data
											//
											if (receivedSize<=sizeToReceive)
											{
												memcpy(pTempAnswerBuffer+receivedCmdIndex*7*sizeof(uint8), receivedData+sizeof(uint8), receivedLength-sizeof(uint8));
											}
										}
										else
										{
											//
											// We have missed some data
											//
											errorCode = SBG_READ_ERROR;
										}
									}
									else if ( (receivedLength == sizeof(uint8)) && (receivedCmdIndex == SBG_CAN_EXT_DEVICE_CONF_END) )
									{
										//
										// We have received a END frame, check if we have received all the data
										//
										if (receivedSize == sizeToReceive)
										{
											//
											// Check if the user buffer is big enough to hold the received frame
											//
											if (receivedSize<=answerMaxSize)
											{
												//
												// Copy the received data if possible
												//
												if ( (pAnswerSize) && (pAnswer) )
												{
													//
													// Copy the recevied size and buffer
													//
													*pAnswerSize = receivedSize;
													memcpy(pAnswer, pTempAnswerBuffer, sizeof(uint8)*receivedSize);

													//
													// We have received the answer successfully!
													//
													receptionComplete = TRUE;
												}
												else
												{
													errorCode = SBG_NULL_POINTER;
												}
											}
											else
											{
												//
												// The user answer buffer is too small to hold the received frame
												//
												errorCode = SBG_BUFFER_OVERFLOW;
											}
										}
										else
										{
											//
											// We have missed some data
											//
											errorCode = SBG_INVALID_FRAME;
										}
									}
									else
									{
										//
										// We have received an invalid frame
										//
										errorCode = SBG_INVALID_FRAME;
									}
								}
							}
							else
							{
								//
								// The received frame is invalid
								//
								errorCode = SBG_INVALID_FRAME;
							}
						}
					} while ( (errorCode == SBG_NO_ERROR) && (!receptionComplete) );

					//
					// Free the allocated buffer
					//
					SBG_FREE_ARRAY(pTempAnswerBuffer);
				}
			}
		}
		else
		{
			//
			// The buffer to send is too long
			//
			errorCode = SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		//
		//	NULL handle pass as argument
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}
