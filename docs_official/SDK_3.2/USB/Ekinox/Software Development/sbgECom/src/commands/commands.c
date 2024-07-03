#include "commands.h"
#include "../misc/sbgStreamBuffer.h"
#include "../misc/transfer.h"

//----------------------------------------------------------------------//
//- Helpers commands operations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Wait for an ACK for a specified amount of time.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						The command we would like the ACK for.
 *	\param[in]	timeOut						Time out in ms during which we can receive the ACK.
 *	\return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint16 command, uint32 timeOut)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	uint8 payload[2*sizeof(uint16)];
	SbgStreamBuffer inputStream;
	uint32 receivedSize;

	//
	// Try to receive the ACK
	//
	errorCode = sbgEComReceiveCmd(pHandle, SBG_ECOM_CMD_ACK, payload, &receivedSize, sizeof(payload), timeOut);

	//
	// Test if an ACK frame has been received
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Validate the received ACK frame
		//
		if (receivedSize == 2*sizeof(uint16))
		{
			//
			// Initialize a stream buffer to parse the received payload
			//
			sbgStreamBufferInitForRead(&inputStream, payload, sizeof(payload));

			//
			// The ACK frame contains two uint16, the command id of the acknoledge frame and the return error code
			// We make sure that the ACK is for the correct command
			//
			if (sbgStreamBufferReadUint16(&inputStream) == command)
			{
				//
				// Parse the error code and return it
				//
				errorCode = sbgStreamBufferReadUint16(&inputStream);
			}
			else
			{
				//
				// We have received an ACK but not for this frame!
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
		else
		{
			//
			// The ACK is invalid
			//
			errorCode = SBG_INVALID_FRAME;
		}
	}	

	return errorCode;
}

//----------------------------------------------------------------------//
//- Generic commands                                                   -//
//----------------------------------------------------------------------//

/*!
 *	Save the current settings configuration into FLASH memory.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComSaveSettings(SbgEComHandle *pHandle)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	uint32 trial;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Send the command three times
		//
		for (trial = 0; trial < 3; trial++)
		{
			//
			// Send the command
			//
			errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CMD_SAVE_SETTINGS, NULL, 0);

			//
			// Make sure that the command has been sent
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Try to read the device answer for 500 ms
				//
				errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CMD_SAVE_SETTINGS, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

				//
				// Test if we have received a valid ACK
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// The command has been executed successfully so return
					//
					break;
				}
			}
			else
			{
				//
				// We have a write error so exit the try loop
				//
				break;
			}
		}
	}
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Send a complete set of settings to the device and store them into the FLASH memory.
 *	The device will reboot automatically to use the new settings.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the settings.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComImportSettings(SbgEComHandle *pHandle, const void *pBuffer, uint32 size)
{
	SbgErrorCode		 errorCode = SBG_NO_ERROR;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Call function that handle data transfer
		//
		errorCode = sbgEComTransferSend(pHandle, SBG_ECOM_CMD_IMPORT_SETTINGS, pBuffer, size);
	}
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 *	Retrieve a complete set of settings from the device as a buffer.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Allocated buffer that can hold the received settings.
 *	\param[out]	pSize						The number of bytes that have been stored into pBuffer.
 *	\param[in]	maxSize						The maximum buffer size in bytes that can be stored into pBuffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComExportSettings(SbgEComHandle *pHandle, void *pBuffer, uint32 *pSize, uint32 maxSize)
{
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Call function that handle data transfer
		//
		errorCode = sbgEComTransferReceive(pHandle, SBG_ECOM_CMD_EXPORT_SETTINGS, pBuffer, pSize, maxSize);
	}
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

//----------------------------------------------------------------------//
//- Magnetometers commands                                             -//
//----------------------------------------------------------------------//

/*!
 *	Send a command that set the magnetometers calibration parameters.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	offset						Magnetometers calibration offset vector.
 *	\param[in]	matix						Magnetometers calibration 3x3 matrix.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComSetMagCalibData(SbgEComHandle *pHandle, const float offset[3], const float matrix[9])
{
	SbgErrorCode errorCode = SBG_NO_ERROR;
	SbgStreamBuffer outputStream;
	uint8 payload[12*sizeof(float)];
	uint32 trial;
	uint32 i;

	//
	// Test that the protocol handle is valid
	//
	if (pHandle)
	{
		//
		// Initialize a stream buffer to write the command payload
		//
		errorCode = sbgStreamBufferInitForWrite(&outputStream, payload, sizeof(payload));

		//
		// Write the offset vector
		//
		sbgStreamBufferWriteFloat(&outputStream, offset[0]);
		sbgStreamBufferWriteFloat(&outputStream, offset[1]);
		sbgStreamBufferWriteFloat(&outputStream, offset[2]);

		//
		// Write the matrix
		//
		for (i = 0; i < 9; i++)
		{
			sbgStreamBufferWriteFloat(&outputStream, matrix[i]);
		}

		//
		// Make sure that the stream buffer has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the command three times
			//
			for (trial = 0; trial < 3; trial++)
			{
				//
				// Send the command
				//
				errorCode = sbgEComProtocolSend(&pHandle->protocolHandle, SBG_ECOM_CMD_SET_MAG_CALIB, payload, sbgStreamBufferGetLength(&outputStream));

				//
				// Make sure that the command has been sent
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Try to read the device answer for 500 ms
					//
					errorCode = sbgEComWaitForAck(pHandle, SBG_ECOM_CMD_SET_MAG_CALIB, SBG_ECOM_DEFAULT_CMD_TIME_OUT);

					//
					// Test if we have received a valid ACK
					//
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// The command has been executed successfully so return
						//
						break;
					}
				}
				else
				{
					//
					// We have a write error so exit the try loop
					//
					break;
				}
			}
		}
	}
	else
	{
		//
		// Invalid protocol handle.
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}
