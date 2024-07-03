#include "sbgCanCommandsFilter.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Kalman filter commands											   -//
//----------------------------------------------------------------------//
/*!
 * Defines the kalman filter motion profile to be used
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pMpBuffer						Motion profile buffer pointer
 * \param[in]	mpSize							Motion profile buffer size
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanSetMotionProfile(SbgCanDeviceHandle deviceHandle, void *pMpBuffer, uint16 mpSize)
{
	SbgErrorCode error = SBG_NO_ERROR;
	const uint16 subBufferSize = 8-2;
	uint16 numBuffers;
	uint16 i;
	uint16 index;
	uint16 size;
	uint16 trial;

	//
	// Check input pointer
	//
	if (pMpBuffer)
	{
		//
		// In order to comply with low level protocol, we have to split the buffer in several parts
		// Each sub buffer will contain 8 bytes maximum including index
		//
		numBuffers = mpSize / subBufferSize;
		if (mpSize % subBufferSize)
		{
			numBuffers++;
		}

		//
		// Send each sub buffer to the IG-Device
		//
		for (i = 0; i < numBuffers; i++)
		{
			//
			// Compute the index of the sub buffer to transmit
			//
			index = i*subBufferSize;
		
			//
			// Compute the size in bytes of the sub buffer to transmit
			//
			if (i < (numBuffers - 1))
			{
				size = subBufferSize;
			}
			else
			{
				size = mpSize % subBufferSize;
			}

			//
			// Now send the sub buffer and wait for the answer
			// We have three tries to send this sub buffer
			//
			for (trial = 0; trial < 3; trial++)
			{
				//
				// Send the sub buffer
				//
				error = sbgCanSendMPBuffer(deviceHandle, ((uint8*)pMpBuffer) + index, index, size);

				//
				// Test if the sub buffer has been sent
				//
				if (error == SBG_NO_ERROR)
				{
					//
					// Sub buffer successfully sent so exit the trial loop
					//
					break;
				}
			}

			//
			// Test if the sub buffer has been sent successfully
			//
			if (error != SBG_NO_ERROR)
			{
				//
				// An error has occured while sending the sub buffer so exit
				//
				break;
			}
		}

		//
		// Now if everything went allright, try to validate the buffer and return answer
		//
		if (error == SBG_NO_ERROR)
		{
			error = sbgCanValidateMPBuffer(deviceHandle);
		}
	}
	else
	{
		error = SBG_NULL_POINTER;
	}

	return error;
}

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Sends a motion profile buffer part
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pMpBuffer						Motion profile buffer pointer
 * \param[in]	index							Index in the IG-500 motion profile buffer
 * \param[in]	size							Number of bytes to transmit
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanSendMPBuffer(SbgCanDeviceHandle deviceHandle, void *pMpBuffer, uint16 index, uint16 size)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[8];
	SbgErrorCode errorCode;
	
	//
	// Check if we have an input pointer
	//
	if (pMpBuffer)
	{
		//
		// Check if size is consistent with a CAN message size
		//
		if (size <= (sizeof(dataBuffer) - sizeof(uint16)))
		{
			//
			// Fill the CAN message data
			//
			*(uint16*)dataBuffer = sbgCanHostToTargetU16(index);
			memcpy(dataBuffer+sizeof(uint16), pMpBuffer, size);

			//
			// Create and send the CAN command
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_MP_BUFFER, (uint8)size + sizeof(uint16), dataBuffer);

			//
			// Check if we were able to send the CAN message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Wait for an answer and return it
				//
				errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_MP_BUFFER, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
		
				//
				// Check if we were able to receive the CAN message
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Check if the answer is valid
					//
					if (receivedLength != (size + sizeof(uint16))) 
					{
						//
						// The answer length is invalid
						//
						errorCode = SBG_INVALID_FRAME;
					}
					else if ( memcmp(receivedData, dataBuffer, size + sizeof(uint16)) != 0)
					{
						//
						// The received buffer isn't the same as the sent one
						//
						errorCode = SBG_INVALID_PARAMETER;
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
			errorCode = SBG_BUFFER_OVERFLOW;
		}
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 * Low level function. Users should call sbgSetMotionProfile <br>
 * Once the buffer is fully sent, Validate and use a motion profile buffer
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanValidateMPBuffer(SbgCanDeviceHandle deviceHandle)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_VALIDATE_MP_BUFFER, 0, NULL);
		
	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_VALIDATE_MP_BUFFER, 2000, &receivedLength, receivedData);
		
		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == sizeof(uint8))
			{
				//
				// Convert code returned into error code if there was an error
				//
				if (receivedData[0] != SBG_CAN_OPERATION_SUCCESS)
				{
					errorCode = SBG_ERROR;
				}
			}
			else
			{
				//
				// Answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 * Retrives the kalman filter motion profile information
 *	\param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	pId								Motion profile Identifier is returned here
 * \param[in]	pVersion						Motion profile version is returned here
 * \return										SBG_NO_ERROR in case of good operation
 */
SbgErrorCode sbgCanGetMotionProfileInfo(SbgCanDeviceHandle deviceHandle, uint32 *pId, uint32 *pVersion)
{
	uint8 receivedLength;
	uint32 receivedData[2];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_MP_INFO, 0, NULL);
		
	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_MP_INFO, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, (uint8*)receivedData);
		
		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == 8*sizeof(uint8))
			{
				//
				// If possible, return the motion profile parameters
				//
				if (pId)
				{
					*pId = sbgCanTargetToHostU32(receivedData[0]);
				}
				if (pVersion)
				{
					*pVersion = sbgCanTargetToHostU32(receivedData[1]);
				}
			}
			else
			{
				//
				// Answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Set the external source for heading.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	source							The heading source.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetFilterHeadingSource(SbgCanDeviceHandle deviceHandle, SbgCanHeadingSource source)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)];
	SbgErrorCode errorCode;
	
	//
	// Fill the CAN message data
	//
	*dataBuffer = (uint8)source;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_FILTER_HEADING_SOURCE, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_FILTER_HEADING_SOURCE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
		
		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint8)) 
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if ( memcmp(receivedData, dataBuffer, sizeof(uint8)) != 0)
			{
				//
				// The received buffer isn't the same as the sent one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}
	
/*!
 *	Get the external source for heading.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pSource							Pointer to an uint8 used to hold the heading source.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetFilterHeadingSource(SbgCanDeviceHandle deviceHandle, SbgCanHeadingSource *pSource)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_FILTER_HEADING_SOURCE, 0, NULL);
		
	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_FILTER_HEADING_SOURCE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
		
		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == sizeof(uint8))
			{
				//
				// If possible, return the heading source
				//
				if (pSource)
				{
					*pSource = (SbgCanHeadingSource)receivedData[0];
				}
			}
			else
			{
				//
				// Answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}	

/*!
 *	Set the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	declination						The north magnetic declination in radians.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetMagneticDeclination(SbgCanDeviceHandle deviceHandle, float declination)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data
	//
	*((uint32*)dataBuffer) = sbgCanHostToTargetFixed32(declination);

	//
	// Create and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_MAGNETIC_DECLINATION, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_MAGNETIC_DECLINATION, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
		
		//
		// Check if we were able to receive a valid answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint32)) 
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if ( memcmp(receivedData, dataBuffer, sizeof(uint32)) != 0)
			{
				//
				// The received buffer isn't the same as the sent one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Get the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDeclination					Pointer to a float used to hold the north magnetic declination in radians.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetMagneticDeclination(SbgCanDeviceHandle deviceHandle, float *pDeclination)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;
	
	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_MAGNETIC_DECLINATION, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_MAGNETIC_DECLINATION, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == sizeof(uint32))
			{
				//
				// If possible, return the converted data
				//
				if (pDeclination)
				{
					*pDeclination = sbgCanTargetToHostFloat( *(uint32*)(receivedData) );
				}
			}
			else
			{
				//
				// Answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}


/*!
 *	Defines the Heave configuration
 *	\param[in]	deviceHandle			A valid sbgCan library handle.
 *	\param[in]	enableHeave				Set to true if heave has to be computed
 *	\param[in]	heavePeriod				Set the average heave period. Must be between 0.3 and 20.
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetHeaveConf(SbgCanDeviceHandle deviceHandle, bool enableHeave, float heavePeriod)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[8];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data
	//
	memset(dataBuffer, 0, sizeof(dataBuffer));
	dataBuffer[0] = enableHeave;
	*((uint32*)(dataBuffer+sizeof(uint8))) = sbgCanHostToTargetFixed32(heavePeriod);

	//
	// Create and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_HEAVE_CONF, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_HEAVE_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
		
		//
		// Check if we were able to receive a valid answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(dataBuffer)) 
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if ( memcmp(receivedData, dataBuffer, sizeof(dataBuffer)) != 0)
			{
				//
				// The received buffer isn't the same as the sent one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Returns the heave configuration
 *	\param[in]	deviceHandle			A valid sbgCan library handle.
 *	\param[out]	pEnableHeave			Set to true if heave has to be computed
 *	\param[out]	pHeavePeriod			Average heave period if heave computation is enabled
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetHeaveConf(SbgCanDeviceHandle deviceHandle, bool *pEnableHeave, float *pHeavePeriod)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;
	
	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_HEAVE_CONF, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_HEAVE_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == 8*sizeof(uint8))
			{
				//
				// If possible, return the converted data
				//
				if (pEnableHeave)
				{
					*pEnableHeave = (bool)receivedData[0];
				}
				if (pHeavePeriod)
				{
					*pHeavePeriod = sbgCanTargetToHostFloat( *(uint32*)(receivedData + sizeof(uint8)) );
				}
			}
			else
			{
				//
				// Answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Send the magnetic declination used by the Kalman filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	heading							The desired heading in degrees used by the device as input.
 *	\param[in]	accuracy						The heading accuracy in degrees for the previous datum.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterHeading(SbgCanDeviceHandle deviceHandle, float heading, float accuracy)
{
	uint8 dataBuffer[sizeof(int32)+sizeof(uint32)];
	SbgErrorCode errorCode;
	
	//
	// Fill the CAN message data
	//
	*(int32*)(dataBuffer) = sbgCanHostToTarget32( (int32)(heading*1e5f) );
	*(uint32*)(dataBuffer + sizeof(int32)) = sbgCanHostToTarget32( (uint32)(accuracy*1e5f) );
		
	//
	// Create and send the CAN command, the device shouldn't return an answer
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_FILTER_HEADING, sizeof(dataBuffer), dataBuffer);

	return errorCode;
}
