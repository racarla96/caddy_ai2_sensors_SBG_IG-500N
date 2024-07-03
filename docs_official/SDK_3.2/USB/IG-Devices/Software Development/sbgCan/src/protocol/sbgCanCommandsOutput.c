#include "sbgCanCommandsOutput.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Outputs configuration commands                                     -//
//----------------------------------------------------------------------//

/*!
 *	Configures for a specific device output its triggers.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultId						The requested device output default id.
 *	\param[in]	trigger							The configuration of the output trigger.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetOutputConf(SbgCanDeviceHandle deviceHandle, uint16 defaultId, uint16 trigger)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[2*sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Fill the message buffer and convert to device units
	//
	*(uint16*)(dataBuffer) = sbgCanHostToTarget16(defaultId);
	*(uint16*)(dataBuffer + sizeof(uint16)) = sbgCanHostToTarget16(trigger);

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_OUTPUT_TRIGGERS_CONF, sizeof(dataBuffer), dataBuffer);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_OUTPUT_TRIGGERS_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength != 2*sizeof(uint16))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, 2*sizeof(uint16)) != 0)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Returns for a specific device output its triggers.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultId						The requested device output default id.
 *	\param[out] pTrigger						Pointer to the configuration of the output trigger.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOutputConf(SbgCanDeviceHandle deviceHandle, uint16 defaultId, uint16 *pTrigger)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Fill the message buffer and convert to device units
	//
	*((uint16*)dataBuffer) = sbgCanHostToTarget16(defaultId);

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_OUTPUT_TRIGGERS_CONF, sizeof(dataBuffer), dataBuffer);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_OUTPUT_TRIGGERS_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength != sizeof(uint16)+sizeof(uint16))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, sizeof(uint16)) != 0)
			{
				//
				// The returned defaultId is invalid
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
			else
			{
				//
				// Return the message data
				//
				if (pTrigger)
				{
					*pTrigger = sbgCanTargetToHost16( *( (uint16*)(receivedData + sizeof(uint16)) ) );
				}
			}
		}
	}

	return errorCode;
}

/*!
 *	Configures the device main loop frequency divider.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	divider							The new frequency divider.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetFrequencyDivider(SbgCanDeviceHandle deviceHandle, uint8 divider)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Fill the message buffer and convert to device units
	//
	dataBuffer[0] = divider;

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_MAIN_LOOP_DIVIDER, sizeof(dataBuffer), dataBuffer);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_MAIN_LOOP_DIVIDER, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR) 
		{
			//
			// Check if the response is valid
			//
			if (receivedLength != sizeof(uint8))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (receivedData[0] != divider)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Retrieves the device main loop frequency divider.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDivider						Pointer to the frequency divider.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetFrequencyDivider(SbgCanDeviceHandle deviceHandle, uint8 *pDivider)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_MAIN_LOOP_DIVIDER, 0, NULL);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_MAIN_LOOP_DIVIDER, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(uint8))
			{
				//
				// Return the setting if needed
				//
				if (pDivider)
				{
					*pDivider = receivedData[0];
				}
			}
			else
			{
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}


	return errorCode;
}

/*!
 *	Retrieves a specific output for the device and return the corresponding CAN message.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	outputId						The requested device output default id as listed in the SbgCanId enum.
 *	\param[out] pOutput							Pointer to a structure that hold the output data and default id.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanGetSpecificOutput(SbgCanDeviceHandle deviceHandle, SbgCanId outputId, SbgCanOutputDataStr *pOutput)
{
	SbgCanMessageRaw receivedMsg;
	SbgErrorCode errorCode;
	
	//
	// Check that the requested default CAN message id is an output frame
	//
	if (outputId <= SBG_CAN_OUTPUT_ID_LAST)
	{
		//
		// Send an empty command to get the output
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, outputId, 0, NULL);

		//
		// Check if the message has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanBusReceiveSpecificMessage(deviceHandle->canBusHandle, &receivedMsg, deviceHandle->framesIdList[outputId].messageFormat, deviceHandle->framesIdList[outputId].messageId, SBG_CAN_FRAME_RECEPTION_TIME_OUT);

			//
			// Check if we were able to get the answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// We were able to receive the answer so fill the output data if needed
				//
				if (pOutput)
				{
					//
					// Try to fill the output data with the received CAN message
					//
					errorCode = sbgCanDeviceHandleOutputs(deviceHandle, &receivedMsg, pOutput);
				}
			}
		}
	}
	else
	{
		//
		// The requested output id isn't a valid output frame
		//
		errorCode = SBG_INVALID_PARAMETER;
	}


	return errorCode;
}
