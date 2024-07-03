#include "sbgCanCommandsSync.h"
#include "sbgCanProtocolOutputMode.h"
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
// Logic inputs operations                                              //
//----------------------------------------------------------------------//

/*!
 *	Set a logic input channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[in]	inputType			Type of the logic input, may be <br>
 * 										- SBG_CAN_IN_DISABLED
 *										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *										- SBG_CAN_IN_TIME_PULSE
 * 										- SBG_CAN_IN_BARO
 * 										- SBG_CAN_IN_ODOMETER
 * \param[in]	sensitivity			Sensitivity of the trigger. It may be:<br>
 * 										- SBG_CAN_IN_FALLING_EDGE
 * 										- SBG_CAN_IN_RISING_EDGE
 * 										- SBG_CAN_IN_LEVEL_CHANGE
 * \param[in]	location			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
										- SBG_CAN_IN_STD_LOCATION (default value, leave to this value if not used)
										- SBG_CAN_IN_EXT_LOCATION
 * \param[in]	nsDelay				Delay to be added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are allowed:<br>
 *									This delay is only used when the input is set to:
 * 										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanSetLogicInChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicInType inputType, SbgCanLogicInSensitivity sensitivity,SbgCanLogicInLocation location, uint32 nsDelay)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[3*sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;
	dataBuffer[1] = (uint8)inputType;
	dataBuffer[2] = (uint8)sensitivity + (uint8)location ;		
	*(uint32*)(dataBuffer+3*sizeof(uint8)) = sbgCanHostToTarget32(nsDelay);

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_IN_CHANNEL, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_IN_CHANNEL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
 *	Get a logic input channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. May Be 0 or 1
 *	\param[out]	pInputType			Type of the logic input, may be <br>
 * 										- SBG_CAN_IN_DISABLED
 *										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *										- SBG_CAN_IN_TIME_PULSE
 * 										- SBG_CAN_IN_BARO
 * 										- SBG_CAN_IN_ODOMETER	
 * \param[out]	pSensitivity		Sensitivity of the trigger. It may be:<br>
 * 										- SBG_CAN_IN_FALLING_EDGE
 * 										- SBG_CAN_IN_RISING_EDGE
 * 										- SBG_CAN_IN_LEVEL_CHANGE
 * \param[out]	pLocation			Physical location of the input pin for the syncIN channel 0 on IG-500E. <br>
										- SBG_CAN_IN_STD_LOCATION
										- SBG_CAN_IN_EXT_LOCATION
 * \param[out]	pNsDelay			Delay added before the actual trigger is taken into account (in nano seconds) delays up to 2seconds are possible:<:<br>
 *									This delay is only valid when the input is set to:
 * 										- SBG_CAN_IN_EVENT
 * 										- SBG_CAN_IN_MAIN_LOOP_START
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanGetLogicInChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicInType *pInputType, SbgCanLogicInSensitivity *pSensitivity, SbgCanLogicInLocation *pLocation, uint32 *pNsDelay)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_IN_CHANNEL, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_IN_CHANNEL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( (receivedLength == 3*sizeof(uint8) + sizeof(uint32)) && (receivedData[0] == channel) )
			{
				//
				// Fill the syncIn input type if needed
				//
				if (pInputType)
				{
					*pInputType = (SbgCanLogicInType)receivedData[1];
				}

				//
				// Fill the syncIn config if needed
				//
				if (pSensitivity)
				{
					*pSensitivity = (SbgCanLogicInSensitivity)(receivedData[2] & SBG_CAN_IN_SENSE_MASK);
				}

				if (pLocation)
				{
					*pLocation = (SbgCanLogicInLocation)(receivedData[2] & SBG_CAN_IN_EXT_LOCATION);
				}
				if (pNsDelay)
				{
					*pNsDelay = sbgCanTargetToHost32(*(uint32*)(receivedData + 3*sizeof(uint8)));
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
 *	Set a logic output channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. Set always 0
 *	\param[in]	outputType			Type of the logic output, may be <br>
 * 										- SBG_CAN_OUT_DISABLED
 *										- SBG_CAN_OUT_MAIN_LOOP_START
 *										- SBG_CAN_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_CAN_OUT_TIME_PULSE_COPY
 *										- SBG_CAN_OUT_EVENT_COPY
 * \param[in]	polarity			Polarity of the out pulse. It may be:
 * 										- SBG_CAN_OUT_FALLING_EDGE
 * 										- SBG_CAN_OUT_RISING_EDGE
 * 										- SBG_CAN_OUT_TOGGLE
 *  \param[in]	duration			When the polarity is set to SBG_CAN_OUT_FALLING_EDGE or SBG_CAN_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *									Leave to 0 if not used.
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanSetLogicOutChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicOutType outputType, SbgCanLogicOutPolarity polarity, uint8 duration)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[4*sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;
	dataBuffer[1] = (uint8)outputType;
	dataBuffer[2] = (uint8)polarity;		
	dataBuffer[3] = duration;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_OUT_CHANNEL, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_OUT_CHANNEL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
 *	Get a logic output channel setting
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	channel				Channel to configure. Leave always to 0
 *	\param[out]	pOutputType			Type of the logic output, may be <br>
 * 										- SBG_CAN_OUT_DISABLED
 *										- SBG_CAN_OUT_MAIN_LOOP_START
 *										- SBG_CAN_OUT_MAIN_LOOP_DIVIDER
 *										- SBG_CAN_OUT_TIME_PULSE_COPY
 *										- SBG_CAN_OUT_EVENT_COPY
 * \param[out]	pPolarity			Polarity of the out pulse. It may be:
 * 										- SBG_CAN_OUT_FALLING_EDGE
 * 										- SBG_CAN_OUT_RISING_EDGE
 * 										- SBG_CAN_OUT_TOGGLE
 *  \param[out]	pDuration			When the polarity is set to SBG_CAN_OUT_FALLING_EDGE or SBG_CAN_OUT_RISING_EDGE, this is the pulse duration in ms. <br>
 *	\return							SBG_NO_ERROR if no error
 */
SbgErrorCode sbgCanGetLogicOutChannel(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanLogicOutType *pOutputType, SbgCanLogicOutPolarity *pPolarity, uint8 *pDuration)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_OUT_CHANNEL, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOGIC_OUT_CHANNEL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( (receivedLength == 4*sizeof(uint8)) && (receivedData[0] == channel) )
			{
				//
				// Fill the syncOut output type if needed
				//
				if (pOutputType)
				{
					*pOutputType = (SbgCanLogicOutType)receivedData[1];
				}

				//
				// Fill the syncOut polarity if needed
				//
				if (pPolarity)
				{
					*pPolarity = (SbgCanLogicOutPolarity)receivedData[2];
				}

				//
				// Fill the syncOut duration if needed
				//
				if (pDuration)
				{
					*pDuration = receivedData[3];
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
 *	Defines the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[in]	distance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetVirtualOdoConf(SbgCanDeviceHandle deviceHandle, float distance)
{
	uint8 receivedLength;
	uint32 receivedData[2];
	uint32 dataBuffer[2];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = sbgCanHostToTargetFixed32(distance);
	dataBuffer[1] = 0;
	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_VIRTUAL_ODO_CONF, sizeof(dataBuffer), (const uint8*)dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_VIRTUAL_ODO_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, (uint8*)receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( receivedLength != sizeof(receivedData) || (memcmp(receivedData, dataBuffer, sizeof(dataBuffer) != 0)) )
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
 *	Retrieves the distance between two pulses when sync out is configured as a virtual odometer
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *	\param[out]	pDistance				Distance in meters
 *	\return								SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetVirtualOdoConf(SbgCanDeviceHandle deviceHandle, float *pDistance)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create the CAN message question and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_VIRTUAL_ODO_CONF, 0, NULL);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_VIRTUAL_ODO_CONF, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( receivedLength == sizeof(receivedData) )
			{
				//
				// Fill the distance
				//
				if (pDistance)
				{
					*pDistance = sbgCanTargetToHostFloat(*(uint32*)(receivedData));
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