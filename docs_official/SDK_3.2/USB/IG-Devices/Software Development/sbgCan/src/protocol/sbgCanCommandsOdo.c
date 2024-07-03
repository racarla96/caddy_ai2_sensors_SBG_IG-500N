#include "sbgCanCommandsOdo.h"
#include "sbgCanProtocolOutputMode.h"
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
//- Odometer commands                                                  -//
//----------------------------------------------------------------------//

/*!
 *	Set the main configuration of the external odometer channels
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	axis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_CAN_ODO_X
 *										- SBG_CAN_ODO_Y
 *										- SBG_CAN_ODO_Z
 *  \param[in]	pulsePerMeter		decimal number of pulses per meter
 *  \param[in]	gainError			Error in percent on the previous gain value
 *  \param[in]	gpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain 
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoConfig(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoAxis axis, float pulsesPerMeter, uint8 gainError, bool gpsGainCorrection)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[3*sizeof(uint8)+sizeof(uint32)];
	uint8 gpsGainOpts;
	SbgErrorCode errorCode;

	//
	// Check that we have a valid sbgCan handle
	//
	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Set last field depending on GPS automatic correction
		//
		if (gpsGainCorrection)
		{
			gpsGainOpts = SBG_CAN_ODO_AUTO_GPS_GAIN | (gainError & (~SBG_CAN_ODO_AUTO_GPS_GAIN));
		}
		else
		{
			gpsGainOpts =  (gainError & (~SBG_CAN_ODO_AUTO_GPS_GAIN));
		}

		//
		// Fill the CAN message data buffer
		//
		dataBuffer[0] = channel;
		dataBuffer[1] = (uint8)axis;
		*(uint32*)(dataBuffer+ 2*sizeof(uint8)) = sbgCanHostToTargetFixed32(pulsesPerMeter);
		*(dataBuffer+ 2*sizeof(uint8)+sizeof(uint32)) = gpsGainOpts;

		//
		// Create the CAN message and send it
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_CONFIG, sizeof(dataBuffer), dataBuffer);

		//
		// Check that the CAN message has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_CONFIG, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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


/*!
 *	Get the main configuration of the external odometer channels
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pAxis				Odometer measurement axis (in sensor coordinate frame) May be:
 *										- SBG_CAN_ODO_X
 *										- SBG_CAN_ODO_Y
 *										- SBG_CAN_ODO_Z
 *  \param[out]	pPulsePerMeter		decimal number of pulses per meter
 *  \param[out]	pGainError			Error in percent on the previous gain value
 *	\param[out]	pGpsGainCorrection	If set to TRUE, the GPS will automatically enhance the odometer gain
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoConfig(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoAxis *pAxis, float *pPulsesPerMeter, uint8 *pGainError, bool *pGpsGainCorrection)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[1];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_CONFIG, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_CONFIG, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( (receivedLength == 3*sizeof(uint8)+sizeof(uint32)) && (receivedData[0] == channel) )
			{
				//
				// Fill the odometer axis if needed
				//
				if (pAxis)
				{
					*pAxis = (SbgCanOdoAxis)receivedData[1];
				}

				//
				// Fill the odometer pulses per meter if needed
				//
				if (pPulsesPerMeter)
				{
					*pPulsesPerMeter = sbgCanTargetToHostFloat(*(uint32*)(receivedData+2*sizeof(uint8)));
				}

				//
				// Fill the odometer gain errorCode if needed
				//
				if (pGainError)
				{
					*pGainError = *(receivedData+2*sizeof(uint8)+sizeof(uint32))  & (~SBG_CAN_ODO_AUTO_GPS_GAIN);
				}

				if (pGpsGainCorrection)
				{
					if (*(receivedData+2*sizeof(uint8)+sizeof(uint32)) & SBG_CAN_ODO_AUTO_GPS_GAIN)
					{
						*pGpsGainCorrection = TRUE;
					}
					else
					{
						*pGpsGainCorrection = FALSE;
					}
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
 *  Configures the odometer direction for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoDirection		Direction of the odometer. May be:
 *									- SBG_CAN_ODO_DIR_POSITIVE
 *									- SBG_CAN_ODO_DIR_NEGATIVE
 *									- SBG_CAN_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoDirection(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoDirection odoDirection)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[2];
	SbgErrorCode errorCode;


	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = (uint8)channel;
	dataBuffer[1] = (uint8)odoDirection;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_DIRECTION, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_DIRECTION, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
 *  Get the odometer direction for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	pOdoDirection		Direction of the odometer. May be:
 *									- SBG_CAN_ODO_DIR_POSITIVE
 *									- SBG_CAN_ODO_DIR_NEGATIVE
 *									- SBG_CAN_ODO_DIR_AUTO
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoDirection(SbgCanDeviceHandle deviceHandle, uint8 channel, SbgCanOdoDirection *pOdoDirection)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[1];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = channel;

	//
	// Create the CAN message and send it
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_DIRECTION, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_DIRECTION, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( (receivedLength == 2*sizeof(uint8)) && (receivedData[0] == channel) )
			{
				//
				// Fill the odometer direction if needed
				//
				if (pOdoDirection)
				{
					*pOdoDirection = (SbgCanOdoDirection)*(uint8*)(receivedData+sizeof(uint8));
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
 *  Configures the odometer lever arm for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[in]	odoLeverArm			X,Y,Z vector representing the distance between the device and the odometer.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetOdoLeverArm(SbgCanDeviceHandle deviceHandle, uint8 channel, const float odoLeverArm[3])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)+sizeof(int16)*3];
	SbgErrorCode errorCode;

	//
	// Check input parameters
	//
	if (odoLeverArm)
	{
		//
		// Fill the CAN message data buffer
		//
		dataBuffer[0] = channel;
		*(int16*)(dataBuffer+sizeof(uint8)+sizeof(int16)*0) = sbgCanHostToTarget16((int16)(odoLeverArm[0]*1000));
		*(int16*)(dataBuffer+sizeof(uint8)+sizeof(int16)*1) = sbgCanHostToTarget16((int16)(odoLeverArm[1]*1000));
		*(int16*)(dataBuffer+sizeof(uint8)+sizeof(int16)*2) = sbgCanHostToTarget16((int16)(odoLeverArm[2]*1000));

		//
		// Create the CAN message and send it
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_LEVER_ARM, sizeof(dataBuffer), dataBuffer);

		//
		// Check that the CAN message has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_LEVER_ARM, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
	}
	else
	{
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *  Get the odometer lever arm for the corresponding channel
 *	\param[in]	deviceHandle		A valid ig can device handle.
 *  \param[in]	channel				Odometer Channel to use 0 or 1
 *  \param[out]	odoLeverArm			X,Y,Z vector representing the distance between the device and the odometer.<br>
 *									The distance is exressed in meters in the device coordinate system.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetOdoLeverArm(SbgCanDeviceHandle deviceHandle, uint8 channel, float odoLeverArm[3])
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
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_LEVER_ARM, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ODO_LEVER_ARM, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if ( (receivedLength == sizeof(uint8)+sizeof(int16)*3) && (receivedData[0] == channel) )
			{
				//
				// Fill the odometer lever arm if needed
				//
				if (odoLeverArm)
				{
					odoLeverArm[0] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData+sizeof(uint8)+sizeof(int16)*0) ))/1000.0f;
					odoLeverArm[1] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData+sizeof(uint8)+sizeof(int16)*1) ))/1000.0f;
					odoLeverArm[2] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData+sizeof(uint8)+sizeof(int16)*2) ))/1000.0f;
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
