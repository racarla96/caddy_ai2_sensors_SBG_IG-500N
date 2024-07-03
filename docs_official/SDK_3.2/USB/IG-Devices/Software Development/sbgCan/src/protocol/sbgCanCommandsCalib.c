#include "sbgCanCommandsCalib.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Calibration commands operations                                    -//
//----------------------------------------------------------------------//

/*!
 * Initiate or terminate the magnetometers calibration procedure.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	argument						Pointer to an int8 used to hold the calibration argument.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanCalibMagProcedure(SbgCanDeviceHandle deviceHandle, SbgCanCalibMagsAction argument)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[1];
	SbgErrorCode errorCode;
	
	//
	// Fill the CAN message data
	//
	dataBuffer[0] = (uint8)argument;

	//
	// Build and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive an answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint8))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (receivedData[0] != dataBuffer[0])
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}



	return errorCode;
}

/*!
 * Set a manual calibration data for magnetometers.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	bias							The magnetometers offset vector.
 * \param[in]	agm								The magnetometers Alignment and Gain Matrix.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetCalibMagManual(SbgCanDeviceHandle deviceHandle, const float bias[3], const float agm[9])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)+sizeof(uint32)];
	uint8 index;
	SbgErrorCode errorCode = SBG_NO_ERROR;

	//
	// Check input pointers
	//
	if ((bias) && (agm))
	{
		//
		// We should send 12 frames 3 for the offset and 9 for the alignment matrix
		//
		for (index=0; index<12; index++)
		{
			//
			// The first uint8 is the frame index
			//
			*dataBuffer = index;

			//
			// Check if we are sending the bias or the gain matrix
			//
			if (index<3)
			{
				//
				// We are sending the bias
				//
				*(uint32*)(dataBuffer+sizeof(uint8)) = sbgCanHostToTargetFixed32(bias[index]);
			}
			else
			{
				//
				// We are sending the gain matrix
				//
				*(uint32*)(dataBuffer+sizeof(uint8)) = sbgCanHostToTargetFixed32(agm[index-3]);
			}

			//
			// Build the CAN message and send it
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG_MANUAL, sizeof(dataBuffer), dataBuffer);

			//
			// Check if we were able to send the message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Wait for an answer
				//
				errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG_MANUAL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

				//
				// Check if we were able to receive an answer
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Check if the response is valid
					//
					if (receivedLength != sizeof(uint8)+sizeof(uint32))
					{
						//
						// Invalid received frame size
						//
						errorCode = SBG_INVALID_FRAME;
						break;
					}
					else if (memcmp(dataBuffer, receivedData, sizeof(uint8)+sizeof(uint32)) != 0)
					{
						//
						// The received frame doesn't match with the sent frame
						//
						errorCode = SBG_INVALID_PARAMETER;
						break;
					}
				}
				else
				{
					//
					// Unable to receive the frame so exit
					//
					break;
				}
			}
			else
			{
				//
				// Unable to send the frame so exit
				//
				break;
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
 * Retrieve a manual calibration data for magnetometers.
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[out]	bias							Float array used to hold the offset vector.
 * \param[out]	agm								Float array used to hold the Alignment and Gain Matrix.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetCalibMagManual(SbgCanDeviceHandle deviceHandle, float bias[3], float agm[9])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint16 index;
	uint16 receivedFrameIndex;
	SbgErrorCode errorCode;

	//
	// Check input pointers
	//
	if ((bias) && (agm))
	{
		//
		// Build and send the CAN message
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG_MANUAL, 0, NULL);

		//
		// Check if we were able to send the message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// We should receive 12 frames, 3 for the offset and 9 for the gain matrix
			//
			for (index=0; index<12; index++)
			{
				//
				// Try to receive a frame
				//
				errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_MAG_MANUAL, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

				//
				// Check if we were able to receive the frame
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Check if the answer is valid
					//
					if (receivedLength == sizeof(uint8)+sizeof(uint32))
					{
						//
						// Extract the frame index
						//
						receivedFrameIndex = *receivedData;

						//
						// Check if the received frame index is consistent
						//
						if (receivedFrameIndex == index)
						{
							//
							// Check if we are getting the vector or the gain and alignement matrix
							//
							if (receivedFrameIndex<3)
							{
								//
								// We have received the offset vector
								//
								if (bias)
								{
									bias[receivedFrameIndex] = sbgCanTargetToHostFloat(*(uint32*)(receivedData+sizeof(uint8)));
								}
							}
							else
							{
								//
								// We have received the gain and alingement matrix
								//
								if (agm)
								{
									agm[receivedFrameIndex-3] = sbgCanTargetToHostFloat(*(uint32*)(receivedData+sizeof(uint8)));
								}
							}
						}
						else
						{
							//
							// We have maybe missed a frame
							//
							errorCode = SBG_INVALID_FRAME;
							break;
						}
					}
					else
					{
						//
						// The received frame has an invalid length
						//
						errorCode = SBG_INVALID_FRAME;
						break;
					}
				}
				else
				{
					//
					// Unable to receive the frame so exit
					//
					break;
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
 * Read the current gyro value to evaluate the gyro bias
 * \param[in]	deviceHandle					A valid ig can device handle.
 * \param[in]	argument						The argument for gyro calibration.
 * \return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanCalibGyroBias(SbgCanDeviceHandle deviceHandle, SbgCanCalibGyrosAction argument)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)];
	SbgErrorCode errorCode;
	
	//
	// Fill the CAN message data
	//
	dataBuffer[0] = (uint8)argument;

	//
	// Build and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_GYRO_BIAS, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_CALIB_GYRO_BIAS, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive an answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint8))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (receivedData[0] != dataBuffer[0])
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}
