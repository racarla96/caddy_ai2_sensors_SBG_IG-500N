#include "sbgCanCommandsNav.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Navigation configuration commands								   -//
//----------------------------------------------------------------------//

/*!
 *	Set the source for the velocity and position used by the navigation filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	positionSource					The postion source.
 *	\param[in]	velocitySource					The Velocity source.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetNavSources(SbgCanDeviceHandle deviceHandle, SbgCanAidingPosSrc positionSource, SbgCanAidingVelSrc velocitySource)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[2*sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data
	//
	dataBuffer[0] = (uint8)positionSource;
	dataBuffer[1] = (uint8)velocitySource;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_NAV_SOURCES, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_NAV_SOURCES, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != 2*sizeof(uint8)) 
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, 2*sizeof(uint8)) != 0)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Get the source for the velocity and position used by the navigation filter.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pPositionSource					Pointer to an uint8 used to hold the postion source.
 *	\param[out]	pVelocitySource					Pointer to an uint8 used to hold the velocity source.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetNavSources(SbgCanDeviceHandle deviceHandle, SbgCanAidingPosSrc *pPositionSource, SbgCanAidingVelSrc *pVelocitySource)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_NAV_SOURCES, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_NAV_SOURCES, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength == 2*sizeof(uint8))
			{
				//
				// Return the configuration is possible
				//
				if (pPositionSource)
				{
					*pPositionSource = (SbgCanAidingPosSrc)receivedData[0];
				}
				if (pVelocitySource)
				{
					*pVelocitySource = (SbgCanAidingVelSrc)receivedData[1];
				}
			}
			else
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Set the GPS lever arm vector from the device to the antenna. (IG-500N only)
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	gpsLeverArm						X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *												The distance is exressed in meters in the device coordinate system.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetGpsLeverArm(SbgCanDeviceHandle deviceHandle, const float gpsLeverArm[3])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[3*sizeof(int16)];
	SbgErrorCode errorCode;

	//
	// Check if we have a valid parameter
	//
	if (gpsLeverArm)
	{
		//
		// Fill the CAN message data, the lever arm should be sent using millimeters
		//
		*(int16*)(dataBuffer) = sbgCanHostToTarget16((int16) (gpsLeverArm[0]*1000));
		*(int16*)(dataBuffer + sizeof(int16)) = sbgCanHostToTarget16((int16) (gpsLeverArm[1]*1000));
		*(int16*)(dataBuffer + 2*sizeof(int16)) = sbgCanHostToTarget16((int16) (gpsLeverArm[2]*1000));

		//
		// Create and send the CAN command
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_GPS_LEVER_ARM, sizeof(dataBuffer), dataBuffer);

		//
		// Check if we were able to send the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_GPS_LEVER_ARM, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

			//
			// Check if we were able to receive the CAN message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the answer is valid
				//
				if (receivedLength != 3*sizeof(int16)) 
				{
					errorCode = SBG_INVALID_FRAME;
				}
				else if (memcmp(receivedData, dataBuffer, 3*sizeof(int16)) != 0)
				{
					errorCode = SBG_INVALID_PARAMETER;
				}
			}
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
 *	Get the GPS lever arm vector from the device to the antenna. (IG-500N- only)
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[out]	gpsLeverArm						X,Y,Z vector representing the distance between the device and the GPS antenna.<br>
 *												The distance is exressed in meters in the device coordinate system.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetGpsLeverArm(SbgCanDeviceHandle deviceHandle, float gpsLeverArm[3])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_GPS_LEVER_ARM, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_GPS_LEVER_ARM, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength == 3*sizeof(int16))
			{
				//
				// If possible, get the GPS lever arm and convert it from miilimeters to meters
				//
				if (gpsLeverArm)
				{
					gpsLeverArm[0] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData) ))/1000.0f;
					gpsLeverArm[1] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData + sizeof(int16))))/1000.0f;
					gpsLeverArm[2] = ((float)sbgCanTargetToHost16( *(int16*)(receivedData + 2*sizeof(int16))))/1000.0f;
				}
			}
			else
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Set the local gravity magnitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	magnitude						The device local magnitude (in m.s-²).
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetGravityMagnitude(SbgCanDeviceHandle deviceHandle, float magnitude)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data
	//
	*(uint32*)(dataBuffer) = sbgCanHostToTargetFixed32(magnitude);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_GRAVITY_MAGNITUDE, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_GRAVITY_MAGNITUDE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint32))
			{
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, sizeof(uint32)) != 0)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Get the local gravity magnitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pMagnitude						Pointer to a float used to hold the device local magnitude.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetGravityMagnitude(SbgCanDeviceHandle deviceHandle, float *pMagnitude)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_GRAVITY_MAGNITUDE, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_GRAVITY_MAGNITUDE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength == sizeof(uint32))
			{
				//
				// If possible, return the setting
				//
				if (pMagnitude)
				{
					*pMagnitude = sbgCanTargetToHostFloat( *(uint32*)(receivedData) );
				}
			}
			else
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Send the velocity aiding information used by the Navigation filter.<br>
 *	The velocity is expressed in meters per second in the device local coordinate system.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	velocity						The aiding velocity respectively along x, y and z axis in m/s in the device coordinate system.
 *	\param[in]	accuracy						The accuracy of the provided velocity in m/s.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterVelocityLocal(SbgCanDeviceHandle deviceHandle, const float velocity[3], float accuracy)
{
	uint8 dataBuffer0[sizeof(int32)+sizeof(int32)];
	uint8 dataBuffer1[sizeof(int32)+sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Check parameters
	//
	if ((velocity) && (accuracy > 0.0f))
	{
		//
		// Fill the first CAN message, the velocity should be sent in cm/s
		//
		*(int32*)(dataBuffer0) = sbgCanHostToTarget32( (int32)(velocity[0]*100.0f) );
		*(int32*)(dataBuffer0 + sizeof(int32)) = sbgCanHostToTarget32( (int32)(velocity[1]*100.0f) );

		//
		// Fill the second CAN message, the velocity and accuracy should be sent in cm/s
		//
		*(int32*)(dataBuffer1) = sbgCanHostToTarget32( (int32)(velocity[2]*100.0f) );
		*(uint16*)(dataBuffer1 + sizeof(int32)) = sbgCanHostToTarget16( (uint16)(accuracy*100.0f) );
			
		//
		// Send the first CAN message, the device shouldn't return an answer
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_VELOCITY_LOCAL_1, sizeof(dataBuffer0), dataBuffer0);		

		//
		// Check if we were able to send the first CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the second CAN message, the device shoudln't return an answer
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_VELOCITY_LOCAL_2, sizeof(dataBuffer1), dataBuffer1);
		}
	}
	else
	{
		//
		// The input velocity is invalid
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 *	Send the velocity aiding information used by the Navigation filter.<br>
 *	The velocity is expressed in meters per second in the North East Down (NED) coordinate system.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	velocity						The aiding velocity respectively along North, East and Down axis in m/s in NED coordinate system.
 *	\param[in]	accuracy						The accuracy of the provided velocity in m/s.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterVelocityNED(SbgCanDeviceHandle deviceHandle, const float velocity[3], float accuracy)
{
	uint8 dataBuffer0[sizeof(int32)+sizeof(int32)];
	uint8 dataBuffer1[sizeof(int32)+sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Check parameters
	//
	if ((velocity) && (accuracy > 0.0f))
	{
		//
		// Fill the first CAN message, the velocity should be sent in cm/s
		//
		*(int32*)(dataBuffer0) = sbgCanHostToTarget32( (int32)(velocity[0]*100.0f) );
		*(int32*)(dataBuffer0 + sizeof(int32)) = sbgCanHostToTarget32( (int32)(velocity[1]*100.0f) );

		//
		// Fill the second CAN message, the velocity and accuracy should be sent in cm/s
		//
		*(int32*)(dataBuffer1) = sbgCanHostToTarget32( (int32)(velocity[2]*100.0f) );
		*(uint16*)(dataBuffer1 + sizeof(int32)) = sbgCanHostToTarget16( (uint16)(accuracy*100.0f) );

		//
		// Send the first CAN message, the device shouldn't return an answer
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_VELOCITY_NED_1, sizeof(dataBuffer0), dataBuffer0);		

		//
		// Check if we were able to send the first CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the second CAN message, the device shoudln't return an answer
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_VELOCITY_NED_2, sizeof(dataBuffer1), dataBuffer1);
		}
	}
	else
	{
		//
		// The input velocity is invalid
		//
		errorCode = SBG_INVALID_PARAMETER;
	}

	return errorCode;
}

/*!
 *	Send the position aiding information used by the Navigation filter.<br>
 *	The position is expressed in WGS84 format in degrees for latitude/longitude and in meters for altitude.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	position						The desired positions respectively latitude, longitude and height in degrees, degrees and meters.
 *	\param[in]	accuracies						The accuracy of the provided position respectively horizontal and vertical in meters.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSendFilterPosition(SbgCanDeviceHandle deviceHandle, const double position[3], const float accuracies[2])
{
	uint8 dataBuffer0[sizeof(int32)+sizeof(int32)];
	uint8 dataBuffer1[sizeof(int32)+2*sizeof(uint16)];
	SbgErrorCode errorCode;
	
	//
	// Check if we have valid parameters
	//
	if ( (position) && (accuracies) && (accuracies[0] > 0.0f) && (accuracies[1] > 0.0f) )
	{
		//
		// Fill the first CAN message, the position should be sent in 10^7 degrees
		//
		*(int32*)(dataBuffer0) = sbgCanHostToTarget32( (int32)(position[0]*1e7) );
		*(int32*)(dataBuffer0 + sizeof(int32)) = sbgCanHostToTarget32( (int32)(position[1]*1e7) );

		//
		// Fill the second CAN message, the altitude should be sent in mm and accuracies in cm
		//
		*(int32*)(dataBuffer1) = sbgCanHostToTarget32( (int32)(position[2]*1000.0f) );
		*(uint16*)(dataBuffer1 + sizeof(int32)) = sbgCanHostToTarget16( (uint16)(accuracies[0]*100.0f) );
		*(uint16*)(dataBuffer1 + sizeof(int32) + sizeof(uint16)) = sbgCanHostToTarget16( (uint16)(accuracies[1]*100.0f) );

		//
		// Send the first CAN message, the device shouldn't return an answer
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_POSITION_1, sizeof(dataBuffer0), dataBuffer0);		

		//
		// Check if we were able to send the first CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Send the second CAN message, the device shoudln't return an answer
			//
			errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SEND_NAV_POSITION_2, sizeof(dataBuffer1), dataBuffer1);
		}
	}
	else
	{
		//
		// The input velocity is invalid
		//
		errorCode = SBG_INVALID_PARAMETER;
	}
	
	return errorCode;
}
