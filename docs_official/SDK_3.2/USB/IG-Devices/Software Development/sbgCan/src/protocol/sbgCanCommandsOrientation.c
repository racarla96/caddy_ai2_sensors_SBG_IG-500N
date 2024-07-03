#include "sbgCanCommandsOrientation.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Orientation commands											   -//
//----------------------------------------------------------------------//

/*!
 *	Configures the device to align to the coordinate frame's axis.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	offset							The orientation offset type.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSetAutoOrientationOffset(SbgCanDeviceHandle deviceHandle, SbgCanOffsetType offset)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[1];
	SbgErrorCode errorCode;

	//
	// Fill the message buffer and convert to device units
	//
	dataBuffer[0] = (uint8)offset;
		
	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_AUTO_ORIENTATION_OFFSET, sizeof(dataBuffer), dataBuffer);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer (add 200 ms time out requiered by the command)
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_AUTO_ORIENTATION_OFFSET, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
			else if ((SbgCanOffsetType)(receivedData[0]) != offset)
			{
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Set the manual rotation to be applied on sensors input in pre-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetManualPreOrientationOffset(SbgCanDeviceHandle deviceHandle, const float quaternion[4])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[4*sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Check if we have a valid input
	//
	if (quaternion)
	{
		//
		// Fill the message buffer and convert to device units
		//
		*(uint16*)(dataBuffer + 0*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[0]);
		*(uint16*)(dataBuffer + 1*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[1]);
		*(uint16*)(dataBuffer + 2*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[2]);
		*(uint16*)(dataBuffer + 3*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[3]);

		//
		// Build and send the command message
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_PRE_ORIENTATION_OFFSET, sizeof(dataBuffer), dataBuffer);

		//
		// Check if the message has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_PRE_ORIENTATION_OFFSET, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

			//
			// Check if we were able to receive the frame
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the response is valid
				//
				if (receivedLength != sizeof(uint16)*4)
				{
					errorCode = SBG_INVALID_FRAME;
				}
				else if (memcmp(receivedData, dataBuffer, sizeof(uint16)*4) != 0)
				{
					errorCode = SBG_INVALID_PARAMETER;
				}
			}
		}
	}
	else
	{
		//
		// NULL handle pass as argument
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Get the manual rotation to be applied on sensors input in pre-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetManualPreOrientationOffset(SbgCanDeviceHandle deviceHandle, float quaternion[4])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_PRE_ORIENTATION_OFFSET, 0, NULL);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_PRE_ORIENTATION_OFFSET, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(uint16)*4)
			{
				//
				// Return the converted data
				//
				if (quaternion)
				{
					quaternion[0] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 0*sizeof(uint16)) );
					quaternion[1] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 1*sizeof(uint16)) );
					quaternion[2] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 2*sizeof(uint16)) );
					quaternion[3] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 3*sizeof(uint16)) );
				}
			}
			else
			{
				//
				// Answer size is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}

/*!
 *	Set the manual rotation to be applied on sensors input in post-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetManualPostOrientationOffset(SbgCanDeviceHandle deviceHandle, const float quaternion[4])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[4*sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Check if we have a valid input
	//
	if (quaternion)
	{
		//
		// Fill the message buffer and convert to device units
		//
		*(uint16*)(dataBuffer + 0*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[0]);
		*(uint16*)(dataBuffer + 1*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[1]);
		*(uint16*)(dataBuffer + 2*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[2]);
		*(uint16*)(dataBuffer + 3*sizeof(uint16)) = sbgCanHostToTargetFrac16(quaternion[3]);

		//
		// Build and send the command message
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_POST_ORIENTATION_OFFSET, sizeof(dataBuffer), dataBuffer);

		//
		// Check if the message has been sent
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_POST_ORIENTATION_OFFSET, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

			//
			// Check if we have received an answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the response is valid
				//
				if (receivedLength != sizeof(uint16)*4) 
				{
					errorCode = SBG_INVALID_FRAME;
				}
				else if (memcmp(receivedData, dataBuffer, sizeof(uint16)*4) != 0)
				{
					errorCode = SBG_INVALID_PARAMETER;
				}
			}
		}
	}
	else
	{
		//
		// NULL handle pass as argument
		//
		errorCode = SBG_NULL_POINTER;
	}

	return errorCode;
}

/*!
 *	Get the manual rotation to be applied on sensors input in post-rotation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	quaternion						Quaternion used for orientation.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetManualPostOrientationOffset(SbgCanDeviceHandle deviceHandle, float quaternion[4])
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Build and send the command message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_POST_ORIENTATION_OFFSET, 0, NULL);

	//
	// Check if the message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_POST_ORIENTATION_OFFSET, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to get the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer length is valid
			//
			if (receivedLength == sizeof(uint16)*4)
			{
				//
				//	Return the converted data
				//
				if (quaternion)
				{
					quaternion[0] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 0*sizeof(uint16)) );
					quaternion[1] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 1*sizeof(uint16)) );
					quaternion[2] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 2*sizeof(uint16)) );
					quaternion[3] = sbgCanTargetToHostFrac16( *(uint16*)(receivedData + 3*sizeof(uint16)) );
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
