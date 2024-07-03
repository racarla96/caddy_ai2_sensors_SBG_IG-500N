#include "sbgCanCommandsGps.h"
#include "sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- Gps and reference pressure configuration commands				   -//
//----------------------------------------------------------------------//

/*!
 *	Set the reference pressure for altitude calculation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	refPressure						Reference pressure.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetRefPressure(SbgCanDeviceHandle deviceHandle, uint32 refPressure)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data
	//
	*(uint32*)(dataBuffer) = sbgCanHostToTarget32(refPressure);

	//
	// Create and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_REFERENCE_PRESSURE, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_REFERENCE_PRESSURE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
 *	Get the reference pressure for altitude calculation.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pRefPressure					Pointer to an uint32 used to hold the reference pressure.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetRefPressure(SbgCanDeviceHandle deviceHandle, uint32 *pRefPressure)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;
	
	//
	// Create and send the CAN message
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_REFERENCE_PRESSURE, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_REFERENCE_PRESSURE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check if we were able to receive a valid answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength == sizeof(uint32))
			{
				//
				// Return the configuration if needed
				//
				if (pRefPressure)
				{
					*pRefPressure = sbgCanTargetToHost32( *(uint32*)(receivedData) );
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
