#include "sbgCanCommandsSetting.h"
#include "sbgCanProtocolOutputMode.h"
#include "../time/sbgTime.h"

//----------------------------------------------------------------------//
//- Settings commands operations                                       -//
//----------------------------------------------------------------------//

/*!
 *	Saves the current device settings into its flash memory and then return the result.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSaveSettings(SbgCanDeviceHandle deviceHandle)
{
	SbgErrorCode errorCode;
	uint8 receivedLength;
	uint8 receivedData[8];

	//
	// Check if we have a valid can device handle
	//
	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Create and send the CAN command
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_SAVE_SETTINGS, 0, NULL);
	
		//
		// Check if we were able to send the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_SAVE_SETTINGS, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
				else if (receivedData[0] != SBG_CAN_OPERATION_SUCCESS)
				{
					//
					// The save settings command has failed
					//
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
 *	Restores all settings to factory defaults and save them to FLASH memory.										<br>
 *	Calibration data such as gyroscopes bias and magnetometers calibration are not affected by this command!		<br>
 *	The device bit rate is reseted to 1000 Mbit/s.																	<br>
 *	The device send the result of this command afterward with the default device bit rate.							<br>
 *	This command change the bit rate on the device and the library.													<br>
 *	To change only the bit rate used by the library, you have to use the function sbgCanBusChangeBitRate.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanRestoreSettings(SbgCanDeviceHandle deviceHandle)
{
	SbgErrorCode errorCode;
	uint8 receivedLength;
	uint8 receivedData[8];

	//
	// Check that we have a valid device handle
	//
	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Create and send the CAN command
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_RESTORE_SETTINGS, 0, NULL);

		//
		// Check if we were able to send the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_RESTORE_SETTINGS, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
				else if (receivedData[0] != SBG_CAN_OPERATION_SUCCESS)
				{
					//
					// The save settings command has failed
					//
					errorCode = SBG_INVALID_PARAMETER;
				}
				else
				{
					//
					// Change the library baud rate to default value (1000 kb/s)
					//
					errorCode = sbgCanBusChangeBitRate(deviceHandle->canBusHandle, 1000);
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
 *	Gets device information such as product code, hardware and firmware revisions.																	<br>
 *	For versions, use SBG_VERSION_GET_MAJOR, SBG_VERSION_GET_MINOR, SBG_VERSION_GET_REV and SBG_VERSION_GET_BUILD to extract versions inforamtion.	<br>
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	productCode						Device product code string.
 *	\param[out]	pDeviceId						The intern device id.
 *	\param[out]	pFirmwareVersion				The device firmware version. ('1.0.0.0')
 *	\param[out]	pCalibDataVersion				The device calibration data version. ('1.0.0.0')
 *	\param[out]	pMainBoardVersion				The device main board hardware version. ('1.0.0.0')
 *	\param[out]	pGpsBoardVersion				The device gps/top board hardware version. ('1.0.0.0')
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetInfo(SbgCanDeviceHandle deviceHandle, char productCode[32], uint32 *pDeviceId, uint32 *pFirmwareVersion, uint32 *pCalibDataVersion, uint32 *pMainBoardVersion, uint32 *pGpsBoardVersion)
{
	SbgErrorCode errorCode;
	uint8 receivedLength;
	uint8 receivedData[8];
	uint32 i;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_DEVICE_INFO, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// We have to receive 13 frames so wait for all the answer to get the device information
		//
		for (i=0; i<13; i++)
		{
			//
			// Try to receive a CAN message
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_DEVICE_INFO, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
				
			//
			// Check if we were able to receive the CAN message
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the answer is valid
				//
				if (receivedLength == sizeof(uint8)+sizeof(uint32))
				{
					//
					// Check if we are receiving the product code
					//
					if ( (productCode) && (receivedData[0] <= 7) )
					{
						*(((uint32*)productCode)+i) = *(uint32*)(receivedData+sizeof(uint8));
					}
					if ( (pDeviceId) && (receivedData[0] == 8) )
					{
						*pDeviceId = sbgCanTargetToHost32( *(uint32*)(receivedData + sizeof(uint8)) );
					}
					if ( (pFirmwareVersion) && (receivedData[0] == 9) )
					{
						*(uint32*)pFirmwareVersion = sbgCanTargetToHost32( *(uint32*)(receivedData + sizeof(uint8)) ); 
					}
					if ( (pCalibDataVersion) && (receivedData[0] == 10) )
					{
						*(uint32*)pCalibDataVersion = sbgCanTargetToHost32( *(uint32*)(receivedData + sizeof(uint8)) );
					}
					if ( (pMainBoardVersion) && (receivedData[0] == 11) )
					{
						*(uint32*)pMainBoardVersion = sbgCanTargetToHost32( *(uint32*)(receivedData + sizeof(uint8)) );
					}
					if ( (pGpsBoardVersion) && (receivedData[0] == 12) )
					{
						*pGpsBoardVersion = sbgCanTargetToHost32( *(uint32*)(receivedData + sizeof(uint8)) );
					}
				}
				else
				{
					//
					// We have received an invalid frame so exit
					//
					errorCode = SBG_INVALID_FRAME;
					break;
				}
			}
			else
			{
				//
				// Unable to receive a CAN frame so exit
				//
				break;
			}
		}
	}

	return errorCode;
}

/*!
 *	Defines a user selectable id for the device.
 *	\param[in]	deviceHandle					A valid ig can device handle.
  *	\param[in]	userId							The new device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetUserId(SbgCanDeviceHandle deviceHandle, uint32 userId)
{
	uint8 dataBuffer[sizeof(uint32)];
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	*(uint32*)(dataBuffer) = sbgCanHostToTarget32(userId);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_USER_ID, sizeof(dataBuffer), dataBuffer);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_USER_ID, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
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
 *	Returns the device user id.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pUserId							Pointer used to hold the device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetUserId(SbgCanDeviceHandle deviceHandle, uint32 *pUserId)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_USER_ID, 0, NULL);

	//
	// Check that the CAN message has been sent
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_USER_ID, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(uint32))
			{
				//
				//If needed, return the user id
				//
				if (pUserId)
				{
					//
					// Return user id
					//
					*pUserId = sbgCanTargetToHost32(*(uint32*)(receivedData));
				}
			}
			else
			{
				//
				// Invalid frame was received
				//
				errorCode = SBG_INVALID_FRAME;
			}
		}
	}

	return errorCode;
}



/*!
 *	For a specific CAN message, defines a custom id or disable the message using the specific id SBG_CAN_DISABLED_FRAME.<br>
 *	For example, to define SBG_CAN_OUTPUT_EULER CAN message id to 0x0CD3 use the following syntax:<br>
 *	sbgCanSetFrameId(deviceHandle, SBG_CAN_OUTPUT_EULER, 0x00000CD3);
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultMsgId					Id of the CAN message we would like to define (use an id from SbgCanId enum.)
 *	\param[in]	newMsgFormat					New CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[in]	newMsgId						The new CAN message id or SBG_CAN_DISABLED_FRAME to disable this message.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultMsgId, SbgCanMessageFormat newMsgFormat, uint32 newMsgId)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint16) + sizeof(uint8) + sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	*(uint16*)(dataBuffer) = sbgCanHostToTarget16((uint16)defaultMsgId);
	*(dataBuffer+sizeof(uint16)) = (uint8)newMsgFormat;
	*(uint32*)(dataBuffer+sizeof(uint16)+sizeof(uint8)) = sbgCanHostToTarget32(newMsgId);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_FRAME_ID, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_FRAME_ID, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
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
			else if (memcmp(receivedData, dataBuffer, sizeof(dataBuffer)) != 0)
			{
				//
				// The received buffer isn't the same as the sent one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
			else
			{
				//
				// We have received a valid answer so change the id for the library
				//
				sbgCanDeviceSetFrameId(deviceHandle, defaultMsgId, newMsgFormat, newMsgId);
			}
		}
	}

	return errorCode;
}

/*!
 *	For a specific CAN message, returns the used id.<br>
 *	If SBG_CAN_DISABLED_FRAME is returned, it means that this CAN message is disabled.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultMsgId					Id of the CAN message we would like to read its id (use an id from SbgCanId enum.)
  *	\param[out]	pNewMsgFormat					Used CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[out]	pNewMsgId						Used CAN message id or SBG_CAN_DISABLED_FRAME if this message is disabled.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultMsgId, SbgCanMessageFormat *pNewMsgFormat, uint32 *pNewMsgId)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint16)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	*((uint16*)dataBuffer) = sbgCanHostToTarget16((uint16)defaultMsgId);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_FRAME_ID, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_FRAME_ID, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint16)+sizeof(uint8)+sizeof(uint32)) 
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if ( memcmp(receivedData, dataBuffer, sizeof(uint16)) != 0)
			{
				//
				// The received frame index isn't the same as the requested one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
			else
			{
				//
				// We have received a valid frame so extract the settings and return it if needed
				//
				if (pNewMsgFormat)
				{
					*pNewMsgFormat = (SbgCanMessageFormat)*(receivedData+sizeof(uint16));
				}
				if (pNewMsgId)
				{
					*pNewMsgId = sbgCanTargetToHost32(*((uint32*) (receivedData+sizeof(uint8)+sizeof(uint16)) ));
				}
			}
		}
	}

	return errorCode;
}

/*!
 *	Sets the Low power modes for the IG-Device
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	devicePowerMode					Defines the device power mode then return it.
 *	\param[in]	gpsPowerMode					Defines the the GPS receiver power mode then return it. (leave to SBG_CAN_GPS_MAX_PERF if there is no GPS reveicer)	<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetLowPowerMode(SbgCanDeviceHandle deviceHandle, SbgCanPowerModeDevice devicePowerMode, SbgCanPowerModeGps gpsPowerMode)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[2];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = (uint8)devicePowerMode;
	dataBuffer[1] = (uint8)gpsPowerMode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOW_POWER_MODE, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOW_POWER_MODE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if ( memcmp(receivedData, dataBuffer, 2*sizeof(uint8)) != 0)
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
 *	Gets the Low power modes for the IG-Device
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDevicePowerMode				Returns the device power mode. (pass NULL if not used).
 *	\param[out]	pGpsPowerMode					Returns the GPS receiver power mode. (pass NULL if not used).	<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetLowPowerMode(SbgCanDeviceHandle deviceHandle, SbgCanPowerModeDevice *pDevicePowerMode, SbgCanPowerModeGps *pGpsPowerMode)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_LOW_POWER_MODE, 0, NULL);
		
	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_LOW_POWER_MODE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

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
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else
			{
				//
				// The received buffer is valid so return message data if needed
				//
				if (pDevicePowerMode)
				{
					*pDevicePowerMode = (SbgCanPowerModeDevice)(receivedData[0]);
				}
				if (pGpsPowerMode)
				{
					*pGpsPowerMode = (SbgCanPowerModeGps)(receivedData[1]);
				}
			}
		}
	}

	return errorCode;
}

/*!
 *	Write to the IG device's user buffer memory a unit32 at a specific index.
 *	The user buffer is an array of 160 uint32 that can be written or read.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	index							Index in the user buffer array to write data to.
 *	\param[in]	data							Data to write to IG device's memory at the specific index.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetUserBuffer(SbgCanDeviceHandle deviceHandle, uint8 index, uint32 data)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint8)+sizeof(uint32)];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = index;
	*(uint32*)(dataBuffer+sizeof(uint8)) = sbgCanHostToTarget32(data);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_USER_BUFFER, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_USER_BUFFER, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint8)+sizeof(uint32))
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, sizeof(uint8)+sizeof(uint32)) != 0)
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
 *	Read from the IG device's user buffer memory a unit32 at a specific index.
 *	The user buffer is an array of 160 uint32 that can be written or read.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	index							Index in the user buffer array to read data from.
 *	\param[in]	pData							Read data from the IG device's memory at the specific index.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetUserBuffer(SbgCanDeviceHandle deviceHandle, uint8 index, uint32 *pData)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;
	uint8 dataBuffer[sizeof(uint8)];

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = index;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_USER_BUFFER, sizeof(dataBuffer), dataBuffer);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for the answer
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_USER_BUFFER, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check if we were able to receive the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != sizeof(uint8)+sizeof(uint32))
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer, sizeof(uint8)) != 0)
			{
				//
				// The received index isn't the same as the requested one
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
			else
			{
				//
				// We have successfully received the user buffer value so return it if needed
				//
				if (pData)
				{
					//
					// Return the user buffer value
					//
					*pData = sbgCanTargetToHost32(*(uint32*)(receivedData + sizeof(uint8)));
				}
			}
		}
	}

	return errorCode;
}
/*!
 *	Defines the bit rate of the device CAN communication.<br>
 *	If the command is valid, the device acknoledge the bit rate change at the current speed and THEN change it's bit rate.		<br>
 *	This command change the bit rate on the device and the library.																<br>
 *	To change only the bit rate used by the library, you have to use the function sbgCanBusChangeBitRate.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	bitRate							The new device bit rate to use in kBit/s.										<br>
 *												Possible values are:
 *												<ul>
 *													<li>1 000 kBit/s</li>
 *													<li>  500 kBit/s</li>
 *													<li>  250 kBit/s</li>
 *													<li>  125 kBit/s</li>
 *													<li>  100 kBit/s</li>
 *													<li>   50 kBit/s</li>
 *													<li>   20 kBit/s</li>
 *													<li>   10 kBit/s</li>
 *												</ul>
 *	\return										If SBG_NO_ERROR, the device has been configured to the new speed.
 */
SbgErrorCode sbgCanSetProtocolMode(SbgCanDeviceHandle deviceHandle, uint16 bitRate)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint8 dataBuffer[sizeof(uint16) + 6*sizeof(uint8)];
	SbgErrorCode errorCode;

	//
	// Check that we have a valid device handle
	//
	if (deviceHandle != SBG_CAN_INVALID_DEVICE_HANDLE)
	{
		//
		// Fill the CAN message data buffer
		//
		*((uint32*)dataBuffer) = sbgCanHostToTarget16(bitRate);
		memset(dataBuffer + sizeof(uint16), 0x00, 6*sizeof(uint8));

		//
		// Create and send the CAN command
		//
		errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_PROTOCOL_MODE, sizeof(dataBuffer), dataBuffer);
	
		//
		// Check if we were able to send the CAN message
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Wait for an answer and return it
			//
			errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_PROTOCOL_MODE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

			//
			// Check that we have received the answer
			//
			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Check if the answer is valid
				//
				if (receivedLength != (sizeof(uint16)+6*sizeof(uint8)))
				{
					//
					// The answer length is invalid
					//
					errorCode = SBG_INVALID_FRAME;
				}
				else if (memcmp(receivedData, dataBuffer,  sizeof(uint16)+6*sizeof(uint8)) != 0)
				{
					//
					// The answer and the sent command are not the same
					//
					errorCode = SBG_INVALID_PARAMETER;
				}
				else
				{
					//
					// The answer and the sent command are the same so change the library bit rate
					//
					sbgCanBusChangeBitRate(deviceHandle->canBusHandle, bitRate);
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
 *	Command used to get the current bitrate used by the device.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pBitRate						The current device bit rate in in kBit/s.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetProtocolMode(SbgCanDeviceHandle deviceHandle, uint16 *pBitRate)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_PROTOCOL_MODE, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_PROTOCOL_MODE, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);
			
		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(uint16)+6*sizeof(uint8))
			{
				if (pBitRate)
				{
					*pBitRate = sbgCanTargetToHost16( *((uint16*)receivedData) );
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
 *	Defines the device advanced settings
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	settings						Advanced settings bitmask
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetAdvancedSettings(SbgCanDeviceHandle deviceHandle, uint32 settings)
{
	uint8 receivedLength;
	uint8 receivedData[8];
	uint32 dataBuffer[2];
	SbgErrorCode errorCode;

	//
	// Fill the CAN message data buffer
	//
	dataBuffer[0] = sbgCanHostToTarget32(settings);
	dataBuffer[1] = sbgCanHostToTarget32(0);

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ADVANCED_OPTIONS, sizeof(dataBuffer), (const uint8*)dataBuffer);
	
	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ADVANCED_OPTIONS, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, receivedData);

		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the answer is valid
			//
			if (receivedLength != (sizeof(dataBuffer)))
			{
				//
				// The answer length is invalid
				//
				errorCode = SBG_INVALID_FRAME;
			}
			else if (memcmp(receivedData, dataBuffer,  sizeof(uint16)+6*sizeof(uint8)) != 0)
			{
				//
				// The answer and the sent command are not the same
				//
				errorCode = SBG_INVALID_PARAMETER;
			}
		}
	}

	return errorCode;
}

/*!
 *	Get the device advanced settings
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pSettings						Advanced settings bitmask
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetAdvancedSettings(SbgCanDeviceHandle deviceHandle, uint32 *pSettings)
{
	uint8 receivedLength;
	uint32 receivedData[2];
	SbgErrorCode errorCode;

	//
	// Create and send the CAN command
	//
	errorCode = sbgCanDeviceSendSpecificMessage(deviceHandle, SBG_CAN_ID_ADVANCED_OPTIONS, 0, NULL);

	//
	// Check if we were able to send the CAN message
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Wait for an answer and return it
		//
		errorCode = sbgCanDeviceReceiveSpecificMessage(deviceHandle, SBG_CAN_ID_ADVANCED_OPTIONS, SBG_CAN_FRAME_RECEPTION_TIME_OUT, &receivedLength, (uint8*)receivedData);
			
		//
		// Check that we have received the answer
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Check if the response is valid
			//
			if (receivedLength == sizeof(receivedData))
			{
				if (pSettings)
				{
					*pSettings = sbgCanTargetToHost32(receivedData[0]);
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