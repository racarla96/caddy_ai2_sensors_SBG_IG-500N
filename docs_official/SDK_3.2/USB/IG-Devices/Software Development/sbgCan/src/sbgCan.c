#include "sbgCan.h"
#include "sbgCanVersion.h"
#include <string.h>
#include <stdio.h>

/*!
 *	Returns an integer representing the version of the sbgCan library.
 *	\return									An integer representing the version of the sbgCan library.<br>
 *											Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 sbgCanGetVersion(void)
{
	return SBG_CAN_VERSION_U;
}

/*!
 *	Retreive the sbgCan library version as a string (1.0.0.0).
 *	\return										Null terminated string that contains the sbgCan library version.
 */
const char *sbgCanGetVersionAsString(void)
{
	return SBG_CAN_VERSION;
}

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode					The errorCode to convert into a string.
 *	\param[out]	errorMsg					String buffer used to hold the error string.
 */
void sbgCanErrorToString(char errorMsg[256], SbgErrorCode errorCode)
{
	if (errorMsg)
	{
		//
		// For each error code, copy the error msg
		//
		switch (errorCode)
		{
		case SBG_NO_ERROR:
			strcpy(errorMsg, "SBG_NO_ERROR: No error."); 
			break;
		case SBG_ERROR:
			strcpy(errorMsg, "SBG_ERROR: Generic error."); 
			break;
		case SBG_NULL_POINTER:
			strcpy(errorMsg, "SBG_NULL_POINTER: A pointer is null."); 
			break;
		case SBG_INVALID_CRC:
			strcpy(errorMsg, "SBG_INVALID_CRC: The received frame has an invalid CRC.");
			break;
		case SBG_INVALID_FRAME:
			strcpy(errorMsg, "SBG_INVALID_FRAME: The received frame is invalid.");
			break;
		case SBG_TIME_OUT:
			strcpy(errorMsg, "SBG_TIME_OUT: We have a time out during frame reception.");
			break;
		case SBG_WRITE_ERROR:
			strcpy(errorMsg, "SBG_WRITE_ERROR: All bytes hasn't been written.");
			break;
		case SBG_READ_ERROR:
			strcpy(errorMsg, "SBG_READ_ERROR: All bytes hasn't been read.");
			break;
		case SBG_BUFFER_OVERFLOW:
			strcpy(errorMsg, "SBG_BUFFER_OVERFLOW: A buffer is too small to contain so much data.");
			break;
		case SBG_INVALID_PARAMETER:
			strcpy(errorMsg, "SBG_INVALID_PARAMETER: An invalid parameter has been found.");
			break;
		case SBG_NOT_READY:
			strcpy(errorMsg, "SBG_NOT_READY: A device isn't ready (Rx isn't ready for example).");
			break;
		case SBG_MALLOC_FAILED:
			strcpy(errorMsg, "SBG_MALLOC_FAILED: Failed to allocate a buffer.");
			break;
		case SGB_CALIB_MAG_NOT_ENOUGH_POINTS:
			strcpy(errorMsg, "SGB_CALIB_MAG_NOT_ENOUGH_POINTS: Not enough points were available to perform magnetometers calibration.");
			break;
		case SBG_CALIB_MAG_INVALID_TAKE:
			strcpy(errorMsg, "SBG_CALIB_MAG_INVALID_TAKE: The calibration procedure could not be properly executed due to insufficient precision.");
			break;
		case SBG_CALIB_MAG_SATURATION:
			strcpy(errorMsg, "SBG_CALIB_MAG_SATURATION: Saturation were detected when attempt to calibrate magnetos.");
			break;
		case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE:
			strcpy(errorMsg, "SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE: 2D calibration procedure could not be performed.");
			break;
		case SBG_DEVICE_NOT_FOUND:
			strcpy(errorMsg, "SBG_DEVICE_NOT_FOUND: A device couldn't be founded or opened.");
			break;
		case SBG_OPERATION_CANCELLED:
			strcpy(errorMsg, "SBG_OPERATION_CANCELLED: An operation has been cancelled by a user.");
			break;
		case SBG_NOT_CONTINUOUS_FRAME:
			strcpy(errorMsg, "SBG_NOT_CONTINUOUS_FRAME: We have received a frame that isn't a continuous one.");
			break;
		case SBG_INCOMPATIBLE_HARDWARE:
			strcpy(errorMsg, "SBG_INCOMPATIBLE_HARDWARE: Hence valid, the configuration cannot be executed because of incompatible hardware.");
			break;
		default:
			sprintf(errorMsg, "Undefined error code: %u", errorCode);
			break;
		}
	}
}