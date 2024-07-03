#include "sbgCanExtNmea.h"
#include "../sbgCanCommandsExt.h"
#include "../sbgCanProtocolOutputMode.h"

//----------------------------------------------------------------------//
//- NMEA operations                                                    -//
//----------------------------------------------------------------------//

/*!
 *	Configures the remote NMEA GPS options: Altitude reference and distance between GPS antennas (if available)
 *	\param[in]	handle				A valid sbgCan device handle.
 *  \param[in]	options				NMEA GPS options. Possible choices are:
 *									- SBG_CAN_NMEA_OPT_ALT_ABOVE_MSL
 *									- SBG_CAN_NMEA_OPT_ALT_ABOVE_ELLIPSOID
 *	\param[in]	stdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °.
									Leave to 100000 if not used.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtNmeaSetOptions(SbgCanDeviceHandle handle, uint16 options, uint32 stdHeadingAcuracy)
{
	uint8 command[2+sizeof(uint16)+sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_NMEA_SET_OPTIONS;
	command[1] = 0;
	*((uint16*)(command+2*sizeof(uint8))) = sbgCanHostToTarget16(options);
	*((uint32*)(command+2*sizeof(uint8)+sizeof(uint16))) = sbgCanHostToTarget32(stdHeadingAcuracy);

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgCanExtDeviceConfig(handle,command,2*sizeof(uint8)+sizeof(uint16)+sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_NMEA_ACK))
		{
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Get the remote NMEA GPS options: Altitude reference and distance between GPS antennas (if available)
 *	\param[in]	handle				A valid sbgCan device handle.
 *  \param[out]	pOptions			NMEA GPS options. Possible choices are:
 *									- SBG_CAN_NMEA_OPT_ALT_ABOVE_MSL
 *									- SBG_CAN_NMEA_OPT_ALT_ABOVE_ELLIPSOID
 *	\param[out]	pStdHeadingAcuracy	True heading standard accuracy. Expressed in ° with 1 LSB = 10e-5 °.
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtNmeaGetOptions(SbgCanDeviceHandle handle, uint16 *pOptions, uint32 *pStdHeadingAcuracy)
{
	uint8 command[sizeof(uint8)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;


	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_NMEA_GET_OPTIONS;

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgCanExtDeviceConfig(handle,command,sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually a matrix output
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_CAN_EXT_CMD_NMEA_RET_OPTIONS) && (answerSize == (sizeof(uint8)+sizeof(uint16)+sizeof(uint32))))
		{
			if (pOptions)
			{
				*pOptions = sbgCanTargetToHostU16(*(uint16*)(answer+sizeof(uint8)));
			}
			if (pStdHeadingAcuracy)
			{
				*pStdHeadingAcuracy = sbgCanTargetToHostU32(*((uint32*)(answer+sizeof(uint8)+sizeof(uint16))));
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_NMEA_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Configures the offset between the IG-500E orientation and the GPS true Heading data. <br>
 *  IG-500E heading is then GPS True Heading - offset; This offset actually is stored as a rotation matrix. <br>
 *	See sbgCanExtNmeaSetMatrixOffset or sbgCanExtNmeagetMatrixOffset for more information
 *	\param[in]	handle				A valid sbgCom library handle.
 *	\param[in]	permanent			If TRUE, stores this settings in flash memory.
 *	\param[in]	offset				offset between the GPS true heading and IG-500E heading
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtNmeaSetYawOffset(SbgCanDeviceHandle handle, float offset)
{
	uint8 command[2+sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_NMEA_SET_YAW_OFFSET;
	command[1] = 0;
	*((uint32*)(command+2*sizeof(uint8))) = sbgCanHostToTargetFixed32(offset);

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgCanExtDeviceConfig(handle,command,2*sizeof(uint8)+sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_NMEA_ACK))
		{
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Configures the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCan device handle.
 *	\param[in]	matrixOffset		Orientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtNmeaSetMatrixOffset(SbgCanDeviceHandle handle, const float matrixOffset[9])
{
	uint8 command[2*sizeof(uint8)+9*sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_NMEA_SET_MATRIX_OFFSET;
	command[1] = 0;
	for (i=0;i<9;i++)
	{
		*((uint32*)(command+2*sizeof(uint8)+i*sizeof(uint32))) = sbgCanHostToTargetFixed32(matrixOffset[i]);
	}

	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgCanExtDeviceConfig(handle,command,2*sizeof(uint8)+9*sizeof(uint32),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually an acknowledge
		//
		if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_NMEA_ACK))
		{
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}

/*!
 *	Get the offset between the IG-500E orientation and the GPS dual antennas orientation. <br>
 *	\param[in]	handle				A valid sbgCan device handle.
 *	\param[out]	matrixOffset		Orientation offset between the GPS dual antennas and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtNmeaGetMatrixOffset(SbgCanDeviceHandle handle, float matrixOffset[9])
{
	uint8 command[sizeof(uint8)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	uint32* matrix;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_NMEA_GET_MATRIX_OFFSET;


	//
	// Send the external module configuration, and wait for the answer
	//
	error = sbgCanExtDeviceConfig(handle,command,sizeof(uint8),answer,&answerSize,256);

	if (error == SBG_NO_ERROR)
	{
		//
		// Check if the answer is actually a matrix output
		// Handle acknowledge received to make an error
		//
		if ((answer[0] == SBG_CAN_EXT_CMD_NMEA_RET_MATRIX_OFFSET) && (answerSize == (sizeof(uint8)+9*sizeof(uint32))))
		{
			if (matrixOffset)
			{
				matrix = (uint32*)(answer + sizeof(uint8));
				for (i=0;i<9;i++)
				{
					matrixOffset[i] = sbgCanTargetToHostFloat(matrix[i]);
				}
			}
		}
		else if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_NMEA_ACK) && (answer[1] != SBG_NO_ERROR))
		{
			//
			// We received an error
			//
			error = answer[1];
		}
		else
		{
			error = SBG_INVALID_FRAME;
		}
	}

	return error;
}
