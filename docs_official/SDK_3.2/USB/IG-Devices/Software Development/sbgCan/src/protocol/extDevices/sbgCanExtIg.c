#include "sbgCanExtIg.h"
#include "../sbgCanCommandsExt.h"
#include <string.h>
#include "../sbgCanProtocolOutputMode.h"


/*!
 *	Set a manual orientation offset between the IG-500E and remote device, in a matrix form
 *	\param[in]	handle				A valid sbgCan device handle.
 *	\param[in]	matrixOffset		orientation offset between the IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtIgSetMatrixOffset(SbgCanDeviceHandle handle, const float matrixOffset[9])
{
	uint8 command[2*sizeof(uint8)+9*sizeof(uint32)];
	uint8 answer[256];
	uint16 answerSize;
	SbgErrorCode error;
	int8 i;

	//
	// Build up the external device command buffer
	//
	command[0] = SBG_CAN_EXT_CMD_IG_SET_MATRIX_OFFSET;
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
		if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_IG_ACK))
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
 *	Get the offset between the IG-500E orientation and the IG-Device orientation. <br>
 *	\param[in]	handle				A valid sbgCan device handle.
 *	\param[out]	matrixOffset		Orientation offset between the remote IG-Device and IG-500E
 *	\return							SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtIgGetMatrixOffset(SbgCanDeviceHandle handle, float matrixOffset[9])
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
	command[0] = SBG_CAN_EXT_CMD_IG_GET_MATRIX_OFFSET;
	
	

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
		if ((answer[0] == SBG_CAN_EXT_CMD_IG_RET_MATRIX_OFFSET) && (answerSize == (sizeof(uint8)+9*sizeof(uint32))))
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
		else if ((answerSize == 2) && (answer[0] == SBG_CAN_EXT_CMD_IG_ACK) && (answer[1] != SBG_NO_ERROR))
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
