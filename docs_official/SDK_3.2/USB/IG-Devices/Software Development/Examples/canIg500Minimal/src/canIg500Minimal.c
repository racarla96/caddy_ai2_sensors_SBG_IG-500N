/*!
 *	\file		canIg500Minimal.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		20/03/2011
 *
 *	\brief		Minimal CAN example for IG-Devices
 *
 *	This minimal project illustrates how simple
 *	it is to implement a small program interfaced
 *	with an IG-500 device on the CAN bus.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2012, SBG Systems SAS. All rights reserved.
 *	
 *	This source code is intended for use only by SBG Systems SAS and
 *	those that have explicit written permission to use it from
 *	SBG Systems SAS.
 *	
 *	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 *	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *	PARTICULAR PURPOSE.
 */
 
#include <stdio.h>
#include <conio.h>
#include <sbgCan.h>

//----------------------------------------------------------------------//
//-  Callback routines                                                 -//
//----------------------------------------------------------------------//

/*!
 * Basic error handler
 * \param[in]	pDeviceHandler		Device handle corresponding to the error
 * \param[in]	errorCode			Error code returned
 * \param[in]	pUsrArg				User parameter. Unused here
 */
void errorHandle(SbgCanDeviceHandleStr *pDeviceHandler, SbgErrorCode errorCode, void *pUsrArg)
{
	//
	// Handle incomming error
	//
	printf("Error on input frame: %d\n", errorCode);
}

/*!
 * Basic output handler
 * \param[in]	pDeviceHandler		Device handle corresponding to the error
 * \param[in]	pOutput				Output structure containing our output type and the corresponding data
 * \param[in]	pUsrArg				User parameter. Unused here
 */
void outputHandle(SbgCanDeviceHandleStr *pDeviceHandler, const SbgCanOutputDataStr *pOutput, void *pUsrArg)
{
	//
	// Handle incomming message: Check what frame it is.
	// Here, we will only handle euler angles
	//
	if (pOutput->outputId == SBG_CAN_ID_OUTPUT_EULER)
	{
		printf("Gyroscopes values:\t %3.2f\t %3.2f\t %3.2f\r",	SBG_RAD_TO_DEG(pOutput->outputData.eulerAngle[0]), 
																SBG_RAD_TO_DEG(pOutput->outputData.eulerAngle[1]), 
																SBG_RAD_TO_DEG(pOutput->outputData.eulerAngle[2]));
	}
}

//----------------------------------------------------------------------//
//-  Main program                                                      -//
//----------------------------------------------------------------------//

/*!
 *	Main entry point.
 *	\param[in]	argc		Number of input arguments.
 *	\param[in]	argv		Input arguments as an array of strings.
 *	\return					0 if no error and -1 in case of error.
 */
int main(int argc, char **argv)
{
	SbgCanBusHandle handle;			// The CAN bus handle variable
	SbgCanDeviceHandle device;		// The CAN IG-500 device handle variable
	uint32 errorCouner;				// An error counter used with the incomming messages

	//
	// Init our CAN interface (Please change here the can port and bit rate)
	//
	if (sbgCanBusInit(&handle, "CAN0", 1000) == SBG_NO_ERROR)
	{
		//
		// Try to add a device to the communication protocol
		// Here we consider the default output IDs so no parameter is given for ID list
		//
		if (sbgCanAddDevice(&device, handle, NULL) == SBG_NO_ERROR)
		{
			//
			// Set the trigger output and error handles
			// Our output callback will print euler angles on screen
			//
			if (sbgCanSetDeviceOutputCallback(device, outputHandle, NULL) == SBG_NO_ERROR)
			{
				if (sbgCanSetDeviceErrorCallback(device, errorHandle, NULL) == SBG_NO_ERROR)
				{
					printf("CAN bus and Device fully initialized.\n Press any key to exit.\n\nEuler Angles output:\n");

					//
					// Now the callback are defined.
					// We can start the main loop, until a key is pressed
					//
					while (!_kbhit())
					{
						//
						// Simply handle incomming messages and make a pause to unload the CPU
						//
						sbgCanBusContinuousReceptionHandle(handle, &errorCouner);
						sbgSleep(10);
					}
				}
				else
				{
					printf("Error: Could not define callback\n");
				}
			}
			else
			{
				printf("Error: Could not define callback\n");
			}
			
			printf("\n");

			//
			//	Remove the device
			//
			sbgCanRemoveDevice(&device);
		}
		else
		{
			printf("Error: Unable to open IG-500 device\n");
		}

		//
		// Close our protocol system
		//
		sbgCanBusClose(&handle);

		return 0;
	}
	else
	{
		printf("Error: Unable to open CAN Bus\n");
		return -1;
	}
}