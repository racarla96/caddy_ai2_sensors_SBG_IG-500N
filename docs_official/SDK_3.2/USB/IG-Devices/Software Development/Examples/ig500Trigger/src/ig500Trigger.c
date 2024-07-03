/*!
 *	\file		ig500Trigger.c
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		15/05/2012
 *
 *	\brief		C example that demonstrates continuous outputs.
 *
 *	This small example demonstrates how to configure the device in
 *	triggered output mode and to read data from it.
 *	
 *	Triggered output are a powerful mechanism used to get data
 *	from the device without any host intervention.
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
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <sbgCom.h>

/*!
 *	Structure used to store the received data.
 */
typedef struct _UserData
{
	float	accelerometers[3];	/*!< Accelerometers in m/s^2. */
	float	gpsVelocity[3];		/*!< GPS velocity in m/s. */
	uint32	triggerMask;		/*!< Store which new data we have. */
} UserData;

//----------------------------------------------------------------------//
//  Callback functions used by trigger mode                             //
//----------------------------------------------------------------------//

/*!
 *	Functon called each time we have an error on a triggered frame.
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	errorCode			Error code that have occured during a continuous operation.
 *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousErrorCallback function.
 */
void triggerErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
{
	char errorMsg[256];

	//
	// Convert our error code to a human readable error message
	//
	sbgComErrorToString(errorCode, errorMsg);

	//
	// Display an error message
	//
	fprintf(stderr,"continuousErrorCallback: We have received the following error %s\n", errorMsg);
	fflush(stderr);
}

/*!
 *	Functon called each time we have received a new data on a triggered frame.
 *	\param[in]	pHandler								Our associated protocol handle.
 *  \param[in]	triggerMask								Trigger bit mask indicating which data have generated the triggered output
 *	\param[in]	pOutput									Pointer to our filled SbgOutput struct.<br>
 *														Don't delete output as you don't have the ownership.
 *	\param[in]	pUsrArg									Pointer to our user defined argument.
 */
void triggerCallback(SbgProtocolHandleInt *handler, uint32 triggerMask, SbgOutput *pOutput, void *pUsrArg)
{
	UserData *pUserData;

	//
	// Check if our intput arguments are valid
	//
	if ( (pOutput) && (pUsrArg) )
	{
		//
		// Get our user buffer to store our received data
		//
		pUserData = (UserData*)pUsrArg;

		//
		// Copy our trigger mask
		//
		pUserData->triggerMask = triggerMask;

		//
		// Check if our output struct contains accelerometers data and if we have received new one (according to our triggers configuration)
		//
		if ( (pOutput->outputMask&SBG_OUTPUT_ACCELEROMETERS) && (triggerMask&SBG_TRIGGER_MAIN_LOOP_DIVIDER) )
		{
			//
			// We have received a new accelerometers data
			//
			pUserData->accelerometers[0] = pOutput->accelerometers[0];
			pUserData->accelerometers[1] = pOutput->accelerometers[1];
			pUserData->accelerometers[2] = pOutput->accelerometers[2];
		}

		//
		// Check if our output struct contains GPS velocity data and if we have received new one (according to our triggers configuration)
		//
		if ( (pOutput->outputMask&SBG_OUTPUT_GPS_NAVIGATION) && (triggerMask&SBG_TRIGGER_GPS_VELOCITY) )
		{
			//
			// We have received a new GPS velocity data
			//
			pUserData->gpsVelocity[0] = pOutput->gpsVelocity[0] / 100.0f;
			pUserData->gpsVelocity[1] = pOutput->gpsVelocity[1] / 100.0f;
			pUserData->gpsVelocity[2] = pOutput->gpsVelocity[2] / 100.0f;
		}
	}
}


//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

/*!
 *	Main entry point.
 *	\param[in]	argc		Number of input arguments.
 *	\param[in]	argv		Input arguments as an array of strings.
 *	\return					0 if no error and -1 in case of error.
 */
int main(int argc, char** argv)
{
	SbgProtocolHandle protocolHandle;
	SbgErrorCode error = SBG_NO_ERROR;
	UserData receivedData;

	//
	// Initialise our receivedData struct to 0
	//
	memset(&receivedData, 0x00, sizeof(UserData));

	//
	// Init our communications with the device (Please change here the com port and baud rate)
	//
	error = sbgComInit("COM6", 115200, &protocolHandle);

	//
	// Test if the sbgCom initialization was ok
	//
	if (error == SBG_NO_ERROR)
	{
		//
		// Configure the first trigger channel on the main loop divider and outputs the accelerometers
		//
		error = sbgSetTriggeredMode(protocolHandle, 0, SBG_TRIGGER_MAIN_LOOP_DIVIDER, SBG_OUTPUT_ACCELEROMETERS);

		if (error == SBG_NO_ERROR)
		{
			//
			// Configure the second trigger channel on a new GPS velocity information and outputs the GPS navigation such as velocity
			//
			error = sbgSetTriggeredMode(protocolHandle, 1, SBG_TRIGGER_GPS_VELOCITY, SBG_OUTPUT_GPS_NAVIGATION);

			if (error == SBG_NO_ERROR)
			{
				//
				// We could configure the device, now, simply enable the triggered output mode, with a main loop freq divider at 4
				//
				error = sbgSetContinuousMode(protocolHandle, SBG_TRIGGERED_MODE_ENABLE, 4);
				
				if (error == SBG_NO_ERROR)
				{
					//
					// Now, we can define our trigger handlers and optionaly a error handler
					// The trigger callback function will store euler angles and velocity in the eulerVelocity list
					//
					sbgSetContinuousErrorCallback(protocolHandle, triggerErrorCallback, NULL);
					sbgSetTriggeredModeCallback(protocolHandle, triggerCallback, &receivedData);

					//
					// Print our header
					//
					printf("Acceleromters\t\t\tGPS Velocity\n");

					//
					// Loop forever
					//
					do
					{
						//
						// Check for new frames received
						//
						sbgProtocolContinuousModeHandle(protocolHandle);

						//
						// Display values on the screen
						//
						printf("%3.2f\t%3.2f\t%3.2f\t\t%3.2f\t%3.2f\t%3.2f\n",	receivedData.accelerometers[0], receivedData.accelerometers[1], receivedData.accelerometers[2],
																				receivedData.gpsVelocity[0], receivedData.gpsVelocity[1], receivedData.gpsVelocity[2]);
						//
						// Unload the CPU
						//
						sbgSleep(10);

					} while(1);
				}
				else
				{
					fprintf(stderr, "Could not set the trigger configuration\n");
				}

			}
			else
			{
				fprintf(stderr, "Could not set the trigger configuration\n");
			}
		}
		else
		{
			fprintf(stderr, "Could not set the trigger configuration\n");
		}

		//
		// Close our protocol system
		//
		sbgProtocolClose(protocolHandle);
	}
	else
	{
		fprintf(stderr, "Unable to open IG-500 device\n");
	}

	return 0;
}
