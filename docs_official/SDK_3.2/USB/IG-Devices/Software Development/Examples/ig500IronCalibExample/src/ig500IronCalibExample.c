/*!
 *	\file		ig500Trigger.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		05/05/2010.
 *
 *	This small project illustrates how to use the sbgIronCalibration library.
 *	This library is used to compute a 2D or 3D magnetic calibration to account for
 *	soft and hard iron effects.
 */
#include "stdio.h"
#include "time.h"
#include <stdlib.h>
#include <string.h>
#include <sbgCom.h>
#include <sbgIronCalibration.h>

//----------------------------------------------------------------------//
//  Callback functions used by trigger mode                             //
//----------------------------------------------------------------------//

/*!
 *	Called each time we have an error on a triggered frame.
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	errorCode			Error code that have occurred during a continuous operation.
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
 *	Called each time we have received a new data on a triggered frame.
 *	\param[in]	pHandler								Our associated protocol handle.
 *  \param[in]	triggerMask								Trigger bit mask indicating which data have generated the triggered output
 *	\param[in]	pOutput									Pointer to our filled SbgOutput struct.<br>
 *														Don't delete output as you don't have the ownership.
 *	\param[in]	pUsrArg									Pointer to our user defined argument.
 */
void triggerCallback(SbgProtocolHandleInt *handler, uint32 triggerMask, SbgOutput *pOutput, void *pUsrArg)
{
	SbgIronCalibHandle ironCalibHandle;

	//
	// Check if our intput arguments are valid
	//
	if ( (pOutput) && (pUsrArg) )
	{
		//
		// Get the sbgIronCalibration library handle
		//
		ironCalibHandle = (SbgIronCalibHandle)pUsrArg;

		//
		// Check if the output struct contains magnetometers calibration data
		//
		if (pOutput->outputMask&SBG_OUTPUT_MAG_CALIB_DATA)
		{
			//
			// We have received a new accelerometers data
			//
			sbgIronCalibAddPoint(ironCalibHandle, pOutput->magCalibData);
		}	
	}
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

int main(int argc, char** argv)
{
	SbgIronCalibHandle calibLibHandle;
	SbgIronCalibResults calibResults;
	SbgProtocolHandle protocolHandle;
	SbgErrorCode errorCode = SBG_NO_ERROR;
	char errorMsg[256];
	
	//
	// Display a welcome message with libraries version
	//
	printf("Welcome to the SBG Systems IG-500 magnetometers calibration sample.\n");
	printf("This demo is using the following libraries versions:\n");
	printf("\t- sbgIronCalibration library version: %s\n", sbgIronCalibGetVersionAsString());
	printf("\t- sbgCom library version: %s\n\n\n", sbgComGetVersionAsString());
	
	//
	// Create the iron calibration library (put your own licence number)
	//
	errorCode = sbgIronCalibInit("XXXXX-XXXXX-XXXXX-XXXXX-XXXXX", &calibLibHandle);
	
	//
	// Check if the sbgIronCalibration library has been initialised
	//
	if (errorCode == SBG_NO_ERROR)
	{		
		//
		// Initialize the communications with the device (Please change here the com port and baud rate)
		//
		errorCode = sbgComInit("COM1", 115200, &protocolHandle);

		//
		// Check if the sbgCom library has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Configure the first trigger channel on the main loop divider and outputs the magnetometers calibration data
			//
			errorCode = sbgSetTriggeredMode(protocolHandle, 0, SBG_TRIGGER_MAIN_LOOP_DIVIDER, SBG_OUTPUT_MAG_CALIB_DATA);

			if (errorCode == SBG_NO_ERROR)
			{
				//
				// Enable the trigger mode for data output and define a main loop divider of 1 (normally, a 100 Hz output)
				//
				errorCode = sbgSetContinuousMode(protocolHandle, SBG_TRIGGERED_MODE_ENABLE, 1);
				
				//
				// Test that the command has been executed successfully
				//
				if (errorCode == SBG_NO_ERROR)
				{
					//
					// Now, we can define the trigger handlers and optionally an error handler
					// The trigger callback function will store magnetometers calibration data into the sbgIronCalibration library
					//
					sbgSetContinuousErrorCallback(protocolHandle, triggerErrorCallback, NULL);
					sbgSetTriggeredModeCallback(protocolHandle, triggerCallback, calibLibHandle);

					//
					// Acquire at least 1000 magnetometers data
					//
					while (sbgIronCalibGetNumData(calibLibHandle) < 1000)
					{
						//
						// Check for new data received
						//
						sbgProtocolContinuousModeHandle(protocolHandle);

						//
						// Unload the CPU
						//
						sbgSleep(10);

						//
						// Display a progress message
						//
						printf("Acquiring data %.4u/1000...\r", sbgIronCalibGetNumData(calibLibHandle));
					}

					//
					// Display a message to inform the user that we are computing the magnetometers calibration
					//
					printf("\nCompute magnetometers calibration...\n");

					//
					// We have enough magnetometers point so compute the magnetometers calibration
					//
					errorCode = sbgIronCalibCompute(calibLibHandle, SBG_CALIB_MODE_2D_HORIZONTAL, &calibResults);
				
					//
					// Check if we were able to calibrate our magnetometers
					//
					if (errorCode == SBG_NO_ERROR)
					{
						//
						// Display our calibration results
						//
						printf("Num points: %u\n", sbgIronCalibGetNumData(calibLibHandle));
						printf("Before avg: %f\tAfter avg: %f\tExpected accuracy: %f\n", calibResults.m_beforeAvgDeviation, calibResults.m_afterAvgDeviation, calibResults.m_avgExpectedAccuracy);
						printf("Before max: %f\tAfter max: %f\tWorst accuracy: %f\n", calibResults.m_beforeMaxDeviation, calibResults.m_afterMaxDeviation, calibResults.m_maxExpectedAccuracy);
					
						//
						// Send the computed magnetometers calibration to the device
						//
						errorCode = sbgCalibMagnetometersSetTransformations(protocolHandle, calibResults.m_offsetVector, calibResults.m_crossAxisMatrix);

						//
						// Test if the magnetometers calibration has been set
						//
						if (errorCode == SBG_NO_ERROR)
						{
							//
							// Save the just uploaded magnetometers calibration to the device FLASH memory
							//
							errorCode = sbgCalibMagnetometers(protocolHandle, SBG_CALIB_MAGS_SAVE);

							//
							// Test if the magnetometers calibration has been saved to FLASH
							//
							if (errorCode == SBG_NO_ERROR)
							{
								printf("New magnetometers calibration has been successfully saved to FLASH memory.\n");
							}
							else
							{
								fprintf(stderr, "Unable to save the new magnetometers calibration to device FLASH memory.\n");
							}
						}
						else
						{
							fprintf(stderr, "Unable to upload the new magnetometers calibration to the device.\n");
						}						
					}
					else
					{
						//
						// Build our error message and display it
						//
						sbgIronCalibErrorToString(errorCode, errorMsg);
						fprintf(stderr,"Unable to calibrate our magnetometers with error: %s\n", errorMsg);
					}
				}
				else
				{
					fprintf(stderr, "Could not enable the trigger mode.\n");
				}
			}
			else
			{
				fprintf(stderr, "Could not set the trigger configuration.\n");
			}

			//
			// Close our protocol system
			//
			sbgProtocolClose(protocolHandle);
		}
		else
		{
			//
			// Unable to initialize the sbgCom library
			//
			fprintf(stderr, "Unable to initialize the sbgCom library, please check the serial configuration.\n");
		}

		//
		// Close the sbgIronCalibration library
		//
		sbgIronCalibClose(calibLibHandle);
	}
	else
	{
		//
		// Unable to initialize the sbgIronCalibration library.
		// Build the error message and display it
		//
		sbgIronCalibErrorToString(errorCode, errorMsg);
		fprintf(stderr, "sbgIronCalibInit failed with: %s.\n", errorMsg);
	}

	return -1;
}
