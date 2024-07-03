/*!
 *	\file		ig500LogMagFile.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		23/11/2009
 *
 *	\brief		C example that demonstrates log mag files generation.
 *
 *	This small example demonstrates how to generate a .mag log file
 *	using data outputted by an IG-Device.
 *	
 *	Mag log files are used either by the sbgCenter or the
 *	sbgIronCalibration library to compute remotely
 *	magnetometers soft and hard iron calibration data.
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
#include <sbgCom.h>

#ifdef WIN32
#include <conio.h>
#else
#include <unistd.h>
#ifndef _kbhit
/*!
 *	Define a kbhit equivalent in unix environment.
 *	\return							The pressed key value.
 */
int _kbhit(void)
{
	struct timeval tv = { 0, 0 };
	fd_set readfds;
	
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);
	
	return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) == 1;
}
#endif
#endif

/*!
 *	Functon called each time we have an error on a continuous frame.
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	errorCode			Error code that have occured during a continuous operation.
 *	\param[in]	pUsrArg				User argument pointer as defined in sbgSetContinuousErrorCallback function.
 */
void continuousErrorCallback(SbgProtocolHandleInt *pHandler, SbgErrorCode errorCode, void *pUsrArg)
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
 *	Functon called each time we have received a new data on a continuous frame.
 *  Here, we will log magnetic data in the file pointer passed as an argument
 *	\param[in]	pHandler			Our sbgCom protocol handler.
 *	\param[in]	pOutput				Pointer on our received data struct. (You don't have the ownership so don't delete it!)
 *	\param[in]	filePointer			User argument pointer as defined in sbgSetContinuousModeCallback function. <br>
									In our example, it is the log file structure pointer
 */
void continuousCallback(SbgProtocolHandleInt *handler, SbgOutput *pOutput, void *pFile)
{
	//
	// First, check pointers
	//
	if ((pOutput) && (pFile))
	{
		//
		// Here, we actually log the magnetic field calibration data
		//
		fwrite(pOutput->magCalibData, sizeof(uint8), 12, (FILE*)pFile);
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
	FILE *pFile;

	//
	// Init our communications with the device (Please change here the com port and baud rate
	// On MacOS X, use for example "/dev/cu.SLAB_USBtoUART"
	// On Windows, use for example "COM3"
	//
	if (sbgComInit("COM6", 115200, &protocolHandle) == SBG_NO_ERROR)
	{
		//
		// Wait until the device has been initialised
		//
		sbgSleep(50);

		//
		// Display the title
		//
		printf("Magnetic Calibration Data Logger:\n");
		
		//
		// Define the default output mask for continuous mode
		// This setting is not saved in flash non volatile memory
		//
		if (sbgSetDefaultOutputMask(protocolHandle, SBG_OUTPUT_MAG_CALIB_DATA) == SBG_NO_ERROR)
		{
			//
			// This command actually enables continuous mode with Output divider set to 2
			// Setting is not saved in flash memory
			//
			if (sbgSetContinuousMode(protocolHandle, SBG_CONTINUOUS_MODE_ENABLE, 2) == SBG_NO_ERROR)
			{
				//
				// Open our log file in binary format
				//
				pFile = fopen("magLog.mag","wb");

				if (pFile)
				{
					//
					// Now, data logging will really start
					// Define our continuous handlers, with the file pointer as user pointer
					//
					sbgSetContinuousErrorCallback(protocolHandle, continuousErrorCallback, NULL);
					sbgSetContinuousModeCallback(protocolHandle, continuousCallback, pFile);

					//
					// Start our points acquisition until the user press a key
					//
					printf("Please start rotating the device for calibration.\nPress a key to finish.\n");
					fflush(stdin);

					//
					// Loop until user wants to finish
					//
					while (!_kbhit())
					{
						//
						// Handle continuous frames once every 10ms
						//
						sbgProtocolContinuousModeHandle(protocolHandle);
						sbgSleep(10);
					}

					//
					// Finally, close the log file
					//
					fclose(pFile);
				}
				else
				{
					printf("Unable to open log file\n");
				}
			}
			else
			{
				fprintf(stderr,"Unable to enable continuous mode\n");
				fflush(stderr);
			}
		}
		else
		{
			fprintf(stderr,"Unable to set default output mask\n");
			fflush(stderr);
		}

		//
		// Close our protocol system
		//
		sbgProtocolClose(protocolHandle);

		return 0;
	}
	else
	{
		fprintf(stderr, "Unable to open IG-500 device\n");
		fflush(stderr);
		return -1;
	}
}
