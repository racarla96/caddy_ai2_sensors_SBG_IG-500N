/*!
 *	\file		ig500Minimal.c
 *  \author		SBG Systems (Alexis GUINAMARD)
 *	\date		28/03/2008
 *
 *	\brief		C example that initialize and read data from a IG-Device.
 *
 *	This small example demonstrates how to initialize the sbgCom library
 *	to read data from an IG-Device using a questions/answer mechanism.
 *
 *	To efficiently read data from the device, prefere the triggered mode
 *	that avoid host interventions provind a better timing.
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
	SbgErrorCode error;
	SbgOutput output;

	//
	// Init our communications with the device (Please change here the com port and baud rate)
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
		printf("Euler Angles:\n");

		//
		// Main loop
		//
		while (1)
		{
			//
			// Ask euler angles to the device
			//
			error = sbgGetSpecificOutput(protocolHandle, SBG_OUTPUT_EULER, &output);

			//
			// Test if we were able to get valid euler angles
			//
			if (error == SBG_NO_ERROR)
			{
				//
				// Displays sensor values in the console
				//
				printf("%3.2f\t%3.2f\t%3.2f\n",	SBG_RAD_TO_DEG(output.stateEuler[0]),
												SBG_RAD_TO_DEG(output.stateEuler[1]),
												SBG_RAD_TO_DEG(output.stateEuler[2]));
			}

			//
			// Small pause to unload CPU
			//
			sbgSleep(10);
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
		return -1;
	}
}
