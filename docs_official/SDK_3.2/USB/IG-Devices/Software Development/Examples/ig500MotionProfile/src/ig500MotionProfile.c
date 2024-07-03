/*!
 *	\file		ig500MotionProfile.c
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		15/05/2012
 *
 *	\brief		C example that demonstrates motion profile upload.
 *
 *	This small example demonstrates how to read a motion profile file
 *	and send it to the device.
 *	The device will when reboot and use the newly uploaded motion profile.
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
	FILE *pMpFile;
	uint8 *pMpBuffer;
	uint32	mpBufferSize;
	
	//
	// Open the motion profile file as a binary file
	//
	pMpFile = fopen("generalPurpose_10_v2_0_0_0.bin", "rb");

	//
	// Test that the motion profile file has been opened
	//
	if (pMpFile)
	{
		//
		// Read the file size
		//
		fseek(pMpFile, 0, SEEK_END);
		mpBufferSize = ftell(pMpFile);
		fseek(pMpFile, 0, SEEK_SET);

		//
		// Allocate a buffer used to hold the motion profile buffer
		//
		pMpBuffer = (uint8*)malloc(mpBufferSize * sizeof(uint8));

		//
		// Test that the buffer has been allocated
		//
		if (pMpBuffer)
		{
			//
			// Read the motion profile buffer and test if the read operation was ok
			//
			if (fread(pMpBuffer, sizeof(uint8), mpBufferSize, pMpFile) == mpBufferSize)
			{
				//
				// The motion profile buffer has been read so we have to upload it to the device
				// Init the communications with the device (Please change here the com port and baud rate)
				//
				if (sbgComInit("COM15", 115200, &protocolHandle) == SBG_NO_ERROR)
				{
					//
					// Send the motion profile buffer
					//
					if (sbgSetMotionProfile(protocolHandle, pMpBuffer, mpBufferSize) == SBG_NO_ERROR)
					{
						//
						// The motion profile has been uploaded successfully.
						// The device will save all current settings to FLASH memory and reboot.
						//
						printf("Motion profile successfully uploaded and applied.\n");
					}
					else
					{
						//
						// Unable to upload the motion profile
						//
						fprintf(stderr, "Unable to upload the motion profile to the device.\n");
					}

					//
					// Close the protocol system
					//
					sbgProtocolClose(protocolHandle);
				}
				else
				{
					//
					// Unable to initialize the sbgCom library
					//
					fprintf(stderr, "Unable to initialize the sbgCom library.\n");
				}
			}
			else
			{
				//
				// Unable to read the motion profile buffer
				//
				fprintf(stderr, "Unable to read the motion profile buffer.\n");
			}

			//
			// Release the allocated buffer
			//
			SBG_FREE_ARRAY(pMpBuffer);
		}
		else
		{
			//
			// Unable to allocate the motion profile buffer
			//
			fprintf(stderr, "Unable to allocate the motion profile buffer.\n");
		}


		//
		// Close the motion profile file
		//
		fclose(pMpFile);
	}
	else
	{
		//
		// Unable to open the motion profile file for reading
		//
		fprintf(stderr, "Unable to open the motion profile file for reading.\n");
	}

	return 0;
}
