#include <stdio.h>
#include "sbgIronCalibration.h"
#include <math.h>

int main (int argc, const char * argv[])
{
	SbgIronCalibHandle calibLibHandle;
	SbgIronCalibResults calibResults;
	SbgErrorCode errorCode;
	char errorMsg[256];
	FILE *pInputFile;
	uint32 fileSize;
	uint32 numPointsToRead;
	uint32 i;
	uint8 data[12];
	float calibratedMag[3];
	
    //
	// Create our iron calibration library (put your own licence number)
	//
	//errorCode = sbgIronCalibInit("XXXXX-XXXXX-XXXXX-XXXXX-XXXXX", &calibLibHandle);
	errorCode = sbgIronCalibInit("CH7CN-VIASU-PVZ8Q-RZRTV-9VSCA", &calibLibHandle);
	
	//
	// Check if our iron calibration library has been initialised
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Display our iron calibration version
		//
		printf("Welcome to the SBG Systems iron calibration library version: %s\n", sbgIronCalibGetVersionAsString());
		
		//
		// Open a .mag file and add all points
		//
		pInputFile = fopen("examples/testMag2d.mag", "rb");
		
		//
		// Check if our file has been opened
		//
		if (pInputFile)
		{
			//
			// Goto the end of the file and get the file size
			//
			fseek(pInputFile, 0, SEEK_END);
			fileSize = ftell(pInputFile);
			fseek(pInputFile, 0, SEEK_SET);
			
			//
			// Check if our file size is valid (contains a multiple of 12 bytes)
			//
			if (fileSize%12 == 0)
			{
				//
				// Compute how many points we have to read in our file
				//
				numPointsToRead = fileSize/12;
				
				//
				// Read each point
				//
				for (i=0; i<numPointsToRead; i++)
				{
					//
					// Read our encrypted buffer
					//
					if (fread(data, sizeof(uint8), 12, pInputFile) == 12)
					{
						//
						// Add our magnetometer point
						//
						sbgIronCalibAddPoint(calibLibHandle, data);
					}
				}
				
				//
				// Compute our 3d calibration
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
					// Display each transformed point
					//
					for (i=0; i<sbgIronCalibGetNumData(calibLibHandle); i++)
					{
						sbgIronCalibGetCalibratedData(calibLibHandle, &calibResults, i, calibratedMag);
						printf("Mag:\t%0.2f\t%0.2f\t%0.2f\t%0.2f\n", calibratedMag[0], calibratedMag[1], calibratedMag[2], (float)sqrtf(calibratedMag[0]*calibratedMag[0]+calibratedMag[1]*calibratedMag[1]+calibratedMag[2]*calibratedMag[2]));
					}
				}
				else
				{
					//
					// Build our error message and display it
					//
					sbgIronCalibErrorToString(errorCode, errorMsg);
					fprintf(stderr,"Unable to calibrate our magnetomters with error: %s\n", errorMsg);
				}
			}
			else
			{
				//
				// Invalid file size
				//
				fprintf(stderr,"Invalid file size %u.\n", fileSize);
			}
			
			//
			// Close our file
			//
			fclose(pInputFile);
		}
		else
		{
			fprintf(stderr, "Unable to open our magnetometers file test.mag.\n");
		}
		
		//
		// Destroy our iron calibration
		//
		errorCode = sbgIronCalibClose(calibLibHandle);
	}
	else
	{
		//
		// Build our error message and display it
		//
		sbgIronCalibErrorToString(errorCode, errorMsg);
		fprintf(stderr, "sbgIronCalibInit failed with: %s.\n", errorMsg);
	}
	
	return 0;
}
