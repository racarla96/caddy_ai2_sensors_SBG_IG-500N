/*!
 *	\file		sbgIronCalibration.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		01/02/10
 *
 *	\brief		Main header file for the magnetometers iron calibration library.
 *
 *	Main header file for the magnetometers iron calibration library.<br>
 *	In order to use the iron calibration library, please include this file in your project.<br>
 *
 *	\section CodeCopyright Copyright Notice
 *	Copyright (C) 2007-2011, SBG Systems SAS. All rights reserved.
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
#ifndef __SBG_IRON_CALIBRATION_H__
#define __SBG_IRON_CALIBRATION_H__

#include "sbgCommon.h"

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------//
//- DLL related export/import definitions                              -//
//----------------------------------------------------------------------//

/*!
 *	In order to use the library as a DLL one, please define SBG_IRON_CALIBRATION_DLL.
 *	If not, the library will be used as a static one.
 */
#ifdef SBG_IRON_CALIBRATION_DLL
	#ifdef SBG_IRON_CALIBRATION_DLL_EXPORT
		#define SBG_IRON_CALIBRATION_API __declspec(dllexport)
	#else
		#define SBG_IRON_CALIBRATION_API __declspec(dllimport)
	#endif
#else
	#define SBG_IRON_CALIBRATION_API
#endif

//----------------------------------------------------------------------//
//- Structs definitions                                                -//
//----------------------------------------------------------------------//

/*!
 *	This struct is used to returns the calibration results and quality indicators.
 */
typedef struct _SbgIronCalibResults
{
	float	m_crossAxisMatrix[9];					/*!< 3x3 magnetometers cross axis matrix. */
	float	m_offsetVector[3];						/*!< X,Y,Z magnetometers offset vector. */
	
	float	m_ellipsoidMatrix[9];					/*!< 3x3 matrix used to transform a shpere into the computed ellipsoid. */
	float	m_ellipsoidOffset[3];					/*!< X,Y,Z offset of the computed ellipsoid matrix. */
	
	float	m_beforeAvgDeviation;					/*!< Average magnetometers deviation before calibration. */
	float	m_beforeMaxDeviation;					/*!< Maximum magnetometers deviation before calibration. */
	
	float	m_afterAvgDeviation;					/*!< Average magnetometers deviation after calibration. */
	float	m_afterMaxDeviation;					/*!< Maximum magnetometers deviation after calibration. */
	
	float	m_avgExpectedAccuracy;					/*!< Average expected heading accuracy in degrees. */
	float	m_maxExpectedAccuracy;					/*!< Maximum expected heading accuracy in degrees. */
} SbgIronCalibResults;

/*!
 *	Defines the main iron calibration library handle.
 */
typedef struct _SbgIronCalibHandleInt* SbgIronCalibHandle;

/*!
 *	Enum used to define the calibration mode (2D or 3D).
 */
typedef enum
{
	SBG_CALIB_MODE_2D_HORIZONTAL,					/*!< 2D horizontal mode to use only when the device is leveled and rotated arround the Z axis. */
	SBG_CALIB_MODE_2D,								/*!< 2D mode used only if all magnetometers data are in a plane. */
	SBG_CALIB_MODE_3D								/*!< 3D mode default mode that fits almost all applications. */
} SbgIronCalibMode;

//----------------------------------------------------------------------//
//- Initialisation                                                     -//
//----------------------------------------------------------------------//

/*!
 *	Create and initialise the iron calibration library.
 *	\param[in]	licenceKey				The provided licence key in the form (XXXXX-XXXXX-XXXXX-XXXXX-XXXXX).
 *	\param[out]	pHandle					Pointer on a SbgIronCalibHandle variable that can hold the created handle.
 *	\return								SBG_NO_ERROR if the library has been initialised.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibInit(const char licenceKey[30], SbgIronCalibHandle *pHandle);

/*!
 *	Destroy the iron calibration library.
 *	\param[in]	handle					Our iron calibration library handle.
 *	\return								SBG_NO_ERROR if a valid iron calibration library handle has been destroyed.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibClose(SbgIronCalibHandle handle);

/*!
 *	Returns an integer representing the version of the iron calibration library.
 *	\return										An integer representing the version of the iron calibration library.<br>
 *												Use #SBG_VERSION_GET_MAJOR, #SBG_VERSION_GET_MINOR, #SBG_VERSION_GET_REV and #SBG_VERSION_GET_BUILD.
 */
uint32 SBG_IRON_CALIBRATION_API sbgIronCalibGetVersion(void);

/*!
 *	Returns the iron calibration library version as a string (1.0.0.0).
 *	\return										Null terminated string that hold the iron calibration library version.
 */
const char SBG_IRON_CALIBRATION_API *sbgIronCalibGetVersionAsString(void);

/*!
 *	Returns if the iron calibration library is using float or double for internal computations.
 *	\return										TRUE if the iron calibration library is using double or FALSE otherwise.
 */
bool SBG_IRON_CALIBRATION_API sbgIronCalibGetIsUsingDouble(void);

/*!
 *	Convert an error code into a human readable string.
 *	\param[in]	errorCode						Our errorCode to convert into a string.
 *	\param[out]	errorMsg						String buffer used to hold the error string.
 */
void SBG_IRON_CALIBRATION_API sbgIronCalibErrorToString(SbgErrorCode errorCode, char errorMsg[256]);

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Add a set of points to the list of magnetometers calibration data using the standard 12 bytes format.
 *	Each points is stored in a 12 bytes buffer as returned by the serial devices.
 *	The sbgCenter also produce .mag files that contains the same type of data.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	pDataList						Our list of magnetomters calibration data to add.
 *	\param[in]	numData							The number of elements we have in the magnetomters calibration data list.
 *	\return										SBG_NO_ERROR if we have sucessfully added the points.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibAddPointsList(SbgIronCalibHandle handle, uint8 *pDataList[12], uint32 numData);
	
/*!
 *	Add a new point to the list of magnetometers calibration datausing the standard 12 bytes format.
 *	Each points is stored in a 12 bytes buffer as returned by the serial devices.
 *	The sbgCenter also produce .mag files that contains the same type of data.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	magnetometerData					Our magnetomter data to add.
 *	\return										SBG_NO_ERROR if we have sucessfully added the point.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibAddPoint(SbgIronCalibHandle handle, uint8 magnetometerData[12]);

/*!
 *	Add a set of points to the list of magnetometers calibration data using the special 6 bytes CAN format.
 *	Due to CAN frame specifications, the CAN devices are using a custom 6 bytes format to store magnetometers calibration.
 *	Warning: The sbgCenter always produce .mag files that contains the standard 12 bytes format.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	pDataList						Our list of magnetomters calibration data to add.
 *	\param[in]	numData							The number of elements we have in the magnetomters calibration data list.
 *	\return										SBG_NO_ERROR if we have sucessfully added the points.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibAddPointsListCAN(SbgIronCalibHandle handle, uint8 *pDataList[6], uint32 numData);
	
/*!
 *	Add a set of points to the list of magnetometers calibration data using the special 6 bytes CAN format.
 *	Due to CAN frame specifications, the CAN devices are using a custom 6 bytes format to store magnetometers calibration.
 *	Warning: The sbgCenter always produce .mag files that contains the standard 12 bytes format.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	magnetometerData					Our magnetomter data to add.
 *	\return										SBG_NO_ERROR if we have sucessfully added the point.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibAddPointCAN(SbgIronCalibHandle handle, uint8 magnetometerData[6]);

/*!
 *	Remove all magnetometers calibration data that have been added.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\return										SBG_NO_ERROR if we have successfully released the magnetometers calibration data.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibReleasePoints(SbgIronCalibHandle handle);

/*!
 *	Returns how many magnetometers calibration data we have.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\return										The number of magnetometers calibration data.
 */
uint32 SBG_IRON_CALIBRATION_API sbgIronCalibGetNumData(SbgIronCalibHandle handle);

/*!
 *	Given an index in the calibration data list, returns the corresponding calibrated magnetometer value.<br>
 *	sbgIronCalibCompute should have been called before you can use this function.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	pCalibrationResults				Pointer to a filled structure SbgIronCalibResults (tipically returned by sbgIronCalibCompute).
 *	\param[in]	index							Index of the magnetometers calibration data.
 *	\param[out]	calibratedMag					Our X,Y,Z magnetometer calibrated value.
 *	\return										SBG_NO_ERROR if we are able to compute the calibrated magnetometer data.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibGetCalibratedData(SbgIronCalibHandle handle, const SbgIronCalibResults *pCalibrationResults, uint32 index, float calibratedMag[3]);

/*!
 *	Computes the calibration data according to the calibration mode.
 *	\param[in]	handle							Our iron calibration library handle.
 *	\param[in]	mode							Defines if we would like to perform a 2D or 3D calibration.
 *	\param[out]	pCalibrationResults				Pointer to a valid structure SbgIronCalibResults that can hold the calibration results.
 *	\return										SBG_NO_ERROR if the calibration has been sucessfully computed.
 */
SbgErrorCode SBG_IRON_CALIBRATION_API sbgIronCalibCompute(SbgIronCalibHandle handle, SbgIronCalibMode mode, SbgIronCalibResults *pCalibrationResults);

/*!
 *	Initialize a iron calibration results struct to default values.
 *	\param[in]	pCalibrationResults				Pointer to a SbgIronCalibResults structure to initialize to zero.
 */
void SBG_IRON_CALIBRATION_API sbgIronCalibInitResultsToZero(SbgIronCalibResults *pCalibrationResults);
 
#ifdef __cplusplus
}
#endif

#endif	// __SBG_IRON_CALIBRATION_H__

