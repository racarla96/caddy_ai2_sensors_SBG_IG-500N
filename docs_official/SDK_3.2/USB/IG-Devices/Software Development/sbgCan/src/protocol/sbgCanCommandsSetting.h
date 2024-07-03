/*!
 *	\file		sbgCanCommandsSetting.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		10/06/10
 *
 *	\brief		Generic commands implementation.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_H__
#define __SBG_CAN_COMMANDS_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"


//----------------------------------------------------------------------//
//- GPS receiver power modes                                           -//
//----------------------------------------------------------------------//

/*!
 *	Defines the device power mode options
 */
typedef enum _SbgCanPowerModeDevice
{
	SBG_CAN_DEVICE_MAX_PERF			= 0x00,										/*!< Max. performance mode. Allows reduced latency as well as heave computation */
	SBG_CAN_DEVICE_NORMAL			= 0x02,										/*!< Normal mode; Limited power consumption but we cannot get heave computation on this mode */
} SbgCanPowerModeDevice;

/*!
 *	Define the GPS power mode options
 */
typedef enum _SbgCanPowerModeGps
{
	SBG_CAN_GPS_MAX_PERF			= 0x00,										/*!< Max. performance mode. */
	SBG_CAN_GPS_ECO_MODE_1			= 0x01,										/*!< Eco mode 1: Current peaks are limited;<br>
																				 *   startup might be longer than in MAX_PERF mode. <br>
																				 *   (Supported on GPS hardware V3 and above) */
	SBG_CAN_GPS_ECO_MODE_2			= 0x02,										/*!< Eco mode 2: Current peaks are limited;<br>
																				 *   In addition, once sufficient satellites are visible, 
																				 *   the GPS enters into a low power mode and stops new satellites tracking<br>
																				 *   (Supported on GPS hardware V3 and above) */
	SBG_CAN_GPS_OFF_MODE			= 0x05,										/*!< GPS receiver is turned off. */
} SbgCanPowerModeGps;


//----------------------------------------------------------------------//
//- IG devices advanced options used by SBG_SET_ADVANCED_OPTIONS       -//
//----------------------------------------------------------------------//

#define SBG_CAN_SETTING_ENABLE_CONING						(0x00000001)			/*!< Use coning integration in the kalman filter instead of gyroscopes values */
#define SBG_CAN_SETTING_ALTITUDE_ABOVE_MSL					(0x00000004)			/*!< -0 -> Altitude is above Ellipsoid
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 			 				 *   -1 -> Altitude is above Mean Sea Level */
#define SBG_CAN_SETTING_DECLINATION_AUTO					(0x00000008)			/*!< Enable/Disable automatic magnetic declination computation */
#define SBG_CAN_SETTING_GRAVITY_AUTO						(0x00000010)			/*!< Enable/Disable automatic local gravity computation */
#define SBG_CAN_SETTING_OUTPUT_UNBIASED_GYRO				(0x00000020)			/*!< Enable/Disable kalman unbiased gyroscope and delta angle outputs */
#define SBG_CAN_SETTING_OUTPUT_UNBIASED_ACCEL				(0x00000040)			/*!< Enable/Disable kalman unbiased accelerometer output */
#define SBG_CAN_SETTING_FORCE_MAG_HORIZONTAL				(0x00000080)			/*!< Force magnetometers use to horizontal position whatever the calibration is */
#define SBG_CAN_SETTING_STATIC_INIT							(0x00000100)			/*!< Initialize the kalman filter with a no motion assumption for 10 seconds for faster startup. */
#define SBG_CAN_SETTING_STATIC_INIT_UNTIL_MOTION_DETECTED	(0x00000200)			/*!< Initialize the kalman filter with a no motion assumption until a motion is detected for faster startup. */

//----------------------------------------------------------------------//
//- Settings commands operations                                       -//
//----------------------------------------------------------------------//

/*!
 *	Saves the current device settings into its flash memory and then return the result.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\return										SBG_NO_ERROR in case of a good operation.
 */
SbgErrorCode sbgCanSaveSettings(SbgCanDeviceHandle deviceHandle);

/*!
 *	Restores all settings to factory defaults and save them to FLASH memory.										<br>
 *	Calibration data such as gyroscopes bias and magnetometers calibration are not affected by this command!		<br>
 *	The device bit rate is reseted to 1000 Mbit/s.																	<br>
 *	The device send the result of this command afterward with the default device bit rate.							<br>
 *	This command change the bit rate on the device and the library.													<br>
 *	To change only the bit rate used by the library, you have to use the function sbgCanBusChangeBitRate.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanRestoreSettings(SbgCanDeviceHandle deviceHandle);

/*!
 *	Gets device information such as product code, hardware and firmware revisions.																	<br>
 *	For versions, use SBG_VERSION_GET_MAJOR, SBG_VERSION_GET_MINOR, SBG_VERSION_GET_REV and SBG_VERSION_GET_BUILD to extract versions inforamtion.	<br>
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	productCode						Device product code string.
 *	\param[out]	pDeviceId						The intern device id.
 *	\param[out]	pFirmwareVersion				The device firmware version. ('1.0.0.0')
 *	\param[out]	pCalibDataVersion				The device calibration data version. ('1.0.0.0')
 *	\param[out]	pMainBoardVersion				The device main board hardware version. ('1.0.0.0')
 *	\param[out]	pGpsBoardVersion				The device gps/top board hardware version. ('1.0.0.0')
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetInfo(SbgCanDeviceHandle deviceHandle, char productCode[32], uint32 *pDeviceId, uint32 *pFirmwareVersion, uint32 *pCalibDataVersion, uint32 *pMainBoardVersion, uint32 *pGpsBoardVersion);

/*!
 *	Defines a user selectable id for the device.
 *	\param[in]	deviceHandle					A valid ig can device handle.
  *	\param[in]	userId							The new device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetUserId(SbgCanDeviceHandle deviceHandle, uint32 userId);

/*!
 *	Returns the device user id.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pUserId							Pointer used to hold the device user id.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetUserId(SbgCanDeviceHandle deviceHandle, uint32 *pUserId);

/*!
 *	For a specific CAN message, defines a custom id or disable the message using the specific id SBG_CAN_DISABLED_FRAME.<br>
 *	For example, to define SBG_CAN_OUTPUT_EULER CAN message id to 0x0CD3 use the following syntax:<br>
 *	sbgCanSetFrameId(deviceHandle, SBG_CAN_OUTPUT_EULER, 0x00000CD3);
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultMsgId					Id of the CAN message we would like to define (use an id from SbgCanId enum.)
 *	\param[in]	newMsgFormat					New CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[in]	newMsgId						The new CAN message id or SBG_CAN_DISABLED_FRAME to disable this message.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultMsgId, SbgCanMessageFormat newMsgFormat, uint32 newMsgId);

/*!
 *	For a specific CAN message, returns the used id.<br>
 *	If SBG_CAN_DISABLED_FRAME is returned, it means that this CAN message is disabled.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	defaultMsgId					Id of the CAN message we would like to read its id (use an id from SbgCanId enum.)
  *	\param[out]	pNewMsgFormat					Used CAN message format (SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED).
 *	\param[out]	pNewMsgId						Used CAN message id or SBG_CAN_DISABLED_FRAME if this message is disabled.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultMsgId, SbgCanMessageFormat *pNewMsgFormat, uint32 *pNewMsgId);

/*!
 *	Sets the Low power modes for the IG-Device
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	devicePowerMode					Defines the device power mode then return it.
 *	\param[in]	gpsPowerMode					Defines the the GPS receiver power mode then return it. (leave to SBG_CAN_GPS_MAX_PERF if there is no GPS reveicer)	
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetLowPowerMode(SbgCanDeviceHandle deviceHandle, SbgCanPowerModeDevice devicePowerMode, SbgCanPowerModeGps gpsPowerMode);

/*!
 *	Gets the Low power modes for the IG-Device
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pDevicePowerMode				Returns the device power mode. (pass NULL if not used).
 *	\param[out]	pGpsPowerMode					Returns the GPS receiver power mode. (pass NULL if not used).	<br>
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetLowPowerMode(SbgCanDeviceHandle deviceHandle, SbgCanPowerModeDevice *pDevicePowerMode, SbgCanPowerModeGps *pGpsPowerMode);

/*!
 *	Write to the IG device's user buffer memory a unit32 at a specific index.
 *	The user buffer is an array of 160 uint32 that can be written or read.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	index							Index in the user buffer array to write data to.
 *	\param[in]	data							Data to write to IG device's memory at the specific index.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetUserBuffer(SbgCanDeviceHandle deviceHandle, uint8 index, uint32 data);

/*!
 *	Read from the IG device's user buffer memory a unit32 at a specific index.
 *	The user buffer is an array of 160 uint32 that can be written or read.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	index							Index in the user buffer array to read data from.
 *	\param[in]	pData							Read data from the IG device's memory at the specific index.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetUserBuffer(SbgCanDeviceHandle deviceHandle, uint8 index, uint32 *pData);

/*!
 *	Defines the bit rate of the device CAN communication.<br>
 *	If the command is valid, the device acknoledge the bit rate change at the current speed and THEN change it's bit rate.		<br>
 *	This command change the bit rate on the device and the library.																<br>
 *	To change only the bit rate used by the library, you have to use the function sbgCanBusChangeBitRate.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	bitRate							The new device bit rate to use in kBit/s.										<br>
 *												Possible values are:
 *												<ul>
 *													<li>1 000 kBit/s</li>
 *													<li>  500 kBit/s</li>
 *													<li>  250 kBit/s</li>
 *													<li>  125 kBit/s</li>
 *													<li>  100 kBit/s</li>
 *													<li>   50 kBit/s</li>
 *													<li>   20 kBit/s</li>
 *													<li>   10 kBit/s</li>
 *												</ul>
 *	\return										If SBG_NO_ERROR, the device has been configured to the new speed.
 */
SbgErrorCode sbgCanSetProtocolMode(SbgCanDeviceHandle deviceHandle, uint16 bitRate);

/*!
 *	Command used to get the current bitrate used by the device.
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pBitRate						The current device bit rate in in kBit/s.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetProtocolMode(SbgCanDeviceHandle deviceHandle, uint16 *pBitRate);

/*!
 *	Defines the device advanced settings
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[in]	settings						Advanced settings bitmask
 *	\return										If SBG_NO_ERROR, if no error.
 */
SbgErrorCode sbgCanSetAdvancedSettings(SbgCanDeviceHandle deviceHandle, uint32 settings);

/*!
 *	Get the device advanced settings
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *	\param[out]	pSettings						Advanced settings bitmask
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetAdvancedSettings(SbgCanDeviceHandle deviceHandle, uint32 *pSettings);


#endif