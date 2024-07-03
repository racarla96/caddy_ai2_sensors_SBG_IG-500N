/*!
 *	\file		sbgCanDevice.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		09/06/10
 *
 *	\brief		Implementation of the IG devices can communication protocol.<br>
 *				You can access low-level communication with the device.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 *
 *	You will find below, the frame definition used by IG devices.<br>
 *
 *	The minimum data length for a can message is 0 byte and the maximum is 8 bytes.<br>
 *	<br>
 *	The CRC is calculated on the data field of the can frame as defined in the can protocol specification.
 *
 */

#ifndef __SBG_CAN_DEVICE_H__
#define __SBG_CAN_DEVICE_H__

#include "sbgCanBus.h"

//----------------------------------------------------------------------//
//- GPS status definitions and macros                                  -//
//----------------------------------------------------------------------//
#define SBG_CAN_GPS_NO_FIX							(0x00)					/*!< The GPS has no position solution. */
#define SBG_CAN_GPS_TIME_ONLY 						(0x01)					/*!< The GPS has only a time information. */
#define SBG_CAN_GPS_2D_FIX							(0x02)					/*!< The GPS has only a valid 2D position. */
#define SBG_CAN_GPS_3D_FIX							(0x03)					/*!< The GPS has a complete 3D position. */

#define SBG_CAN_GPS_VALID_TOW						(0x04)					/*!< Mask 1 = Valid Time of Week */
#define SBG_CAN_GPS_VALID_WKN						(0x08)					/*!< Mask 1 = Valid Week Number */
#define SBG_CAN_GPS_VALID_UTC						(0x10)					/*!< Mask 1 = Valid UTC (Leap Seconds already known) */

#define SBG_CAN_GPS_GET_FIX(flags)					(flags & 0x03)			/*!< Returns the fix status in the passed GPSFlags variable */
#define SBG_CAN_GPS_GET_VALID_TOW(flags)			((flags >>2) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid GPS Time Of Week */
#define SBG_CAN_GPS_GET_VALID_WKN(flags)			((flags >>3) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid GPS Week number */
#define SBG_CAN_GPS_GET_VALID_UTC(flags)			((flags >>4) & 0x01)	/*!< Returns 1 if the GPSFlags variable indicates a valid UTC Time (Leap Seconds already known) */

//----------------------------------------------------------------------//
//-  Definitions concerning space vehicles information GPS frame       -//
//----------------------------------------------------------------------//

#define SBG_CAN_SV_USED_FOR_NAV						(0x01)					/*!< SV is used for navigation */
#define SBG_CAN_SV_DIFF_AVAILABLE					(0x02)					/*!< Differential correction data is available for this SV */
#define SBG_CAN_SV_ORBIT_AVAILABLE					(0x04)					/*!< Orbit information is available for this SV (Ephemeris or Almanach) */
#define SBG_CAN_SV_ORBIT_EPHEMERIS					(0x08)					/*!< Orbit information is Ephemeris */
#define SBG_CAN_SV_UNHEALTHY						(0x10)					/*!< SV is unhealthy / shall not be used */

#define SBG_CAN_SV_QUAL_IDLE						(0x00)					/*!< This channel is idle */
#define SBG_CAN_SV_QUAL_SEARCHING_1					(0x01)					/*!< Channel is searching */
#define SBG_CAN_SV_QUAL_SERACHING_2					(0x02)					/*!< Channel is searching */
#define SBG_CAN_SV_QUAL_DETECTED_UNUSABLE			(0x03)					/*!< Signal detected but unusable */
#define SBG_CAN_SV_QUAL_CODE_LOCK_ON				(0x04)					/*!< Code Lock on Signal */
#define SBG_CAN_SV_QUAL_CODE_AND_CARRIER_LOCKED_1	(0x05)					/*!< Code and Carrier locked */
#define SBG_CAN_SV_QUAL_CODE_AND_CARRIER_LOCKED_2	(0x06)					/*!< Code and Carrier locked */
#define SBG_CAN_SV_QUAL_RECEIVING_DATA				(0x07)					/*!< Code and Carrier locked, receiving 50bps data */

//----------------------------------------------------------------------//
//- Device status Bitmask  definitions                                 -//
//----------------------------------------------------------------------//

//
// In the following bitmasks, a 1 signify normal operation, and a 0 means there is a warning
//
#define SBG_CAN_CALIB_INIT_STATUS_MASK				(0x00000001)				/*!< Bit mask for calibration loading result */
#define SBG_CAN_SETTINGS_INIT_STATUS_MASK			(0x00000002)				/*!< Bit mask for settings init result indication */

#define SBG_CAN_ACCEL_0_SELF_TEST_STATUS_MASK		(0x00000004)				/*!< Bit mask for accelerometer 0 self test result */
#define SBG_CAN_ACCEL_1_SELF_TEST_STATUS_MASK		(0x00000008)				/*!< Bit mask for accelerometer 1 self test result */
#define SBG_CAN_ACCEL_2_SELF_TEST_STATUS_MASK		(0x00000010)				/*!< Bit mask for accelerometer 2 self test result */
#define SBG_CAN_ACCEL_RANGE_STATUS_MASK				(0x00000020)				/*!< Bit mask for Accelerometer range status <br>
																					 - 1: in range operation; 
																					 - 0: over range */

#define SBG_CAN_GYRO_0_SELF_TEST_STATUS_MASK		(0x00000040)				/*!< Bit mask for gyroscope 0 self test result */
#define SBG_CAN_GYRO_1_SELF_TEST_STATUS_MASK		(0x00000080)				/*!< Bit mask for gyroscope 1 self test result */
#define SBG_CAN_GYRO_2_SELF_TEST_STATUS_MASK		(0x00000100)				/*!< Bit mask for gyroscope 2 self test result */
#define SBG_CAN_GYRO_RANGE_STATUS_MASK				(0x00000200)				/*!< Bit mask for Gyroscope range status <br>
																					 - 1: in range operation; 
																					 - 0: over range */

#define SBG_CAN_MAG_CALIBRATION_STATUS_MASK			(0x00000400)				/*!< Bit mask for magnetometers range status	<br>
																					 - 1: calibration seems to be OK;	<br>
																					 - 0: strong magnetic fields are applied, or magnetic field
																					    calibration has to be performed */

#define SBG_CAN_ALTI_INIT_STATUS_BIT_MASK			(0x00000800)				/*!< Bit mask for Altimeter initialization result */
#define SBG_CAN_CAN_GPS_STATUS_BIT_MASK				(0x00001000)				/*!< Bit mask for GPS initialization and communication status: <br>
																					- 1: Active communication with external device (GPS)
																					- 0: Bad GPS initialization / Communication lost */
																					
#define SBG_CAN_G_MEASUREMENT_VALID_MASK			(0x00002000)				/*!< Bit mask for proper gravity measurement status <br>
																					 - 1: valid gravity is measured over long time periods; 
																					 - 0: No gravity is observable over long time periods */
#define SBG_CAN_HEADING_MEASUREMENT_VALID_MASK		(0x00004000)				/*!< Bit mask for proper heading measurement status <br>
																					 - 1: valid heading is measured over long time periods; 
																					 - 0: No heading is observable over long time periods */
#define SBG_CAN_VEL_MEASUREMENT_VALID_MASK			(0x00008000)				/*!< Bit mask for proper velocity measurement status <br>
																					 - 1: valid heading is measured;
																					 - 0: No heading is observable */
#define SBG_CAN_POS_MEASUREMENT_VALID_MASK			(0x00010000)				/*!< Bit mask for proper position measurement status <br>
																					 - 1: valid heading is measured; 
																					 - 0: No heading is observable */
#define SBG_CAN_UTC_VALID_MASK						(0x00020000)				/*!< Bit mask for GPS UTC valid, leap seconds already known */
#define SBG_CAN_UTC_ROUGH_ACCURACY_MASK				(0x00040000)				/*!< Bit mask for UTC time with rough accuracy (about 250 ms)*/
#define SBG_CAN_UTC_SYNCHRONIZED_MASK				(0x00080000)				/*!< Bit mask for UTC time synchronized with the GPS  */

#define SBG_CAN_PROTOCOL_OUTPUT_STATUS				(0x00100000)				/*!< Bit mask for output buffer saturation indicator:<br>
																		  			 - 1: Ouptput buffer in normal operation
																		 			 - 0:  Output buffer is saturated */

//----------------------------------------------------------------------//
//- Avaiable CAN messages default identifiers list                     -//
//----------------------------------------------------------------------//

/*!
 *	Enum of the default CAN messages ids.
 */
typedef enum _SbgCanId
{
	SBG_CAN_ID_OUTPUT_TIMESTAMP_TRIGGER		= 0,						/*!< User CAN id for output and configuration frames.<br>
										 	 	 	 	 	 	 	 	 *   An id superior or equal to 0x20000000 indicates an invalid frame, the corresponding frame will be disabled. */
	SBG_CAN_ID_OUTPUT_DEVICE_STATUS,
	SBG_CAN_ID_OUTPUT_UTC_TIME,

	SBG_CAN_ID_OUTPUT_QUATERNION,
	SBG_CAN_ID_OUTPUT_EULER,
	SBG_CAN_ID_OUTPUT_HEADING,

	SBG_CAN_ID_OUTPUT_GYROSCOPES,
	SBG_CAN_ID_OUTPUT_ACCELEROMETERS,
	SBG_CAN_ID_OUTPUT_MAGNETOMETERS,
	SBG_CAN_ID_OUTPUT_TEMPERATURES,
	SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES,

	SBG_CAN_ID_OUTPUT_POSITION_1,
	SBG_CAN_ID_OUTPUT_POSITION_2,
	SBG_CAN_ID_OUTPUT_VELOCITY_1,
	SBG_CAN_ID_OUTPUT_VELOCITY_2,

	SBG_CAN_ID_OUTPUT_GYROSCOPES_RAW,
	SBG_CAN_ID_OUTPUT_ACCELEROMETERS_RAW,
	SBG_CAN_ID_OUTPUT_MAGNETOMETERS_RAW,
	SBG_CAN_ID_OUTPUT_TEMPERATURES_RAW,
	SBG_CAN_ID_OUTPUT_GYRO_TEMPERATURES_RAW,

	SBG_CAN_ID_OUTPUT_BAROMETER,
	SBG_CAN_ID_OUTPUT_MAG_CALIB_DATA,
	SBG_CAN_ID_OUTPUT_ODOMETER_VELOCITIES,

	SBG_CAN_ID_OUTPUT_GPS_INFO,
	SBG_CAN_ID_OUTPUT_GPS_SVINFO,
	SBG_CAN_ID_OUTPUT_GPS_POSITION_1,
	SBG_CAN_ID_OUTPUT_GPS_POSITION_2,
	SBG_CAN_ID_OUTPUT_GPS_VELOCITY_1,
	SBG_CAN_ID_OUTPUT_GPS_VELOCITY_2,
	SBG_CAN_ID_OUTPUT_GPS_COURSE,
	SBG_CAN_ID_OUTPUT_GPS_TRUE_HEADING,

	SBG_CAN_ID_OUTPUT_DELTA_ANGLES,
	SBG_CAN_ID_OUTPUT_HEAVE,
	SBG_CAN_ID_OUTPUT_RESERVED_2,
	SBG_CAN_ID_OUTPUT_RESERVED_3,
	SBG_CAN_ID_OUTPUT_RESERVED_4,
	SBG_CAN_ID_OUTPUT_RESERVED_5,
	SBG_CAN_ID_OUTPUT_RESERVED_6,
	SBG_CAN_ID_OUTPUT_RESERVED_7,
	SBG_CAN_ID_OUTPUT_RESERVED_8,
	SBG_CAN_ID_OUTPUT_RESERVED_9,
	SBG_CAN_ID_OUTPUT_RESERVED_10,
	SBG_CAN_ID_OUTPUT_RESERVED_11,
	SBG_CAN_ID_OUTPUT_RESERVED_12,
	SBG_CAN_ID_OUTPUT_RESERVED_13,
	SBG_CAN_ID_OUTPUT_RESERVED_14,
	SBG_CAN_ID_OUTPUT_RESERVED_15,

	SBG_CAN_ID_SAVE_SETTINGS,				/*!< ID = 47 */
	SBG_CAN_ID_RESTORE_SETTINGS,
	SBG_CAN_ID_LOW_POWER_MODE,
	SBG_CAN_ID_DEVICE_INFO,
	SBG_CAN_ID_USER_ID,
	SBG_CAN_ID_USER_BUFFER,
	SBG_CAN_ID_OUTPUT_TRIGGERS_CONF,
	SBG_CAN_ID_MAIN_LOOP_DIVIDER,
	SBG_CAN_ID_PROTOCOL_MODE,
	SBG_CAN_ID_FRAME_ID,
	SBG_CAN_ID_FILTER_FREQUENCIES,
	SBG_CAN_ID_KALMAN_FILTER,
	SBG_CAN_ID_COMMAND_DEPRECATED_0,		/*!< Deprecated as of firmware v 2.x */
	SBG_CAN_ID_FILTER_HEADING_SOURCE,
	SBG_CAN_ID_MAGNETIC_DECLINATION,
	SBG_CAN_ID_REFERENCE_PRESSURE,
	SBG_CAN_ID_GPS_OPTIONS,
	SBG_CAN_ID_NAV_SOURCES,
	SBG_CAN_ID_GPS_LEVER_ARM,
	SBG_CAN_ID_GRAVITY_MAGNITUDE,
	SBG_CAN_ID_COMMAND_DEPRECATED_1,		/*!< Deprecated as of firmware v 2.x */
	SBG_CAN_ID_AUTO_ORIENTATION_OFFSET,
	SBG_CAN_ID_PRE_ORIENTATION_OFFSET,
	SBG_CAN_ID_POST_ORIENTATION_OFFSET,
	SBG_CAN_ID_CALIB_MAG,
	SBG_CAN_ID_CALIB_MAG_MANUAL,
	SBG_CAN_ID_CALIB_GYRO_BIAS,
	SBG_CAN_ID_EXT_DEVICE,
	SBG_CAN_ID_EXT_DEVICE_CONF,
	SBG_CAN_ID_ODO_CONFIG,
	SBG_CAN_ID_ODO_DIRECTION,
	SBG_CAN_ID_ODO_LEVER_ARM,
	SBG_CAN_ID_LOGIC_IN_CHANNEL,
	SBG_CAN_ID_LOGIC_OUT_CHANNEL,
	SBG_CAN_ID_COMMAND_DEPRECATED_2,		/*!< Deprecated as of firmware v 2.x */

	SBG_CAN_ID_SEND_MP_BUFFER,				/*!< ID = 82 */
	SBG_CAN_ID_VALIDATE_MP_BUFFER,
	SBG_CAN_ID_MP_INFO,
	SBG_CAN_ID_HEAVE_CONF,
	SBG_CAN_ID_VIRTUAL_ODO_CONF,
	SBG_CAN_ID_ADVANCED_OPTIONS,
	SBG_CAN_ID_COMMAND_RESERVED_6,			/*!< Set of reserved settings for future commands */
	SBG_CAN_ID_COMMAND_RESERVED_7,
	SBG_CAN_ID_COMMAND_RESERVED_8,
	SBG_CAN_ID_COMMAND_RESERVED_9,
	SBG_CAN_ID_COMMAND_RESERVED_10,
	SBG_CAN_ID_COMMAND_RESERVED_11,
	SBG_CAN_ID_COMMAND_RESERVED_12,
	SBG_CAN_ID_COMMAND_RESERVED_13,
	SBG_CAN_ID_COMMAND_RESERVED_14,
	SBG_CAN_ID_COMMAND_RESERVED_15,

	SBG_CAN_ID_SEND_FILTER_HEADING,
	SBG_CAN_ID_SEND_NAV_VELOCITY_LOCAL_1,
	SBG_CAN_ID_SEND_NAV_VELOCITY_LOCAL_2,
	SBG_CAN_ID_SEND_NAV_VELOCITY_NED_1,
	SBG_CAN_ID_SEND_NAV_VELOCITY_NED_2,
	SBG_CAN_ID_SEND_NAV_POSITION_1,
	SBG_CAN_ID_SEND_NAV_POSITION_2,

	SBG_CAN_ID_SEND_RESERVED_0,				/*!< Set of reserved settings for future input frames */
	SBG_CAN_ID_SEND_RESERVED_1,
	SBG_CAN_ID_SEND_RESERVED_2,
	SBG_CAN_ID_SEND_RESERVED_3,
	SBG_CAN_ID_SEND_RESERVED_4,
	SBG_CAN_ID_SEND_RESERVED_5,
	SBG_CAN_ID_SEND_RESERVED_6,
	SBG_CAN_ID_SEND_RESERVED_7,

	SBG_CAN_ID_NUM
} SbgCanId;

/*!
 * This enum allows various error codes to be transmitted by certain CAN commands such as settings saving or restoring default settings
 */
typedef enum _SbgCanAck
{
	SBG_CAN_OPERATION_SUCCESS			= 0x00,		/*!< Operation could be properly performed */
	SBG_CAN_OPERATION_FAILURE			= 0x01		/*!< Operation could not be peformed correctly */
} SbgCanAck;

//----------------------------------------------------------------------//
//- Advanced can message definitions                                   -//
//----------------------------------------------------------------------//
#define SBG_CAN_MAX_STD_VALID_ID	(0x000007FF)													/*!< Max CAN message id available for standard (CAN 2.0A) version. */
#define SBG_CAN_MAX_EXT_VALID_ID	(0x1FFFFFFF)													/*!< Max CAN message id available for extended (CAN 2.0B) version. */
#define SBG_CAN_DISABLED_FRAME		(0xF0000000)													/*!< Invalid CAN frame id, generally used for disabled messages. */

#define SBG_CAN_ID_FIRST			(SBG_CAN_ID_OUTPUT_TIMESTAMP_TRIGGER)							/*!< First CAN message id stored in the settings. */
#define SBG_CAN_ID_LAST				(SBG_CAN_ID_SEND_RESERVED_7)									/*!< Last CAN message id stored in the settings. */

#define SBG_CAN_OUTPUT_ID_FIRST		(SBG_CAN_ID_OUTPUT_TIMESTAMP_TRIGGER)							/*!< First CAN output id stored in the settings. */
#define SBG_CAN_OUTPUT_ID_LAST		(SBG_CAN_ID_OUTPUT_RESERVED_15)									/*!< Last CAN output id stored in the settings. */

#define SBG_CAN_COMMAND_ID_FIRST	(SBG_CAN_ID_SAVE_SETTINGS)										/*!< First CAN command is stored in the settigs. */
#define SBG_CAN_COMMAND_ID_LAST		(SBG_CAN_ID_COMMAND_RESERVED_15)								/*!< Last CAN command id stored in the settings. */

#define SBG_CAN_SEND_ID_FIRST		(SBG_CAN_ID_SEND_FILTER_HEADING)								/*!< First CAN send aiding message id stored in the settings. */
#define SBG_CAN_SEND_ID_LAST		(SBG_CAN_ID_SEND_RESERVED_7)									/*!< Last CAN send aiding message id stored in the settings. */

#define SBG_CAN_ID_COUNT			(SBG_CAN_ID_LAST - SBG_CAN_ID_FIRST+1)							/*!< Number of CAN messages id stored in the settings. */
#define SBG_CAN_OUTPUT_ID_COUNT		(SBG_CAN_ID_OUTPUT_ID_LAST - SBG_CAN_ID_OUTPUT_ID_FIRST+1)		/*!< Number of CAN output id stored in the settings. */
#define SBG_CAN_COMMAND_ID_COUNT	(SBG_CAN_ID_COMMAND_ID_LAST - SBG_CAN_ID_COMMAND_ID_FIRST+1)	/*!< Number of CAN command id stored in the settings. */
#define SBG_CAN_SEND_ID_COUNT		(SBG_CAN_ID_SEND_ID_LAST - SBG_CAN_ID_SEND_ID_FIRST+1)			/*!< Number of CAN send id stored in the settings. */

//----------------------------------------------------------------------//
//- Outputs data definitions							               -//
//----------------------------------------------------------------------//

/*!
 *	Structure to hold both the time since reset and a trigger mask
 */
typedef struct _SbgCanTimestampTrigger
{
	uint32						timeSinceReset;				/*!< Time since reset in ms. */
	uint16						triggerMask;				/*!< Output trigger status mask. */	
} SbgCanTimestampTrigger;

/*!
 *	Structure to hold the utc Time
 */
typedef struct _SbgCanUtcTime
{
	uint8						utcTimeYear;				/*!< Year after  2000. */
	uint8						utcTimeMonth;				/*!< Month of year. */
	uint8						utcTimeDay;					/*!< Day of month. */
	uint8						utcTimeHours;				/*!< Hours of day. */
	uint8						utcTimeMinute;				/*!< Minutes of hour. */
	uint8						utcTimeSecond;				/*!< Seconds of minutes. */
	uint32						utcTimeNanoSecond;			/*!< Nano-seconds. */
} SbgCanUtcTime;

/*!
 *	Structure to hold the horizontal positions
 */
typedef struct _SbgCanPosition1
{
	int32						latitude;					/*!< Latitude in WGS84 in degress (1e-7)*/
	int32						longitude;					/*!< Longitude in WGS84 in degress (1e-7)*/
} SbgCanPosition1;

/*!
 *	Structure to hold the vertical position and position accuracy
 */
typedef struct _SbgCanPosition2
{
	int32						altitude;					/*!< Altitude in mm */
	uint16						horAccuracy;				/*!< Horizontal position accuracy in cm */
	uint16						vertAccuracy;				/*!< Vertical position accuracy in cm */
} SbgCanPosition2;

/*!
 *	Structure to hold the horizontal velocities
 */
typedef struct _SbgCanVelocity1
{
	int32						Vx;							/*!< Velocity for x axis in cm/s*/
	int32						Vy;							/*!< Velocity for y axis in cm/s*/
} SbgCanVelocity1;

/*!
 *	Structure to hold the vertical velocity and velocity accuracy
 */
typedef struct _SbgCanVelocity2
{
	int32						Vz;							/*!< Velocity for z axis in cm/s*/
	uint16						Vacc;						/*!< Velocity accuracy in cm/s */
} SbgCanVelocity2;

/*!
 *	Structure to hold the barometer informations
 */
typedef struct _SbgCanBarometer
{
	uint32						pressure;					/*!< Pressure measured by the pressure sensor in Pa */
	int32						altitude;					/*!< Altitude calculated using pressure information in cm*/
} SbgCanBarometer;

/*!
 *	Structure to hold the GPS Informations
 */
typedef struct _SbgCanGpsInfo
{
	uint32						gpsTime;					/*!< GPS time of the week in ms */
	uint8						gpsFlag;					/*!< GPS Fix information */
	uint8						gpsSatNum;					/*!< Number of satellites used by GPS for navigation solution */
} SbgCanGpsInfo;

/*!
 *	Structure to hold the GPS advanced Informations
 */
typedef struct _SbgCanGpsSvInfo
{
	uint8						gpsChannel;					/*!< Channel number of the GPS receiver. */
	uint8						gpsId;						/*!< Satellite id. */
	uint8						gpsQualityFlag;				/*!< Flags Quality. */
	uint8						sigStrength;				/*!< Signal Strength in dbHz. */
	int8						azimuth;					/*!< Azimuth in 32/45 degrees. */
	int8						elevation;					/*!< Elevation in 32/45 degrees. */
} SbgCanGpsSvInfo;

/*!
 *	Structure that hold a heading and with it's accuracy
 */
typedef struct _SbgCanHeading
{
	int32						heading;					/*!< Heading in degrees (1e-5)*/
	uint32						headingAcc;					/*!< Heading accuracy in degrees (1e-5)*/
} SbgCanHeading;

/*!
 *	Union to hold the output data
 */
typedef union _SbgCanOutputDataUnion
{
	
	SbgCanTimestampTrigger			timestampTrigger;		/*!< Time since reset and trigger mask structure. */
	uint32							deviceStatusMask;		/*!< Device status bits mask. */
	SbgCanUtcTime					utcTime;				/*!< Utc time structure. */
	float							quaternion[4];			/*!< Orientation quaternion. */
	float							eulerAngle[3];			/*!< Orientation in Euler angles in rad. */
	SbgCanHeading					eulerYaw;				/*!< Kalman enhanced heading struct that can be used as an external aiding information for an other IG device (contains the same data as eulerAngle[2]). */
	float							gyroscopes[3];			/*!< Calibrated gyroscopes values in rad/s. */
	float							deltaAngle[3];			/*!< Delta angle values in rad/s. (Result of 1kHz coning integration) */
	float							accelerometers[3];		/*!< Calibrated accelerometers values in m/s. */
	float							magnetometers[3];		/*!< Calibrated magnetometers values in a.u (arbritary unit). */
	float							temperatures[2];		/*!< Temperatures measured by ADC and accelerometer/magnetometer sensors in °C. */
	float							gyroTemperatures[3];	/*!< Temperatures measured by gyroscopes sensors in °C. */
	SbgCanPosition1					position1;				/*!< Horizontal positions structure. */
	SbgCanPosition2					position2;				/*!< Vertical position and position accuracy structure. */
	int32							heave;					/*!< Heave output in mm */
	SbgCanVelocity1					velocity1;				/*!< Horizontal velocities structure. */
	SbgCanVelocity2					velocity2;				/*!< Vertical velocity and velocity accuracy structure. */
	uint16							rawGyroValues[3];		/*!< Raw gyroscopes values. */
	uint16							rawAcceleroValues[3];	/*!< Raw accelerometers values. */
	uint16							rawMagnetoValues[3];	/*!< Raw magnetometers values. */
	uint16							rawTemperatures[2];		/*!< Raw temperatures measured by ADC and accelerometer/magnetometer sensors. */
	uint16							rawGyroTemperatures[3];	/*!< Raw temperatures measured by gyroscopes sensors. */

	SbgCanBarometer					barometer;				/*!< Barometer informations structure. */
	uint8							magCalibData[6];		/*!< Data used to calibrate hard and soft Iron effects. */
	int32							odoRawVelocity[2];		/*!< Odometer two channels velocities in mm/s (IG-500E only). */

	SbgCanGpsInfo					gpsInfo;				/*!< GPS Informations structure. */
	SbgCanGpsSvInfo					gpsSvInfo;				/*!< GPS advanced Informations structure. */
	SbgCanPosition1					gpsRawPosition1;		/*!< Horizontal positions structure. */
	SbgCanPosition2					gpsRawPosition2;		/*!< Vertical position and position accuracy structure. */
	SbgCanVelocity1					gpsRawVelocity1;		/*!< Horizontal velocities structure. */
	SbgCanVelocity2					gpsRawVelocity2;		/*!< Vertical velocity and velocity accuracy structure. */
	SbgCanHeading					gpsRawCourse;			/*!< GPS course structure (the trajectory direction). */

	SbgCanHeading					gpsTrueHeading;			/*!< GPS true heading computed using a two GPS antenna (IG-500E only). */
} SbgCanOutputDataUnion;

/*!
 *	Structure where to find the output data
 */
typedef struct _SbgCanOutputDataStr
{
	SbgCanId						outputId;				/*!< Default output id */
	SbgCanOutputDataUnion			outputData;				/*!< Output union containing all outputs data */
	
} SbgCanOutputDataStr;

//----------------------------------------------------------------------//
//- Device communication definitions				                   -//
//----------------------------------------------------------------------//

#define SBG_CAN_INVALID_DEVICE_HANDLE			(NULL)			/*!< Identify an invalid device handle. */

/*!
 * Structure used to store for each CAN message, its format and identifier.
 */
typedef struct _SbgCanMessageId
{
	SbgCanMessageFormat	messageFormat;						/*!< CAN message format SBG_CAN_ID_STANDARD or SBG_CAN_ID_EXTENDED. */
	uint32				messageId;							/*!< CAN message identifier. */
} SbgCanMessageId;

/*!
 *	List of can id used by a device for each frames.
 */
typedef	SbgCanMessageId	SbgCanFramesIdList[SBG_CAN_ID_COUNT];

/*!
 *	Bus handle associated to the device
 */
typedef  void	*SbgCanBusHandlePtr;

/*!
 *	Struct containing all device related data.
 */
typedef struct _SbgCanDeviceHandleStr
{
	SbgCanFramesIdList				framesIdList;			/*!< Set of can identifiers to use for the device. */
	SbgCanBusHandle					canBusHandle;			/*!< The bus handle of the sbgCan library associated to the device. */

	void (*pUserHandlerDeviceError)(struct _SbgCanDeviceHandleStr *pDeviceHandler, SbgErrorCode errorCode, void *pUsrArg);			/*!< Function pointer that should be called when we have an error on a continuous operation */
	void (*pUserHandlerDeviceOutput)(struct _SbgCanDeviceHandleStr *pDeviceHandler, SbgCanOutputDataStr *pOutput, void *pUsrArg);	/*!< Function pointer that should be called when we receive a new continous frame */
	
	void *pUserArgDeviceError;								/*!< User defined data passed to the continuous error callback function. */
	void *pUserArgDeviceOutput;								/*!< User defined data passed to the continuous callback function */

	struct _SbgCanDeviceHandleStr	*pNext;					/*!< Pointer to the next device in the list. */
	struct _SbgCanDeviceHandleStr	*pPrev;					/*!< Pointer to the previous device in the list. */

} SbgCanDeviceHandleStr;

/*!
 * 	Handle type used by the device system.
 */
typedef SbgCanDeviceHandleStr *SbgCanDeviceHandle;

/*!
 *	Function pointer definition for continuous error callback.<br>
 *	This callback is called each time we have an error on a continuous frame.
 *	\param[in]	pDeviceHandler			The associated device handle.
 *	\param[in]	errorCode				Error code that occured on a continuous mode operation.
 *	\param[in]	pUsrArg					Pointer to the user defined argument.
 */
typedef void (*DeviceErrorCallback)(SbgCanDeviceHandleStr *pDeviceHandler, SbgErrorCode errorCode, void *pUsrArg);

/*!
 *	Function pointer definition for continuous callback.<br>
 *	This callback is called each time we read a new valid continuous frame.
 *	\param[in]	pDeviceHandler			The associated device handle.
 *	\param[in]	pDeviceMsg				Pointer to the can message handle by the device.
 *	\param[in]	pUsrArg					Pointer to the user defined argument.
 */
typedef void (*DeviceOutputCallback)(SbgCanDeviceHandleStr *pDeviceHandler, const SbgCanOutputDataStr *pOutput, void *pUsrArg);

//----------------------------------------------------------------------//
//- Device communication operations									   -//
//----------------------------------------------------------------------//

/*!
 *	Add a device to the sbgCan library handle.
 *	\param[out]	pDeviceHandle			Pointer to the new device handle.
 *	\param[in]	busHandle				A valid sbgCan library handle.
 *	\param[in]	userIdsList				The list of user IDs used for the device. <br>
 *										If set to NULL pointer, the default id list is used.
 *	\return								SBG_NO_ERROR if the device has been added properly.
 */
SbgErrorCode sbgCanAddDevice(SbgCanDeviceHandle *pDeviceHandle, SbgCanBusHandle busHandle, const SbgCanFramesIdList userIdsList);

/*!
 *	Remove a device of the sbgCan library handle.
 *	\param[in]	pDeviceHandle			Pointer to the device handle to remove.
 *	\return								SBG_NO_ERROR if the device has been removed properly.
 */
SbgErrorCode sbgCanRemoveDevice(SbgCanDeviceHandle *pDeviceHandle);

/*!
 *	Defines the handle function to call when we have an error on a continuous frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	callback				Pointer to the error handler function. (could be NULL to remove the callback)
 *	\param[in]	pUserArg				User argument to pass to the error handler function.
 *	\return								SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgCanSetDeviceErrorCallback(SbgCanDeviceHandle deviceHandle, DeviceErrorCallback callback, void *pUserArg);

/*!
 *	Defines the handle function to call when we have received a valid continuous frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	callback				Pointer to the error handler function. (could be NULL to remove the callback)
 *	\param[in]	pUserArg				User argument to pass to the error handler function.
 *	\return								SBG_NO_ERROR if the callback function has been defined. 
 */
SbgErrorCode sbgCanSetDeviceOutputCallback(SbgCanDeviceHandle deviceHandle, DeviceOutputCallback callback, void *pUserArg);

/*!
 *	Send a specific CAN message.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				Generique id used to identify which CAN message to send.
 *	\param[in]	length					CAN message data field length.
 *	\param[in]	pData					Data contained in the CAN message.
 *	\return								SBG_NO_ERROR if we were able to send the CAN message.
 */
SbgErrorCode sbgCanDeviceSendSpecificMessage(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, uint8 length, const uint8 *pData);

/*!
 *	Receive a specific CAN message.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				Generique id used to identify which CAN message we should receive.
 *	\param[in]	timeOut					Time in ms to wait before leaving receive mode.
 *	\param[out]	pLength					Received CAN message length.
 *	\param[out]	pData					Receive CAN message data.
 *	\return								SBG_NO_ERROR if 
 */
SbgErrorCode sbgCanDeviceReceiveSpecificMessage(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, uint16 timeOut, uint8 *pLength, uint8 pData[8]);

//----------------------------------------------------------------------//
//- Internal Device operations										   -//
//----------------------------------------------------------------------//

/*!
 *	For a specific frame, get its user id and frame format.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				The default frame id to get the user id and frame format.
 *	\param[out]	pFormat					The CAN message format (SBG_SBG_CAN_ID_STANDARD or SBG_SBG_CAN_ID_EXTENDED).
 *	\param[out]	pUserId					The user id of the frame.
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanDeviceGetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, SbgCanMessageFormat *pFormat, uint32 *pUserId);

/*!
 *	Set a frame format and user id for a specific frame.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	defaultId				The default id of frame for which to set the id.
 *	\param[in]	format					CAN message format (SBG_SBG_CAN_ID_STANDARD or SBG_SBG_CAN_ID_EXTENDED).
 *	\param[in]	userId					The new user id of the frame.			
 *	\return								SBG_NO_ERROR there is no error in the transmission.
 */
SbgErrorCode sbgCanDeviceSetFrameId(SbgCanDeviceHandle deviceHandle, SbgCanId defaultId, SbgCanMessageFormat format, uint32 userId);

/*!
 *	Treat a received continuous frame and call the user device output callback if the received frame is valid.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	pMsg					Pointer to the received can message
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanDeviceManageContinuousFrame(SbgCanDeviceHandle deviceHandle, const SbgCanMessageRaw *pMsg);

/*!
 *	Treat the output according to their default id and convert the data if needed.
 *	\param[in]	deviceHandle			A valid device handle.
 *	\param[in]	pMsg					Pointer to the device output message.
 *	\param[out]	pOutput					Pointer to a structure that hold the output data and id
 *	\return								SBG_NO_ERROR if we have received a valid frame.
 */
SbgErrorCode sbgCanDeviceHandleOutputs(SbgCanDeviceHandle deviceHandle, const SbgCanMessageRaw *pMsg, SbgCanOutputDataStr *pOutput);

#endif	// __SBG_CAN_DEVICE_H__