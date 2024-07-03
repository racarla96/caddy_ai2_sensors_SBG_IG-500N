/*!
 *	\file		sbgCanCommandsExt.h
 *  \author		SBG-Systems (Raphaël Siryani)
 *	\date		08/12/10
 *
 *	\brief		Commands used to define Extrnal device parameters on IG-500E.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */
#ifndef __SBG_CAN_COMMANDS_EXT_H__
#define __SBG_CAN_COMMANDS_EXT_H__

#include "../sbgCommon.h"
#include "sbgCanDevice.h"

//----------------------------------------------------------------------//
//- External devices Types definition                                  -//   
//----------------------------------------------------------------------//

/*!
 * This enum defines all handeled external devices
 */
typedef enum _SbgCanExtDeviceType
{
	SBG_CAN_EXT_NONE			=	0x00,						/*!< No external device is attached */
	SBG_CAN_EXT_IG_DEVICE		=	0x03,						/*!< SBG Systems IG device */
	SBG_CAN_EXT_NMEA			=	0x04						/*!< Other device which uses standard NMEA protocol */
} SbgCanExtDeviceType;

//----------------------------------------------------------------------//
//- External devices definitions                                       -//
//----------------------------------------------------------------------//

#define SBG_CAN_EXT_PORT_RS232					(0x0000)		/*!< Set the external port in RS-232 mode (default) */
#define SBG_CAN_EXT_PORT_RS422					(0x0001)		/*!< Set the external port in RS-422 mode */
#define SBG_CAN_EXT_PORT_FAST_SLEW				(0x0000)		/*!< Fast slew rate, baudrate is not limited. (default) */
#define SBG_CAN_EXT_PORT_SLOW_SLEW				(0x0002)		/*!< Slow slew rate for EMI reduction. Baudrate is limited to 230 400 bps. */

//----------------------------------------------------------------------//
//- External device commands                                           -//
//----------------------------------------------------------------------//

/*!
 *	Select the external device connected to the IG-500E and its secondary UART configuration
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	deviceType						The device type connected
 *  \param[in]	baudRate						The baud rate used to communicate with the device at initialization
 *  \param[in]	uartOptions						Some uart Options needed to communicate with the device
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanSetExtDevice(SbgCanDeviceHandle deviceHandle, SbgCanExtDeviceType deviceType, uint32 baudRate, uint16 uartOptions);

/*!
 *	Get the external device connected to the IG-500E and its secondary UART configuration
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[out]	pDeviceType						The device type connected
 *  \param[out]	pBaudRate						The baud rate used to communicate with the device at initialization
 *  \param[out]	pUartOptions					Some uart Options used to communicate with the device
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanGetExtDevice(SbgCanDeviceHandle deviceHandle, SbgCanExtDeviceType *pDeviceType, uint32 *pBaudRate, uint16 *pUartOptions);

/*!
 *	Send to the external device a specific configuration.<br>
 *	\param[in]	deviceHandle					A valid ig can device handle.
 *  \param[in]	pCommand						Command sent to the external device.
 *  \param[in]	commandSize						Size of the command to send.
 *  \param[out]	pAnswer							Answer of the external device.
 *  \param[out]	pAnswerSize						Size of the device answer.
 *  \param[in]	answerMaxSize					Maximum allowed size of the answer.
 *	\return										SBG_NO_ERROR if no error.
 */
SbgErrorCode sbgCanExtDeviceConfig(SbgCanDeviceHandle deviceHandle, const uint8 *pCommand, uint16 commandSize, uint8 *pAnswer, uint16 *pAnswerSize, uint16 answerMaxSize);

#endif
