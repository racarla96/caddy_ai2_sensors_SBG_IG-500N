/*!
 *	\file		commands.h
 *  \author		SBG Systems (Raphael Siryani)
 *	\date		20 March 2012
 *
 *	\brief		This file implements Ekinox commands.
 *
 *	\section CodeCopyright Copyright Notice 
 *	Copyright (C) 2007-2013, SBG Systems SAS. All rights reserved.
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
#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "../sbgECom.h"

//----------------------------------------------------------------------//
//- Defintions                                                         -//
//----------------------------------------------------------------------//

#define SBG_ECOM_DEFAULT_CMD_TIME_OUT	(500)			/*!< Default time out in ms for commands reception. */

//----------------------------------------------------------------------//
//- Helpers commands operations                                        -//
//----------------------------------------------------------------------//

/*!
 *	Wait for an ACK for a specified amount of time.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	command						The command we would like the ACK for.
 *	\param[in]	timeOut						Time out in ms during which we can receive the ACK.
 *	\return									SBG_NO_ERROR if the ACK has been received.
 */
SbgErrorCode sbgEComWaitForAck(SbgEComHandle *pHandle, uint16 command, uint32 timeOut);

//----------------------------------------------------------------------//
//- Generic commands                                                   -//
//----------------------------------------------------------------------//

/*!
 *	Save the current settings configuration into FLASH memory.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComSaveSettings(SbgEComHandle *pHandle);

/*!
 *	Send a complete set of settings to the device and store them into the FLASH memory.
 *	The device will reboot automatically to use the new settings.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Read only buffer containing the settings.
 *	\param[in]	size						Size of the buffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComImportSettings(SbgEComHandle *pHandle, const void *pBuffer, uint32 size);

/*!
 *	Retrieve a complete set of settings from the device as a buffer.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	pBuffer						Allocated buffer that can hold the received settings.
 *	\param[out]	pSize						The number of bytes that have been stored into pBuffer.
 *	\param[in]	maxSize						The maximum buffer size in bytes that can be stored into pBuffer.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComExportSettings(SbgEComHandle *pHandle, void *pBuffer, uint32 *pSize, uint32 maxSize);

//----------------------------------------------------------------------//
//- Magnetometers commands                                             -//
//----------------------------------------------------------------------//

/*!
 *	Send a command that set the magnetometers calibration parameters.
 *	\param[in]	pHandle						A valid sbgECom handle.
 *	\param[in]	offset						Magnetometers calibration offset vector.
 *	\param[in]	matix						Magnetometers calibration 3x3 matrix.
 *	\return									SBG_NO_ERROR if the command has been executed successfully.
 */
SbgErrorCode sbgEComSetMagCalibData(SbgEComHandle *pHandle, const float offset[3], const float matrix[9]);

#endif
