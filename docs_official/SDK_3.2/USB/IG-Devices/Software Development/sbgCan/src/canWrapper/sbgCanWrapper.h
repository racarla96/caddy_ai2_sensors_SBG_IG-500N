/*!
 *	\file		sbgCanWrapper.h
 *  \author		SBG-Systems (Olivier Ripaux)
 *	\date		09/06/10
 *
 *	\brief		Wrapper declaration for low-level can commuication functions.<br>
 *				You can provide your own implementation adapated to your platform.<br>
 *				Copyright 2007-2012 SBG Systems. All rights reserved.
 */

#ifndef __SBG_CAN_WRAPPER_H__
#define __SBG_CAN_WRAPPER_H__

#include "../sbgCommon.h"

//------------------------------------------------------------------------------//
//- SBG can device definition												   -//
//------------------------------------------------------------------------------//

#define SBG_CAN_INVALID_HANDLE			(NULL)	/*!< Identify an invalid interface handle. */

//------------------------------------------------------------------------------//
//- SBG CAN status that can returned by sbgCanInterfaceGetBusStatus			   -//
//------------------------------------------------------------------------------//
#define SBG_CAN_ERROR_OK				(0x00000)		/*!< No error returned. */
#define SBG_CAN_ERROR_XMTFULL			(0x00001)		/*!< Transmit buffer in CAN controller is full. */
#define SBG_CAN_ERROR_OVERRUN			(0x00002)		/*!< CAN controller was read too late. */
#define SBG_CAN_ERROR_BUSLIGHT			(0x00004)		/*!< Bus error: an error counter reached the 'light' limit. */
#define SBG_CAN_ERROR_BUSHEAVY			(0x00008)		/*!< Bus error: an error counter reached the 'heavy' limit. */
#define SBG_CAN_ERROR_BUSOFF			(0x00010)		/*!< Bus error: the CAN controller is in bus-off state. */
#define SBG_CAN_ERROR_QRCVEMPTY			(0x00020)		/*!< Receive queue is empty. */
#define SBG_CAN_ERROR_QOVERRUN			(0x00040)		/*!< Receive queue was read too late. */
#define SBG_CAN_ERROR_QXMTFULL			(0x00080)		/*!< Transmit queue is full. */
#define SBG_CAN_ERROR_REGTEST			(0x00100)		/*!< Test of the CAN controller hardware registers failed (no hardware found). */
#define SBG_CAN_ERROR_NODRIVER			(0x00200)		/*!< Driver not loaded. */
#define SBG_CAN_ERROR_HWINUSE			(0x00400)		/*!< Hardware already in use by a Net. */
#define SBG_CAN_ERROR_NETINUSE			(0x00800)		/*!< A Client is already connected to the Net. */
#define SBG_CAN_ERROR_ILLHW				(0x01400)		/*!< Hardware handle is invalid. */
#define SBG_CAN_ERROR_ILLNET			(0x01800)		/*!< Net handle is invalid. */
#define SBG_CAN_ERROR_ILLCLIENT			(0x01C00)		/*!< Client handle is invalid. */
#define SBG_CAN_ERROR_RESOURCE			(0x02000)		/*!< Resource (FIFO, Client, timeout) cannot be created. */
#define SBG_CAN_ERROR_ILLPARAMTYPE		(0x04000)		/*!< Invalid parameter. */
#define SBG_CAN_ERROR_ILLPARAMVAL		(0x08000)		/*!< Invalid parameter value. */
#define SBG_CAN_ERROR_UNKNOWN			(0x10000)		/*!< Unknow error. */
#define SBG_CAN_ERROR_INITIALIZE		(0x40000)		/*!< Channel is not initialized. */


/*!
 *	Can id type definition.
 */
typedef enum _SbgCanMessageFormat
{
	SBG_CAN_ID_STANDARD = 0,						/*!< Message is a CAN Standard Frame (11-bit identifier). */
	SBG_CAN_ID_EXTENDED = 1							/*!< Message is a CAN Extended Frame (29-bit identifier). */
} SbgCanMessageFormat;

/*!
 *	Struct containing all interface related data.
 */
typedef struct _SbgCanInterfaceHandleStr
{
	void*				hardwareHandle;			/*!< The PC interface handle different on each os. */
	uint32				bitRate;				/*!< Bit Rate used by the PC interface. */

	uint32				lowestId;				/*!< The lowest id that the PC interface can receive. */
	uint32				highestId;				/*!< The highest id that the PC interface can receive. */
} SbgCanInterfaceHandleStr;

/*!
 *	Handle type used by the PC interface.
 */
typedef SbgCanInterfaceHandleStr *SbgCanInterfaceHandle;

/*!
 *	Struct containing can message related data.
 */
typedef struct _SbgCanMessageRaw
{
	SbgCanMessageFormat	format;					/*!< Defines the CAN message format (standard or extended). */
	uint32				id;						/*!< 11/29-bit message identifier. */
	uint8				length;					/*!< Data Length of the message (0...8). */
	uint8				data[8];				/*!< Data of the message (data[0]...data[7]). */
} SbgCanMessageRaw;

//------------------------------------------------------------------------------//
//  SBG Interface operations												    //
//------------------------------------------------------------------------------//

/*!
 *	Open a PC usb interface at a specific location.
 *	\param[out]	pInterfaceHandle				Pointer to the interface handle.
 *	\param[in]	pLocation						The access to the interface to be initialized (from 0 to n).
 *	\param[in]	bitRate							The desired bit rate on the CAN bus.<br>
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
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceOpen(SbgCanInterfaceHandle *pInterfaceHandle, const char *pLocation, uint32 bitRate);

/*!
 *	Close a PC usb interface.
 *	\param[in]	pInterfaceHandle				Pointer to the handle of the interface to be close.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceClose(SbgCanInterfaceHandle *pInterfaceHandle);

/*!
 *	Flush the interface rx and tx buffers (remove all old data).
 *	\param[in]	interfaceHandle					Handle of the interface to flush.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceFlush(SbgCanInterfaceHandle interfaceHandle);

/*!
 *	Change the bit rate of the opened interface.
 *	\param[in]	interfaceHandle					Handle of the interface.
 *	\param[in]	bitRate							The new desired bit rate.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceChangeBitRate(SbgCanInterfaceHandle interfaceHandle, uint32 bitRate);

/*!
 *	Get the status of the CAN bus.
 *	\param[in]	interfaceHandle					Handle of the interface.
 *	\return										A CAN status mask that represents all errors or 0 if no error.
 */
uint32 sbgCanInterfaceGetBusStatus(SbgCanInterfaceHandle interfaceHandle);

/*!
 *	Set a mask to filter received frames.
 *	\param[in]	interfaceHandle					Handle of the interface to read.
 *	\param[in]	lowestId						The lowest id to be received.
 *	\param[in]	highestId						The highest id to be received.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceSetReceiveFilter(SbgCanInterfaceHandle interfaceHandle, uint32 lowestId, uint32 highestId);

/*!
 *	Transmit a frame over the CAN bus.
 *	\param[in]	interfaceHandle					Handle of the interface to transmit.
 *	\param[in]	pMsg							Pointer to the can message to be sent.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceWriteFrame(SbgCanInterfaceHandle interfaceHandle, const SbgCanMessageRaw *pMsg);

/*!
 *	Read a frame from the CAN bus.
 *	\param[in]	interfaceHandle					Handle of the interface to read.
 *	\param[out]	pMsg							Pointer to the can message to be read.
 *	\return										SBG_NO_ERROR in case of a successful operation.
 */
SbgErrorCode sbgCanInterfaceReadFrame(SbgCanInterfaceHandle interfaceHandle, SbgCanMessageRaw *pMsg);

#endif	// __CANWRAPPER_H__