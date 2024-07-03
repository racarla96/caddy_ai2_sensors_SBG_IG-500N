#include "binaryLogEvent.h"
#include "../misc/sbgStreamBuffer.h"

//----------------------------------------------------------------------//
//- Operations                                                         -//
//----------------------------------------------------------------------//

/*!
 *	Parse data for the SBG_ECOM_LOG_EVENT_# message and fill the corresponding structure.
 *	\param[in]	pPayload					Read only pointer on the payload buffer.
 *	\param[in]	payloadSize					Payload size in bytes.
 *	\param[out]	pOutputData					Pointer on the output structure that stores parsed data.
 *	\return									SBG_NO_ERROR if the payload has been parsed.
 */
SbgErrorCode sbgEComBinaryLogParseEvent(const void *pPayload, uint32 payloadSize, SbgLogEvent *pOutputData)
{
	SbgStreamBuffer inputStream;

	//
	// Create an input stream to read the payload
	//
	sbgStreamBufferInitForRead(&inputStream, pPayload, payloadSize);

	//
	// Read the frame payload
	//
	pOutputData->timeStamp		= sbgStreamBufferReadUint32(&inputStream);
	pOutputData->status			= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->timeOffset0	= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->timeOffset1	= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->timeOffset2	= sbgStreamBufferReadUint16(&inputStream);
	pOutputData->timeOffset3	= sbgStreamBufferReadUint16(&inputStream);
	
	//
	// TODO: check for an error on the input stream such as buffer overflow
	//
	return SBG_NO_ERROR;
}
