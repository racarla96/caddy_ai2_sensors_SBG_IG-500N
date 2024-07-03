% Returns an error message based on the error code
function errorMsg = buildErrorMsg(this, errorCode)
	% Errors codes definitions
	SBG_NO_ERROR							= uint32(0);    % No error
	SBG_ERROR								= uint32(1);	% We have a generic error
	SBG_NULL_POINTER						= uint32(2);	% A pointer is null
	SBG_INVALID_CRC							= uint32(3);	% Our received frame has an invalid CRC
	SBG_INVALID_FRAME						= uint32(4);	% Our received frame is invalid
	SBG_TIME_OUT							= uint32(5);	% We have started to receive a frame but not the end
	SBG_WRITE_ERROR							= uint32(6);	% All bytes hasn't been written
	SBG_READ_ERROR							= uint32(7);	% All bytes hasn't been read
	SBG_BUFFER_OVERFLOW						= uint32(8);	% A buffer is too small to contain so much data
	SBG_INVALID_PARAMETER					= uint32(9);	% An invalid parameter has been founded
	SBG_NOT_READY							= uint32(10);	% A device isn't ready (Rx isn't ready for example)
	SBG_MALLOC_FAILED						= uint32(11);	% Failed to allocate a buffer
	SGB_CALIB_MAG_NOT_ENOUGH_POINTS			= uint32(12);	% Not enough points were available to perform magnetometers calibration
	SBG_CALIB_MAG_INVALID_TAKE				= uint32(13);	% The calibration procedure could not be properly executed due to insufficient precision
	SBG_CALIB_MAG_SATURATION				= uint32(14);	% Saturation were detected when attempt to calibrate magnetos
	SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE		= uint32(15);	% 2D calibration procedure could not be performed 

	SBG_DEVICE_NOT_FOUND					= uint32(16);	% A device couldn't be founded or opened
	SBG_OPERATION_CANCELLED					= uint32(17);	% An operation was cancelled
	SBG_NOT_CONTINUOUS_FRAME				= uint32(18);	% We have received a frame that isn't a continuous one
		
	% Build our error message according to the error code
	switch errorCode
		case SBG_NO_ERROR
			errorMsg = 'SBG_NO_ERROR';
		case SBG_ERROR
			errorMsg = 'SBG_ERROR';
		case SBG_NULL_POINTER
			errorMsg = 'SBG_NULL_POINTER';
		case SBG_INVALID_CRC
			errorMsg = 'SBG_INVALID_CRC';
		case SBG_INVALID_FRAME
			errorMsg = 'SBG_INVALID_FRAME';
		case SBG_TIME_OUT
			errorMsg = 'SBG_TIME_OUT';
		case SBG_WRITE_ERROR
			errorMsg = 'SBG_WRITE_ERROR';
		case SBG_READ_ERROR
			errorMsg = 'SBG_READ_ERROR';
		case SBG_BUFFER_OVERFLOW
			errorMsg = 'SBG_BUFFER_OVERFLOW';
		case SBG_INVALID_PARAMETER
			errorMsg = 'SBG_INVALID_PARAMETER';
		case SBG_NOT_READY
			errorMsg = 'SBG_NOT_READY';
		case  SBG_MALLOC_FAILED
			errorMsg = 'SBG_MALLOC_FAILED';
		case SGB_CALIB_MAG_NOT_ENOUGH_POINTS
			errorMsg = 'SGB_CALIB_MAG_NOT_ENOUGH_POINTS';
		case SBG_CALIB_MAG_INVALID_TAKE
			errorMsg = 'SBG_CALIB_MAG_INVALID_TAKE';
		case SBG_CALIB_MAG_SATURATION
			errorMsg = 'SBG_CALIB_MAG_SATURATION';
		case SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE
			errorMsg = 'SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE';
		case SBG_DEVICE_NOT_FOUND
			errorMsg = 'SBG_DEVICE_NOT_FOUND';
		case SBG_OPERATION_CANCELLED
			errorMsg = 'SBG_OPERATION_CANCELLED';
		case SBG_NOT_CONTINUOUS_FRAME
			errorMsg = 'SBG_NOT_CONTINUOUS_FRAME';
		otherwise
			errorMsg = 'SBG_ERROR';
	end % End switch case	
end % End buildErrorMsg  
        
