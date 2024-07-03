classdef CSbgMatLab < handle
	properties
        haveToUnloadLib = [];
        sbgComHandle = [];
		sbgComInitialised = [];
        output = [];
        pOutput = [];
    end
    
	methods
        % Class constructor
        function this = CSbgMatLab()
            % Check if we already have a loaded library
            if libisloaded('sbgMatLab')
                 % the library has been already loaded
                this.haveToUnloadLib = false;
            else
                % Load the library and mark the library to be unloaded
                loadlibrary('sbgMatLab', @sbgMatLabHeader);
                this.haveToUnloadLib = true;
            end

            % Create the sbgComHandle pointer
            this.sbgComHandle = libpointer('voidPtr');
 
            % Create the struct model
            this.output = libstruct('s_SbgMatLabData');
            
            % Create a reference on the struct
            this.pOutput = libpointer('s_SbgMatLabData', get(this.output));
						
			% At startupt, the sbgCom lib hasn't been initialised
			this.sbgComInitialised = false;
        end

        % Class destructor
        function delete(this) 
            % Close the sbgCom system only if needed
			if (this.sbgComInitialised == true)
	            close(this);
			end
            
            % Release the handle structure
            clear('this.sbgComHandle');
            clear('this.pOutput');
            clear('this.output');
            clear('this.sbgComInitialised');
			
            % Set all vars to NULL to avoid Matlab messup
            this.sbgComHandle = [];
            this.pOutput = [];
            this.output = [];
			this.sbgComInitialised = [];
            
            % Check if we have to unload the library
            if (this.haveToUnloadLib == true)
                unloadlibrary('sbgMatLab');
            end
        end 
        
        % Open the serial port and initialise the library
        % Example: obj.init('COM1', 115200)
        function init(this, location, speed)
            % Check if we have already initialised the system
            errorCode = calllib('sbgMatLab', 'sbgMatLabInit', location, speed, this.sbgComHandle);
			
			% Check if we have an error
            if (errorCode == 0)
				% No error, the sbgCom has been initialised
				this.sbgComInitialised = true;
			else
                error('CSbgMatLab:init', 'sbgMatLabInit failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
        end
        
        % Close the serial port
        % Example: obj.close()
        function close(this)
            errorCode = calllib('sbgMatLab', 'sbgMatLabClose', this.sbgComHandle);
			
			% Check if we have an error
            if (errorCode == 0)
				% the sbgCom has been closed so mark it
				this.sbgComInitialised = false;
			else
                error('CSbgMatLab:close', 'sbgMatLabClose failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get both the sbgMatLab dll version and sbgCom library version.
        % Example: [sbgMatLabVersion, sbgComVersion] = obj.getVersion();
        function [sbgMatLabVersion, sbgComVersion] = getVersion(this)
			% Init the outputs variables
			sbgMatLabVersion = blanks(32);
			sbgComVersion = blanks(32);
			
			% Create pointers for references args transmission
            pSbgMatLabVersion = libpointer('stringPtr', sbgMatLabVersion);
			pSbgComVersion = libpointer('stringPtr', sbgComVersion);
			
            % Call the dll function
            calllib('sbgMatLab', 'sbgMatLabGetVersion', pSbgMatLabVersion, pSbgComVersion);
			
			% Copy the outputs args
			sbgMatLabVersion = pSbgMatLabVersion.Value;
			sbgComVersion = pSbgComVersion.Value;
		end
		
		%----------------------------------------------------------------------
		%- Data acquisition operations                                             -
		%----------------------------------------------------------------------
		
		% Get the data received using the continous mode
		% For each asked data, returns a column vector based list
		% Example: [num, angles, accels] = obj.getData('SBG_OUTPUT_EULER', 'SBG_OUTPUT_ACCELEROMETERS');
        function [numReceived, varargout] = getData(this, varargin)
            % First init out numReceived to 0
            numReceived = 0;
			
			% Create an empty string of 512 chars used to hold the trigger mask and create a pointer to it
			triggerMaskStr = blanks(512);
			pTriggerMaskStr = libpointer('stringPtr', triggerMaskStr);
            
           % Get the number of optional input arguments
            nInputArgs = nargin-1;
            nOutputArgs = max(nargout,1)-1;
            
            % Pre-fill varargout with 0
            for n=1:nOutputArgs
                varargout{n} = 0;
			end
            
            % Check if we have the same number of argin and argout if not,
            % we have an error
            if nInputArgs == nOutputArgs
                % Retreive the data and get the number of received data
                numNewReceivedData = uint32(0);
                pNewReceivedData = libpointer('uint32Ptr', numNewReceivedData);
                errorCode = calllib('sbgMatLab', 'sbgMatLabHandleData', this.sbgComHandle, pNewReceivedData);
 
                % Check if we have no error
                if (errorCode == 0)
                    % Define the number of received data
                    numReceived = pNewReceivedData.Value;
                    
                    % Check if we have received some data
                    if (numReceived > 0)
                        % Preallocating the outputs
                        for n=1:nInputArgs
                            switch varargin{n}
							case 'SBG_OUTPUT_TRIGGER_MASK'
                                varargout{n} = cell(1, numReceived);
                            case 'SBG_OUTPUT_QUATERNION'
                                varargout{n} = zeros(4, numReceived);
                            case 'SBG_OUTPUT_EULER'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_MATRIX'
                                varargout{n} = zeros(3, 3, numReceived);
                            case 'SBG_OUTPUT_GYROSCOPES'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_ACCELEROMETERS'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_MAGNETOMETERS'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_TEMPERATURES'
                                varargout{n} = zeros(2, numReceived);
                                
                            case 'SBG_OUTPUT_GYROSCOPES_RAW'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_ACCELEROMETERS_RAW'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_MAGNETOMETERS_RAW'
                                varargout{n} = zeros(3,numReceived);
                            case 'SBG_OUTPUT_TEMPERATURES_RAW'
                                varargout{n} = zeros(2, numReceived);

                            case 'SBG_OUTPUT_TIME_SINCE_RESET'
                                varargout{n} = zeros(1, numReceived);
                            case 'SBG_OUTPUT_DEVICE_STATUS'
                                varargout{n} = zeros(1, numReceived);

                            case 'SBG_OUTPUT_GPS_POSITION'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_GPS_NAVIGATION'
                                varargout{n} = zeros(2, numReceived);
                            case 'SBG_OUTPUT_GPS_ACCURACY'
                                varargout{n} = zeros(4, numReceived);
                            case 'SBG_OUTPUT_GPS_INFO'
                                varargout{n} = zeros(3, numReceived);
								
							case 'SBG_OUTPUT_GPS_TRUE_HEADING'
								varargout{n} = zeros(2, numReceived);
								
                            case 'SBG_OUTPUT_BARO_ALTITUDE'
                                varargout{n} = zeros(1, numReceived);
                            case 'SBG_OUTPUT_BARO_PRESSURE'
                                varargout{n} = zeros(1, numReceived);

                            case 'SBG_OUTPUT_ATTITUDE_ACCURACY'
                                varargout{n} = zeros(1, numReceived);
                            case 'SBG_OUTPUT_NAV_ACCURACY'
                                varargout{n} = zeros(2, numReceived);
								
							case 'SBG_OUTPUT_GYRO_TEMPERATURES'
                                varargout{n} = zeros(3, numReceived);
                            case 'SBG_OUTPUT_GYRO_TEMPERATURES_RAW'
                                varargout{n} = zeros(3, numReceived);
								
                            case 'SBG_OUTPUT_UTC_TIME_REFERENCE'
                                varargout{n} = zeros(7, numReceived);
                            case 'SBG_OUTPUT_MAG_CALIB_DATA'
                                varargout{n} = zeros(3, numReceived);
								
							case 'SBG_OUTPUT_ODO_VELOCITIES'
                                varargout{n} = zeros(2, numReceived);
								
							case 'SBG_OUTPUT_DELTA_ANGLES'
                                varargout{n} = zeros(3, numReceived);
								
							case 'SBG_OUTPUT_HEAVE'
                                varargout{n} = zeros(1, numReceived);
                            end
                        end % End loop for each input args, pre-allocate
						
						
                        
                        % For each received data, build the output
                        for dataNum=1:numReceived
                            % Get the data
                            errorCode = calllib('sbgMatLab', 'sbgMatLabGetData', this.sbgComHandle, dataNum-1, this.pOutput, pTriggerMaskStr);
                            
                            % Check if we have no error
                            if (errorCode == 0)
								
                                % Get the data values
                                values = get(this.pOutput, 'Value');
								
                                % For each input argument, returns the data in
                                % column vector formats
                                for n=1:nInputArgs
                                    switch varargin{n}
									case 'SBG_OUTPUT_TRIGGER_MASK'										
										% Convert the int8 array to a string and set the output
										varargout{n}(:, dataNum) = {pTriggerMaskStr.Value};
                                    case 'SBG_OUTPUT_QUATERNION'
                                        varargout{n}(:, dataNum) = [values.q0; values.q1; values.q2;  values.q3];
                                    case 'SBG_OUTPUT_EULER'
                                        varargout{n}(:, dataNum) = [values.roll; values.pitch; values.yaw];
                                    case 'SBG_OUTPUT_MATRIX'
                                        varargout{n}(:,:, dataNum) = [	values.ma0, values.ma3, values.ma6;
                                                                        values.ma1, values.ma4, values.ma7;
                                                                        values.ma2, values.ma5, values.ma8 ];
                                    case 'SBG_OUTPUT_GYROSCOPES'
                                        varargout{n}(:, dataNum) = [values.g0; values.g1; values.g2];
                                    case 'SBG_OUTPUT_ACCELEROMETERS'
                                        varargout{n}(:, dataNum) = [values.a0; values.a1; values.a2];
                                    case 'SBG_OUTPUT_MAGNETOMETERS'
                                        varargout{n}(:, dataNum) = [values.m0; values.m1; values.m2];
                                    case 'SBG_OUTPUT_TEMPERATURES'
                                        varargout{n}(:, dataNum) = [values.t0; values.t1];

                                    case 'SBG_OUTPUT_GYROSCOPES_RAW'
                                        varargout{n}(:, dataNum) = [values.gR0; values.gR1; values.gR2];
                                    case 'SBG_OUTPUT_ACCELEROMETERS_RAW'
                                        varargout{n}(:, dataNum) = [values.aR0; values.aR1; values.aR2];
                                    case 'SBG_OUTPUT_MAGNETOMETERS_RAW'
                                        varargout{n}(:, dataNum) = [values.mR0; values.mR1; values.mR2];
                                    case 'SBG_OUTPUT_TEMPERATURES_RAW'
                                        varargout{n}(:, dataNum) = [values.tR0; values.tR1];

                                    case 'SBG_OUTPUT_TIME_SINCE_RESET'
                                        varargout{n}(:, dataNum) = values.timeSinceReset;
                                    case 'SBG_OUTPUT_DEVICE_STATUS'
                                        varargout{n}(:, dataNum) = values.deviceStatus;

                                    case 'SBG_OUTPUT_GPS_POSITION'
                                        varargout{n}(:, dataNum) = [values.gpsLatitude; values.gpsLongitude; values.gpsAltitude];
                                    case 'SBG_OUTPUT_GPS_NAVIGATION'
                                        varargout{n}(:, dataNum) = [values.gpsVelocity; values.gpsHeading];
                                    case 'SBG_OUTPUT_GPS_ACCURACY'
                                        varargout{n}(:, dataNum) = [values.gpsHorAccuracy; values.gpsVertAccuracy; values.gpsSpeedAccuracy; values.gpsHeadingAccuracy];
                                    case 'SBG_OUTPUT_GPS_INFO'
                                        varargout{n}(:, dataNum) = [values.gpsTimeMs; values.gpsFlags; values.gpsNbSats];
										
									case 'SBG_OUTPUT_GPS_TRUE_HEADING'
										varargout{n}(:, dataNum) = [values.gpsTrueHeading; values.gpsTrueHeadingAccuracy];

                                    case 'SBG_OUTPUT_BARO_ALTITUDE'
                                        varargout{n}(:, dataNum) = values.baroAltitude;
                                    case 'SBG_OUTPUT_BARO_PRESSURE'
                                        varargout{n}(:, dataNum) = values.baroPresure;

                                    case 'SBG_OUTPUT_POSITION'
                                        varargout{n}(:, dataNum) = [values.p0; values.p1; values.p2];
                                    case 'SBG_OUTPUT_VELOCITY'
                                        varargout{n}(:, dataNum) = [values.v0; values.v1; values.v2];
										
									case 'SBG_OUTPUT_ATTITUDE_ACCURACY'
										varargout{n}(:, dataNum) = values.attitudeAccuracy;
									case 'SBG_OUTPUT_NAV_ACCURACY'
										varargout{n}(:, dataNum) = [values.positionAccuracy; values.velocityAccuracy];

									case 'SBG_OUTPUT_GYRO_TEMPERATURES'
										varargout{n}(:, dataNum) = [values.gT0; values.gT1; values.gT2];
									case 'SBG_OUTPUT_GYRO_TEMPERATURES_RAW'
										varargout{n}(:, dataNum) = [values.gTR0; values.gTR1; values.gTR2];

									case 'SBG_OUTPUT_UTC_TIME_REFERENCE'
										varargout{n}(:, dataNum) = [values.utcYear; values.utcMonth; values.utcDay; values.utcHour; values.utcMin; values.utcSec; values.utcNano];
                                    case 'SBG_OUTPUT_MAG_CALIB_DATA'
                                        varargout{n}(:, dataNum) = [values.magCalibData0; values.magCalibData1; values.magCalibData2];
										
									case 'SBG_OUTPUT_ODO_VELOCITIES'
										varargout{n}(:, dataNum) = [values.odoRawVel0; values.odoRawVel1];
								
									case 'SBG_OUTPUT_DELTA_ANGLES'
										varargout{n}(:, dataNum) = [values.deltaAngleX; values.deltaAngleY; values.deltaAngleZ];
								
									case 'SBG_OUTPUT_HEAVE'
										varargout{n}(:, dataNum) = values.heave;
                                    end
                                end % End loop for each input args
                                
                            else
                                error('CSbgMatLab:getData', 'sbgMatLabGetData failed with %s', buildErrorMsg(this, errorCode));
                            end % End check if we have no error                            
                        end % End for each received data
                    end % End check if we have received some new data
                else
                    % Unable to receve some data
                    error('CSbgMatLab:getData', 'sbgMatLabHandleData failed with %s', buildErrorMsg(this, errorCode));
                end  % Check if we have received some data
            else
                % We havn't got the same number of input and outputs args
                error('CSbgMatLab:getData', 'You should have the same number of input and outputs variables arguments');
            end % End check number of arguments
		end
		
		% Get the last data received using the continous mode
		% For each asked data, returns a column vector based list
		% Example: [num, angles, accels] = obj.getLastestData('euler', 'accelerometers');
		% Note: numReceived should be equals to 1 if sucess or 0 if no new data has been received since last function call
        function [numReceived, varargout] = getLastestData(this, varargin)
            % First init out numReceived to 0
            numReceived = 0;
			
			% Create an empty string of 512 chars used to hold the trigger mask and create a pointer to it
			triggerMaskStr = blanks(512);
			pTriggerMaskStr = libpointer('stringPtr', triggerMaskStr);
            
           % Get the number of optional input arguments
            nInputArgs = nargin-1;
            nOutputArgs = max(nargout,1)-1;
            
            % Pre-fill varargout with 0
            for n=1:nOutputArgs
                varargout{n} = 0;
            end
            
            % Check if we have the same number of argin and argout if not,
            % we have an error
            if nInputArgs == nOutputArgs
                % Retreive the data and get the number of received data
                numNewReceivedData = uint32(0);
                pNewReceivedData = libpointer('uint32Ptr', numNewReceivedData);
                errorCode = calllib('sbgMatLab', 'sbgMatLabHandleData', this.sbgComHandle, pNewReceivedData);
 
                % Check if we have no error
                if (errorCode == 0)
                    % Define the number of received data
                    numReceived = pNewReceivedData.Value;
                    
                    % Check if we have received some data
                    if (numReceived > 0)
						% Get the last data in the list
						errorCode = calllib('sbgMatLab', 'sbgMatLabGetData', this.sbgComHandle, numReceived-1, this.pOutput, pTriggerMaskStr);
                            
						% Check if we have no error
						if (errorCode == 0)
							% Get the data values
							values = get(this.pOutput,'Value');

							% For each input argument, returns the data in
							% column vector formats
							for n=1:nInputArgs
								switch varargin{n}
								case 'SBG_OUTPUT_TRIGGER_MASK'
									% Convert the int8 array to a string and set the output
									varargout{n} = {pTriggerMaskStr.Value};
								case 'SBG_OUTPUT_QUATERNION'
									varargout{n} = [values.q0; values.q1; values.q2;  values.q3];
								case 'SBG_OUTPUT_EULER'
									varargout{n} = [values.roll; values.pitch; values.yaw];
								case 'SBG_OUTPUT_MATRIX'
									varargout{n} = [values.ma0, values.ma3, values.ma6;
													values.ma1, values.ma4, values.ma7;
													values.ma2, values.ma5, values.ma8 ];
								case 'SBG_OUTPUT_GYROSCOPES'
									varargout{n} = [values.g0; values.g1; values.g2];
								case 'SBG_OUTPUT_ACCELEROMETERS'
									varargout{n} = [values.a0; values.a1; values.a2];
								case 'SBG_OUTPUT_MAGNETOMETERS'
									varargout{n} = [values.m0; values.m1; values.m2];
								case 'SBG_OUTPUT_TEMPERATURES'
									varargout{n} = [values.t0; values.t1];

								case 'SBG_OUTPUT_GYROSCOPES_RAW'
									varargout{n} = [values.gR0; values.gR1; values.gR2];
								case 'SBG_OUTPUT_ACCELEROMETERS_RAW'
									varargout{n} = [values.aR0; values.aR1; values.aR2];
								case 'SBG_OUTPUT_MAGNETOMETERS_RAW'
									varargout{n} = [values.mR0; values.mR1; values.mR2];
								case 'SBG_OUTPUT_TEMPERATURES_RAW'
									varargout{n} = [values.tR0; values.tR1];

								case 'SBG_OUTPUT_TIME_SINCE_RESET'
									varargout{n} = values.timeSinceReset;
								case 'SBG_OUTPUT_DEVICE_STATUS'
									varargout{n} = values.deviceStatuts;

								case 'SBG_OUTPUT_GPS_POSITION'
									varargout{n} = [values.gpsLatitude; values.gpsLongitude; values.gpsAltitude];
								case 'SBG_OUTPUT_GPS_NAVIGATION'
									varargout{n} = [values.gpsVelocity; values.gpsHeading];
								case 'SBG_OUTPUT_GPS_ACCURACY'
									varargout{n} = [values.gpsHorAccuracy; values.gpsVertAccuracy; values.gpsSpeedAccuracy; values.gpsHeadingAccuracy];
								case 'SBG_OUTPUT_GPS_INFO'
									varargout{n} = [values.gpsTimeMs; values.gpsFlags; values.gpsNbSats];
									
								case 'SBG_OUTPUT_GPS_TRUE_HEADING'
									varargout{n} = [values.gpsTrueHeading; values.gpsTrueHeadingAccuracy];

								case 'SBG_OUTPUT_BARO_ALTITUDE'
									varargout{n} = values.baroAltitude;
								case 'SBG_OUTPUT_BARO_PRESSURE'
									varargout{n} = values.baroPresure;

								case 'SBG_OUTPUT_POSITION'
									varargout{n} = [values.p0; values.p1; values.p2];
								case 'SBG_OUTPUT_VELOCITY'
									varargout{n} = [values.v0; values.v1; values.v2];
									
								case 'SBG_OUTPUT_ATTITUDE_ACCURACY'
									varargout{n} = [values.attitudeAccuracy];
								case 'SBG_OUTPUT_NAV_ACCURACY'
									varargout{n} = [values.positionAccuracy; values.velocityAccuracy];

								case 'SBG_OUTPUT_GYRO_TEMPERATURES'
									varargout{n} = [values.gT0; values.gT1; values.gT2];
								case 'SBG_OUTPUT_GYRO_TEMPERATURES_RAW'
									varargout{n} = [values.gTR0; values.gTR1; values.gTR2];

								case 'SBG_OUTPUT_UTC_TIME_REFERENCE'
									varargout{n} = [values.utcYear; values.utcMonth; values.utcDay; values.utcHour; values.utcMin; values.utcSec; values.utcNano];
                                case 'SBG_OUTPUT_MAG_CALIB_DATA'
                                    varargout{n} = [values.magCalibData0; values.magCalibData1; values.magCalibData2];
									
								case 'SBG_OUTPUT_ODO_VELOCITIES'
									varargout{n} = [values.odoRawVel0; values.odoRawVel1];
								
								case 'SBG_OUTPUT_DELTA_ANGLES'
									varargout{n} = [values.deltaAngleX; values.deltaAngleY; values.deltaAngleZ];
								
								case 'SBG_OUTPUT_HEAVE'
									varargout{n} = values.heave;
								end
							end % End loop for each input args
						else
							error('CSbgMatLab:getLastestData', 'sbgMatLabGetData failed with %s', buildErrorMsg(this, errorCode));
						end % End check if we have no error
                    end % End check if we have received some new data
                else
                    % Unable to receve some data
                    error('CSbgMatLab:getLastestData', 'sbgMatLabHandleData failed with %s', buildErrorMsg(this, errorCode));
                end  % Check if we have received some data
            else
                % We havn't got the same number of input and outputs args
                error('CSbgMatLab:getLastestData', 'You should have the same number of input and outputs variables arguments');
            end % End check number of arguments
		end
		
		
		%----------------------------------------------------------------------
		%- Settings commands operations                                       -
		%----------------------------------------------------------------------

		% Gets device information such as product code, hardware and firmware revisions
		function [productCode, deviceId, firmwareVersion, calibDataVersion, mainBoardVersion, gpsBoardVersion] = getInfo(this)
			% Init the outputs variables
			productCode = blanks(32);
			deviceId = uint32(0);
			firmwareVersion = blanks(32);
			calibDataVersion = blanks(32);
			mainBoardVersion = blanks(32);
			gpsBoardVersion = blanks(32);
			
			% Create pointers for references args transmission
            pProductCode = libpointer('stringPtr', productCode);
			pDeviceId = libpointer('uint32Ptr', deviceId);
			pFirmwareVersion = libpointer('stringPtr', firmwareVersion);
			pCalibDataVersion = libpointer('stringPtr', calibDataVersion);
			pMainBoardVersion = libpointer('stringPtr', mainBoardVersion);
			pGpsBoardVersion = libpointer('stringPtr', gpsBoardVersion);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetInfo', this.sbgComHandle, pProductCode, pDeviceId, pFirmwareVersion, pCalibDataVersion, pMainBoardVersion, pGpsBoardVersion);
			
            % Check if we have an error
            if (errorCode == 0)
				% No error, copy the outputs args
				productCode = pProductCode.Value;
				deviceId = pDeviceId.Value;
				firmwareVersion = pFirmwareVersion.Value;
				calibDataVersion = pCalibDataVersion.Value;
				mainBoardVersion = pMainBoardVersion.Value;
				gpsBoardVersion = pGpsBoardVersion.Value;
			else
                error('CSbgMatLab:getInfo', 'sbgMatLabGetInfo failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines a user selectable ID for the device
		function setUserId(this, userId)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetUserId', this.sbgComHandle, userId);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setUserId', 'sbgMatLabSetUserId failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns the device user id.
		function [userId] = getUserId(this)
			% Init the outputs variables
			userId = uint32(0);
			
			% Create pointers for references args transmission
            pUserId = libpointer('uint32Ptr', userId);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetUserId', this.sbgComHandle, pUserId);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs value
				userId = pUserId.Value;
			else				
                error('CSbgMatLab:getUserId', 'sbgMatLabGetUserId failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the baud rate of the device uart communication and automatically change the computer baud rate to fit the new one.
		function [realBaudRate] = setProtocolMode(this, baudRate, uartOptions)
			% Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetProtocolMode', this.sbgComHandle, baudRate, uartOptions);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setProtocolMode', 'sbgMatLabSetProtocolMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Command used to get the current theorical baudrate used by the device.
		function [baudRate, uartOptions] = getProtocolMode(this)
			% Init the outputs values
			baudRate = uint32(0);
			uartOptions = blanks(256);
			
			% Create pointers for references args transmission
            pBaudRate = libpointer('uint32Ptr', baudRate);
			pUartOptions = libpointer('stringPtr', uartOptions);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetProtocolMode', this.sbgComHandle, pBaudRate, pUartOptions);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				baudRate = pBaudRate.Value;
				uartOptions = pUartOptions.Value;
			else
                error('CSbgMatLab:getProtocolMode', 'sbgMatLabGetProtocolMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
				
		% Defines the output mode of the target, big/little endian and float/fixed format.
		function setOutputMode(this, outputMode)			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetOutputMode', this.sbgComHandle, outputMode);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setOutputMode', 'sbgMatLabSetOutputMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Returns the continous mode option.
		function [outputMode] = getOutputMode(this)
			% Init the outputs values
			outputMode = blanks(256);
			
			% Create pointers for references args transmission
            pOutputMode = libpointer('stringPtr', outputMode);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetOutputMode', this.sbgComHandle, pOutputMode);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				outputMode = pOutputMode.Value;
			else
                error('CSbgMatLab:getOutputMode', 'sbgMatLabGetOutputMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the low power modes for the IG-Device
		function setLowPowerModes(this, devicePowerModeStr, gpsPowerModeStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetLowPowerModes', this.sbgComHandle, devicePowerModeStr, gpsPowerModeStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setLowPowerModes', 'sbgMatLabSetLowPowerModes failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Retrives the low power modes for the device.
		function [devicePowerModeStr, gpsPowerModeStr] = getLowPowerModes(this)
			% Init the outputs values
			devicePowerModeStr = blanks(256);
			gpsPowerModeStr = blanks(256);
			
			% Create pointers for references args transmission
            pDevicePowerModeStr = libpointer('stringPtr', devicePowerModeStr);
			pGpsPowerModeStr = libpointer('stringPtr', gpsPowerModeStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetLowPowerModes', this.sbgComHandle, pDevicePowerModeStr, pGpsPowerModeStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				devicePowerModeStr = pDevicePowerModeStr.Value;
				gpsPowerModeStr = pGpsPowerModeStr.Value;
			else
                error('CSbgMatLab:getLowPowerModes', 'sbgMatLabGetLowPowerModes failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Restore all settings to factory defaults (excepted for calibration data such as gyros bias and magnetometers calibration).
		% Automatically change the computer baud rate to the default device value.
		function restoreDefaultSettings(this) 
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabRestoreDefaultSettings', this.sbgComHandle);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:restoreDefaultSettings', 'sbgMatLabRestoreDefaultSettings failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Save current settings into the flash non volatile memory
		function saveSettings(this) 
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSaveSettings', this.sbgComHandle);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:saveSettings', 'sbgMatLabSaveSettings failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% This command set advanced settings options
		function setAdvancedOptions(this, advancedOptionsStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetAdvancedOptions', this.sbgComHandle, advancedOptionsStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setAdvancedOptions', 'sbgMatLabSetAdvancedOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% This command is used to retrieve advanced options from the device
		function [advancedOptionsStr] = getAdvancedOptions(this)
			% Init the outputs values
			advancedOptionsStr = blanks(256);
			
			% Create pointers for references args transmission
            pAdvancedOptionsStr = libpointer('stringPtr', advancedOptionsStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetAdvancedOptions', this.sbgComHandle, pAdvancedOptionsStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				advancedOptionsStr = pAdvancedOptionsStr.Value;
			else
                error('CSbgMatLab:getAdvancedOptions', 'sbgMatLabGetAdvancedOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Output configuration commands                                      -
		%----------------------------------------------------------------------
		
		% Define continous mode options, enabled/disabled and output frames speed.
		function setContinuousMode(this, continousMode, divider) 
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetContinuousMode', this.sbgComHandle, continousMode, divider);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setContinuousMode', 'sbgMatLabSetContinuousMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns the continous mode option.
		function [continuousMode, divider] = getContinuousMode(this)
			% Init the outputs values
			continuousMode = blanks(256);
			divider = uint8(0);
			
			% Create pointers for references args transmission
            pContinuousMode = libpointer('stringPtr', continuousMode);
			pDivider = libpointer('uint8Ptr', divider);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetContinuousMode', this.sbgComHandle, pContinuousMode, pDivider);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				continuousMode = pContinuousMode.Value;
				divider = pDivider.Value;
			else
                error('CSbgMatLab:getContinuousMode', 'sbgMatLabGetContinuousMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Configure the device default output mask
        function setDefaultOutputMask(this, outputMask)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatSetDefaultOutputMask', this.sbgComHandle, outputMask);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setDefaultOutputMask', 'sbgMatSetDefaultOutputMask failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Returns the default output mask.
		function [outputMask] = getDefaultOutputMask(this)
			% Init the outputs values
			outputMask = blanks(256);
			
			% Create pointers for references args transmission
            pOutputMask = libpointer('stringPtr', outputMask);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetDefaultOutputMask', this.sbgComHandle, pOutputMask);
            
            % Check if we have an error
            if (errorCode == 0)
				outputMask = pOutputMask.Value;
			else
                error('CSbgMatLab:getDefaultOutputMask', 'sbgMatLabGetDefaultOutputMask failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Define triggered output options
		function setTriggeredMode(this, condId, triggerMaskStr, outputMaskStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetTriggeredMode', this.sbgComHandle, condId, triggerMaskStr, outputMaskStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setTriggeredMode', 'sbgMatLabSetTriggeredMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Return a triggered output condition parameters
		function [triggerMaskStr, outputMaskStr] = getTriggeredMode(this, condId)
			% Init the outputs values			
			triggerMaskStr = blanks(256);
			outputMaskStr = blanks(256);
			
			% Create pointers for references args transmission
			pTriggerMaskStr = libpointer('stringPtr', triggerMaskStr);
			pOutputMaskStr = libpointer('stringPtr', outputMaskStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetTriggeredMode', this.sbgComHandle, condId, pTriggerMaskStr, pOutputMaskStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				triggerMaskStr = pTriggerMaskStr.Value;
				outputMaskStr = pOutputMaskStr.Value;				
			else
                error('CSbgMatLab:getTriggeredMode', 'sbgMatLabGetTriggeredMode failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Calibration commands                                               -
		%----------------------------------------------------------------------
		
		% Command used to start/stop/save a magnetometer calibration procedure.
		function calibMagnetometers(this, argumentStr) 
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabCalibMagnetometers', this.sbgComHandle, argumentStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:calibMagnetometers', 'sbgMatLabCalibMagnetometers failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
        end % End calibMagnetometers
		
		% Define the current magnetometer calibration parameters.
		function calibMagnetometersSetTransformations(this, offset, crossAxis)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabCalibMagnetometersSetTransformations', this.sbgComHandle, offset, crossAxis);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:calibMagnetometersSetTransformations', 'sbgMatLabCalibMagnetometersSetTransformations failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns the current magnetometer calibration parameters.
		function [offset, crossAxis] = calibMagnetometersGetTransformations(this)
			% Init the outputs values
			offset = zeros(3, 1, 'single');
			crossAxis = zeros(9, 1, 'single');
			
			% Create pointers for references args transmission
            pOffset = libpointer('singlePtr', offset);
			pCrossAxis = libpointer('singlePtr', crossAxis);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabCalibMagnetometersGetTransformations', this.sbgComHandle, pOffset, pCrossAxis);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				offset = pOffset.Value;
				crossAxis = pCrossAxis.Value;
			else
                error('CSbgMatLab:calibMagnetometersGetTransformations', 'sbgMatLabCalibMagnetometersGetTransformations failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Acquiere and save the current gyroscope bias value.
		function calibGyroBias(this, argumentStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabCalibGyroBias', this.sbgComHandle, argumentStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:calibGyroBias', 'sbgMatLabCalibGyroBias failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Kalman Filter commands                                             -
		%----------------------------------------------------------------------
				
		% Defines some options regarding the Kalman Filter.
		function setFilterAttitudeOptions(this, filterOptionsStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetFilterAttitudeOptions', this.sbgComHandle, filterOptionsStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setFilterAttitudeOptions', 'sbgMatLabSetFilterAttitudeOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Retreives some options regarding the Kalman Filter.
		function [filterOptionsStr] = getFilterAttitudeOptions(this)
			% Init the outputs values
			filterOptionsStr = blanks(256);
			
			% Create pointers for references args transmission
            pFilterOptionsStr = libpointer('stringPtr', filterOptionsStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetFilterAttitudeOptions', this.sbgComHandle, pFilterOptionsStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				filterOptionsStr = pFilterOptionsStr.Value;
			else
                error('CSbgMatLab:getFilterAttitudeOptions', 'sbgMatLabGetFilterAttitudeOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the sensors filter cut-off frequencies and the update rate for the main loop frequency.
		function setFilterFrequencies(this, gyroAccelsSampling, cutoffGyro, cutoffAccel, cutoffMagneto, mainLoopFrequency)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetFilterFrequencies', this.sbgComHandle, gyroAccelsSampling, cutoffGyro, cutoffAccel, cutoffMagneto, mainLoopFrequency);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setFilterFrequencies', 'sbgMatLabSetFilterFrequencies failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Retrives the sensors filter cut-off frequencies and the update rate for the Kalman Filter.
		function [gyroAccelsSampling, cutoffGyro, cutoffAccel, cutoffMagneto, mainLoopFrequency] = getFilterFrequencies(this)
			% Init the outputs values
			gyroAccelsSampling = single(0);
			cutoffGyro = single(0);
			cutoffAccel = single(0);
			cutoffMagneto = single(0);
			mainLoopFrequency = single(0);
			
			% Create pointers for references args transmission
            pGyroAccelsSampling = libpointer('singlePtr', gyroAccelsSampling);
			pCutoffGyro = libpointer('singlePtr', cutoffGyro);
			pCutoffAccel = libpointer('singlePtr', cutoffAccel);
			pCutoffMagneto = libpointer('singlePtr', cutoffMagneto);
			pMainLoopFrequency = libpointer('singlePtr', mainLoopFrequency);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetFilterFrequencies', this.sbgComHandle, pGyroAccelsSampling, pCutoffGyro, pCutoffAccel, pCutoffMagneto, pMainLoopFrequency);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				gyroAccelsSampling = pGyroAccelsSampling.Value;
				cutoffGyro = pCutoffGyro.Value;
				cutoffAccel = pCutoffAccel.Value;
				cutoffMagneto = pCutoffMagneto.Value;
				mainLoopFrequency = pMainLoopFrequency.Value;				
			else
                error('CSbgMatLab:getFilterFrequencies', 'sbgMatLabGetFilterFrequencies failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the kalman filter source for heading estimate.
		function setFilterHeadingSource(this, sourceStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetFilterHeadingSource', this.sbgComHandle, sourceStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setFilterHeadingSource', 'sbgMatLabSetFilterHeadingSource failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Retrives the kalman filter source for heading estimate.
		function [sourceStr] = getFilterHeadingSource(this)
			% Init the outputs values
			sourceStr = blanks(256);
			
			% Create pointers for references args transmission
            pSourceStr = libpointer('stringPtr', sourceStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetFilterHeadingSource', this.sbgComHandle, pSourceStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				sourceStr = pSourceStr.Value;
			else
                error('CSbgMatLab:getFilterHeadingSource', 'sbgMatLabGetFilterHeadingSource failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the magnetic declination in radians.
		function setMagneticDeclination(this, declination)			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetMagneticDeclination', this.sbgComHandle, declination);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setMagneticDeclination', 'sbgMatLabSetMagneticDeclination failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Retreives the magnetic declination in radians.
		function [declination] = getMagneticDeclination(this)
			% Init the outputs values
			declination = single(0);
			
			% Create pointers for references args transmission
			pDeclination = libpointer('singlePtr', declination);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetMagneticDeclination', this.sbgComHandle, pDeclination);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				declination = pDeclination.Value;
			else
                error('CSbgMatLab:getMagneticDeclination', 'sbgMatLabGetMagneticDeclination failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Send a new heading information to the Kalman filter in radians.
		function sendFilterHeading(this, heading, accuracy)			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSendFilterHeading', this.sbgComHandle, heading, accuracy);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:sendFilterHeading', 'sbgMatLabSendFilterHeading failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the Heave configuration
		function setHeaveConf(this, enableHeave)			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetHeaveConf', this.sbgComHandle, enableHeave);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setHeaveConf', 'sbgMatLabSetHeaveConf failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns the heave configuration
		function [enableHeave] = getHeaveConf(this)
			% Init the outputs values
			enableHeave = uint8(0);
			
			% Create pointers for references args transmission
			pEnableHeave = libpointer('uint8Ptr', enableHeave);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetHeaveConf', this.sbgComHandle, pEnableHeave);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				enableHeave = pEnableHeave.Value;
			else
                error('CSbgMatLab:getHeaveConf', 'sbgMatLabGetHeaveConf failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Orientation commands                                               -
		%----------------------------------------------------------------------
		
		% Defines the pre or post rotation to applied to the device.
		function setManualOrientationOffset(this, offsetTypeStr, rotationMatrix)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetManualOrientationOffset', this.sbgComHandle, offsetTypeStr, rotationMatrix);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setManualOrientationOffset', 'sbgMatLabSetManualOrientationOffset failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
	
		% Returns the pre or post rotation applied to the device.
		function [rotationMatrix] = getOrientationOffset(this, offsetTypeStr)
			% Init the outputs values
			rotationMatrix = zeros(9, 1, 'single');
			
			% Create pointers for references args transmission
            pRotationMatrix = libpointer('singlePtr', rotationMatrix);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetOrientationOffset', this.sbgComHandle, offsetTypeStr, pRotationMatrix);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				rotationMatrix = pRotationMatrix.Value;
			else
                error('CSbgMatLab:getOrientationOffset', 'sbgMatLabGetOrientationOffset failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
	
		% Command used to automatically calcualte a pre or post rotation matrix.
		function setAutoOrientationOffset(this, offsetTypeStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetAutoOrientationOffset', this.sbgComHandle, offsetTypeStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setAutoOrientationOffset', 'sbgMatLabSetAutoOrientationOffset failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Navigation commands                                                -
		%----------------------------------------------------------------------
		
		% Configures the reference pressure used for altitude calculation in pascals.
		function setReferencePressure(this, referencePressure)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetReferencePressure', this.sbgComHandle, referencePressure);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setReferencePressure', 'sbgMatLabSetReferencePressure failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%  Get the reference pressure used for altitude calculation in pascals
		function [referencePressure] = getReferencePressure(this)
			% Init the outputs values
			referencePressure = uint32(0);
			
			% Create pointers for references args transmission
			pReferencePressure = libpointer('uint32Ptr', referencePressure);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetReferencePressure', this.sbgComHandle, pReferencePressure);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				referencePressure = pReferencePressure.Value;
			else
                error('CSbgMatLab:getReferencePressure', 'sbgMatLabGetReferencePressure failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Configures the advanced GPS options.
		function setGpsOptions(this, gpsModelStr, gpsOptionsStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetGpsOptions', this.sbgComHandle, gpsModelStr, gpsOptionsStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setGpsOptions', 'sbgMatLabSetGpsOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get the advanced GPS options.
		function [gpsModelStr, gpsOptionsStr] = getGpsOptions(this)
			% Init the outputs values
			gpsModelStr = blanks(256);
			gpsOptionsStr = blanks(256);
			
			% Create pointers for references args transmission
            pGpsModelStr = libpointer('stringPtr', gpsModelStr);
			pGpsOptionsStr = libpointer('stringPtr', gpsOptionsStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetGpsOptions', this.sbgComHandle, pGpsModelStr, pGpsOptionsStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				gpsModelStr = pGpsModelStr.Value;
				gpsOptionsStr = pGpsOptionsStr.Value;
			else
                error('CSbgMatLab:getGpsOptions', 'sbgMatLabGetGpsOptions failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines which source to use to aid the velocity in the navigation filter.
		function setNavVelocitySrc(this, aidingSrcStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetNavVelocitySrc', this.sbgComHandle, aidingSrcStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setNavVelocitySrc', 'sbgMatLabSetNavVelocitySrc failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns which source is used to aid the velocity in the navigation filter.
		function [aidingSrcStr] = getNavVelocitySrc(this)
			% Init the outputs values
			aidingSrcStr = blanks(256);
			
			% Create pointers for references args transmission
            pAidingSrcStr = libpointer('stringPtr', aidingSrcStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetNavVelocitySrc', this.sbgComHandle, pAidingSrcStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				aidingSrcStr = pAidingSrcStr.Value;
			else
                error('CSbgMatLab:getNavVelocitySrc', 'sbgMatLabGetNavVelocitySrc failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines which source to use to aid the position in the navigation filter.
		function setNavPositionSrc(this, aidingSrcStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetNavPositionSrc', this.sbgComHandle, aidingSrcStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setNavPositionSrc', 'sbgMatLabSetNavPositionSrc failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns which source is used to aid the position in the navigation filter.
		function [aidingSrcStr] = getNavPositionSrc(this)
			% Init the outputs values
			aidingSrcStr = blanks(256);
			
			% Create pointers for references args transmission
            pAidingSrcStr = libpointer('stringPtr', aidingSrcStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetNavPositionSrc', this.sbgComHandle, pAidingSrcStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				aidingSrcStr = pAidingSrcStr.Value;
			else
                error('CSbgMatLab:getNavPositionSrc', 'sbgMatLabGetNavPositionSrc failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the GPS lever arm in meters.
		function setGpsLeverArm(this, gpsLeverArm)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetGpsLeverArm', this.sbgComHandle, gpsLeverArm);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setGpsLeverArm', 'sbgMatLabSetGpsLeverArm failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%  Returns the GPS lever arm in meters.
		function [gpsLeverArm] = getGpsLeverArm(this)
			% Init the outputs values
			gpsLeverArm = zeros(3, 1, 'single');
			
			% Create pointers for references args transmission
            pGpsLeverArm = libpointer('singlePtr', gpsLeverArm);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetGpsLeverArm', this.sbgComHandle, pGpsLeverArm);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				gpsLeverArm = pGpsLeverArm.Value;
			else
                error('CSbgMatLab:getGpsLeverArm', 'sbgMatLabGetGpsLeverArm failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the local gravity magnitude in m.s^-2.
		function setGravityMagnitude(this, gravityMagnitude)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetGravityMagnitude', this.sbgComHandle, gravityMagnitude);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setGravityMagnitude', 'sbgMatLabSetGravityMagnitude failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Returns the local gravity magnitude in m.s^-2.
		function [gravityMagnitude] = getGravityMagnitude(this)
			% Init the outputs values
			gravityMagnitude = single(0);
			
			% Create pointers for references args transmission
			pGravityMagnitude = libpointer('singlePtr', gravityMagnitude);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetGravityMagnitude', this.sbgComHandle, pGravityMagnitude);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				gravityMagnitude = pGravityMagnitude.Value;
			else
                error('CSbgMatLab:getGravityMagnitude', 'sbgMatLabGetGravityMagnitude failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
				
		% Send a new velocity information to the Navigation filter in m/s in device frame.
		function sendNavVelocity(this, velocity, accuracy)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSendNavVelocity', this.sbgComHandle, velocity, accuracy);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:sendNavVelocity', 'sbgMatLabSendNavVelocity failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Send a new position information to the Navigation filter in WGS84 (deg,deg,meters) in device frame and accuracy in meters.
		function sendNavPosition(this, position, hAccuracy, vAccuracy)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSendNavPosition', this.sbgComHandle, position, hAccuracy, vAccuracy);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:sendNavPosition', 'sbgMatLabSendNavPosition failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Odometer configurations                                            -
		%----------------------------------------------------------------------
		
		% Set the main configuration of the external odometer channels
		function setOdoConfig(this, channel, axisStr, pulsesPerMeter, gainError, gpsGainCorrection)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetOdoConfig', this.sbgComHandle, channel, axisStr, pulsesPerMeter, gainError, gpsGainCorrection);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setOdoConfig', 'sbgMatLabSetOdoConfig failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get the main configuration of the external odometer channels
		function [axisStr, pulsesPerMeter, gainError, gpsGainCorrection] = getOdoConfig(this, channel)
			% Init the outputs values
			axisStr = blanks(256);
			pulsesPerMeter = single(0);
			gainError = uint8(0);
			gpsGainCorrection = uint8(0);
			
			% Create pointers for references args transmission
            pAxisStr = libpointer('stringPtr', axisStr);
			pPulsesPerMeter = libpointer('singlePtr', pulsesPerMeter);
			pGainError = libpointer('uint8Ptr', gainError);
			pGpsGainCorrection = libpointer('uint8Ptr', gpsGainCorrection);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetOdoConfig', this.sbgComHandle, channel, pAxisStr, pPulsesPerMeter, pGainError, pGpsGainCorrection);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				axisStr = pAxisStr.Value;
				pulsesPerMeter = pPulsesPerMeter.Value;
				gainError = pGainError.Value;
				gpsGainCorrection = pGpsGainCorrection.Value;
			else
                error('CSbgMatLab:getOdoConfig', 'sbgMatLabGetOdoConfig failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Configures the odometer direction for the corresponding channel
		function setOdoDirection(this, channel, odoDirectionStr)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetOdoDirection', this.sbgComHandle, channel, odoDirectionStr);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setOdoDirection', 'sbgMatLabSetOdoDirection failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get the odometer direction for the corresponding channel
		function [odoDirectionStr] = getOdoDirection(this, channel)
			% Init the outputs values
			odoDirectionStr = blanks(256);
						
			% Create pointers for references args transmission
            pOdoDirectionStr = libpointer('stringPtr', odoDirectionStr);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetOdoDirection', this.sbgComHandle, channel, pOdoDirectionStr);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				odoDirectionStr = pOdoDirectionStr.Value;
			else
                error('CSbgMatLab:getOdoDirection', 'sbgMatLabGetOdoDirection failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Configures the odometer lever arm for the corresponding channel
		function setOdoLeverArm(this, channel, odoLeverArm)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetOdoLeverArm', this.sbgComHandle, channel, odoLeverArm);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setOdoLeverArm', 'sbgMatLabSetOdoLeverArm failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get the odometer lever arm for the corresponding channel
		function [odoLeverArm] = getOdoLeverArm(this, channel)
			% Init the outputs values
			odoLeverArm = zeros(3, 1, 'single');
			
			% Create pointers for references args transmission
            pOdoLeverArm = libpointer('singlePtr', odoLeverArm);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetOdoLeverArm', this.sbgComHandle, channel, pOdoLeverArm);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				odoLeverArm = pOdoLeverArm.Value;
			else
                error('CSbgMatLab:getOdoLeverArm', 'sbgMatLabGetOdoLeverArm failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		%----------------------------------------------------------------------
		%- Synchronization input and output operations                        -
		%----------------------------------------------------------------------
		
		% Set a logic input channel setting
		function setLogicInChannel(this, channel, inputTypeStr, sensitivityStr, locationStr, nsDelay)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetLogicInChannel', this.sbgComHandle, channel, inputTypeStr, sensitivityStr, locationStr, nsDelay);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setLogicInChannel', 'sbgMatLabSetLogicInChannel failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get a logic input channel setting
		function [inputTypeStr, sensitivityStr, locationStr, nsDelay] = getLogicInChannel(this, channel)
			% Init the outputs values
			inputTypeStr = blanks(256);
			sensitivityStr = blanks(256);
			locationStr = blanks(256);
			nsDelay = int32(0);
			
			% Create pointers for references args transmission
            pInputTypeStr = libpointer('stringPtr', inputTypeStr);
			pSensitivityStr = libpointer('stringPtr', sensitivityStr);
			pLocationStr = libpointer('stringPtr', locationStr);
			pNsDelay = libpointer('int32Ptr', nsDelay);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetLogicInChannel', this.sbgComHandle, channel, pInputTypeStr, pSensitivityStr, pLocationStr, pNsDelay);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				inputTypeStr = pInputTypeStr.Value;
				sensitivityStr = pSensitivityStr.Value;
				locationStr = pLocationStr.Value;
				nsDelay = pNsDelay.Value;
			else
                error('CSbgMatLab:getLogicInChannel', 'sbgMatLabGetLogicInChannel failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Set a logic output channel setting
		function setLogicOutChannel(this, channel, outputTypeStr, polarityStr, duration)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetLogicOutChannel', this.sbgComHandle, channel, outputTypeStr, polarityStr, duration);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setLogicOutChannel', 'sbgMatLabSetLogicOutChannel failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Get a logic output channel setting
		function [outputTypeStr, polarityStr, duration] = getLogicOutChannel(this, channel)
			% Init the outputs values
			outputTypeStr = blanks(256);
			polarityStr = blanks(256);
			duration = uint8(0);
			
			% Create pointers for references args transmission
            pOutputTypeStr = libpointer('stringPtr', outputTypeStr);
			pPolarityStr = libpointer('stringPtr', polarityStr);
			pDuration = libpointer('uint8Ptr', duration);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetLogicOutChannel', this.sbgComHandle, channel, pOutputTypeStr, pPolarityStr, pDuration);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs values
				outputTypeStr = pOutputTypeStr.Value;
				polarityStr = pPolarityStr.Value;
				duration = pDuration.Value;
			else
                error('CSbgMatLab:getLogicOutChannel', 'sbgMatLabGetLogicOutChannel failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end
		
		% Defines the distance between two pulses when sync out is configured as a virtual odometer
		function setVirtualOdoConf(this, distance)
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabSetVirtualOdoConf', this.sbgComHandle, distance);
            
            % Check if we have an error
            if (errorCode ~= 0)
                error('CSbgMatLab:setVirtualOdoConf', 'sbgMatLabSetVirtualOdoConf failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end

		% Retrieves the distance between two pulses when sync out is configured as a virtual odometer
		function [distance] = getVirtualOdoConf(this)
			% Init the outputs values
			distance = single(0);
			
			% Create pointers for references args transmission
			pDistance = libpointer('singlePtr', distance);
			
            % Call the dll function
            errorCode = calllib('sbgMatLab', 'sbgMatLabGetVirtualOdoConf', this.sbgComHandle, pDistance);
            
            % Check if we have an error
            if (errorCode == 0)
				% Copy the outputs args
				distance = pDistance.Value;
			else
                error('CSbgMatLab:getVirtualOdoConf', 'sbgMatLabGetVirtualOdoConf failed with %s', buildErrorMsg(this, errorCode));
            end % End check if we have an error
		end		
	end % methods
end % classdef
