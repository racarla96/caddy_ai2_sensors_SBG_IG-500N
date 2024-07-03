function testSbgMatLab(comPort, baudRate)
	% Create an instance
	instance = CSbgMatLab();
	
	% Clear our main screen
	clc;
	
	% Get our library version
	[sbgMatLabVersion, sbgComVersion] = instance.getVersion();
	
	% Initialise our library
	instance.init(comPort, baudRate);
	
	% If our system was init, display a welcome message
	disp(['sbgMatLab system version ' sbgMatLabVersion ' using sbgCom library version ' sbgComVersion ' sucessfully initialised.']);
	
	%----------------------------------------------------------------------
	%- Settings commands operations                                       -
	%----------------------------------------------------------------------
		
	% Test all common function
	[productCode, deviceId, firmwareVersion, calibDataVersion, mainBoardVersion, gpsBoardVersion] = instance.getInfo();
	disp(['Device ' productCode ' with id ' num2str(deviceId) ' is using firmware version ' firmwareVersion ' with calibration version ' calibDataVersion ' and hardware version ' mainBoardVersion ' / ' gpsBoardVersion]);
	
	% Test user id
	instance.setUserId(777);
	if (instance.getUserId() == 777)
		disp('setUserId/getUserId test ok!');
	else
		error('setUserId/getUserId test failed.');
	end
	
	% Test protocol mode
	instance.setProtocolMode(115200, 'SBG_PROTOCOL_DIS_TX_EMI_REDUCTION');
	[baudRate, uartOptions] = instance.getProtocolMode();
	
	if ( (baudRate == 115200) && (strcmp(uartOptions, 'SBG_PROTOCOL_DIS_TX_EMI_REDUCTION') == 1) )
		disp('setProtocolMode/getProtocolMode test ok!');
	else
		error('setProtocolMode/getProtocolMode test failed.');
	end

	% Test output mode
	instance.setOutputMode('SBG_OUTPUT_MODE_LITTLE_ENDIAN|SBG_OUTPUT_MODE_FIXED');
	if ( strcmp(instance.getOutputMode(), 'SBG_OUTPUT_MODE_LITTLE_ENDIAN|SBG_OUTPUT_MODE_FIXED') == 1)
		disp('setOutputMode/getOutputMode test ok!');
	else
		error('setOutputMode/getOutputMode test failed.');
	end
	
	% Test our low power modes
% 	instance.setLowPowerModes('SBG_DEVICE_MAX_PERF', 'SBG_GPS_OFF_MODE');
% 	[devicePowerMode, gpsPowerMode] = instance.getLowPowerModes()
% 	if ( (strcmp(devicePowerMode, 'SBG_DEVICE_MAX_PERF') == 1) && (strcmp(gpsPowerMode, 'SBG_GPS_OFF_MODE') == 1) )
% 		disp('setLowPowerModes/getLowPowerModes test ok!');
% 	else
% 		error('setLowPowerModes/getLowPowerModes test failed.');
% 	end
	
	% Test our restore default settings
	instance.restoreDefaultSettings();
	disp('restoreDefaultSettings test ok!');
	
	% Test the save settings
	instance.saveSettings();
	disp('saveSettings test ok!');
	
	% Test advanced settings
	instance.setAdvancedOptions('SBG_SETTING_ENABLE_CONING|SBG_SETTING_OUTPUT_UNBIASED_GYRO');
	[advancedOptions] = instance.getAdvancedOptions();
	
	if (strcmp(advancedOptions, 'SBG_SETTING_ENABLE_CONING|SBG_SETTING_OUTPUT_UNBIASED_GYRO') == 1)
		disp('setAdvancedOptions/getAdvancedOptions test ok!');
	else
		error('setAdvancedOptions/getAdvancedOptions test failed.');
	end
	
	%----------------------------------------------------------------------
	%- Output configuration commands                                      -
	%----------------------------------------------------------------------
	
	% Test continuous mode
	instance.setContinuousMode('SBG_CONTINUOUS_MODE_ENABLE', 10);
	[continuousMode, divider] = instance.getContinuousMode();
	if ( (strcmp(continuousMode, 'SBG_CONTINUOUS_MODE_ENABLE') == 1) && (divider == 10) )
		disp('setContinuousMode/getContinuousMode test ok!');
	else
		error('setContinuousMode/getContinuousMode test failed.');
	end
	
	% Test our default output mask
	instance.setDefaultOutputMask('SBG_OUTPUT_EULER|SBG_OUTPUT_GYROSCOPES|SBG_OUTPUT_ACCELEROMETERS');
	if ( strcmp(instance.getDefaultOutputMask(), 'SBG_OUTPUT_EULER|SBG_OUTPUT_GYROSCOPES|SBG_OUTPUT_ACCELEROMETERS') == 1)
		disp('setDefaultOutputMask/getDefaultOutputMask test ok!');
	else
		error('setDefaultOutputMask/getDefaultOutputMask test failed.');
	end	
	
	% Test the triggered mode
	instance.setTriggeredMode(0, 'SBG_TRIGGER_MAIN_LOOP_DIVIDER|SBG_TRIGGER_MAGNETOMETERS', 'SBG_OUTPUT_EULER|SBG_OUTPUT_DELTA_ANGLES');
	[triggerMask, outputMask] = instance.getTriggeredMode(0);
	if ( (strcmp(triggerMask, 'SBG_TRIGGER_MAIN_LOOP_DIVIDER|SBG_TRIGGER_MAGNETOMETERS') == 1) && (strcmp(outputMask, 'SBG_OUTPUT_EULER|SBG_OUTPUT_DELTA_ANGLES') == 1) )
		disp('setTriggeredMode/getTriggeredMode test ok!');
	else
		error('setTriggeredMode/getTriggeredMode test failed.');
	end
	
	%----------------------------------------------------------------------
	%- Calibration commands                                               -
	%----------------------------------------------------------------------
	
	% Test magnetometers calibration actions
	instance.calibMagnetometers('SBG_CALIB_MAGS_LOAD_DEFAULT');
	disp('calibMagnetometers test ok!');
	
	% Test magnetometers calibMagnetometersGetTransformations
	[magsOffset, magsCrossAxis] = instance.calibMagnetometersGetTransformations();
	disp('calibMagnetometersGetTransformations test ok!');
	
	% Test magnetometers calibMagnetometersSetTransformations
	instance.calibMagnetometersSetTransformations(magsOffset, magsCrossAxis);
	disp('calibMagnetometersSetTransformations test ok!');
	
	% Test gyroscopes bias calibration
	instance.calibGyroBias('SBG_CALIB_GYROS_LOAD_DEFAULT');
	disp('calibGyroBias test ok!');
	
	%----------------------------------------------------------------------
	%- Kalman Filter commands                                             -
	%----------------------------------------------------------------------
		
	% Test filter attitude options
% 	instance.setFilterAttitudeOptions('SBG_FILTER_OPTION_ENABLE_ATTITUDE');
% 	if (strcmp(instance.getFilterAttitudeOptions(), 'SBG_FILTER_OPTION_ENABLE_ATTITUDE') == 1)
% 		disp('setFilterAttitudeOptions/getFilterAttitudeOptions test ok!');
% 	else
% 		error('setFilterAttitudeOptions/getFilterAttitudeOptions test failed.');
% 	end
	
	% Test filter frequencies
% 	instance.setFilterFrequencies(0, 1, 2, 3, 25);
% 	[gyroAccelsSampling, cutoffGyro, cutoffAccel, cutoffMagneto, kalmanFreq] = instance.getFilterFrequencies();
% 	if ( (abs(cutoffGyro-1)<0.01) && (abs(cutoffAccel-2)<0.01) && (abs(cutoffMagneto-3)<0.01) && (abs(kalmanFreq-25)<0.01) )
% 		disp('setFilterFrequencies/getFilterFrequencies test ok!');
% 	else
% 		error('setFilterFrequencies/getFilterFrequencies test failed.');
% 	end
	
	% Test filter heading source
	instance.setFilterHeadingSource('SBG_HEADING_SOURCE_MAGNETOMETERS');
	if (strcmp(instance.getFilterHeadingSource(), 'SBG_HEADING_SOURCE_MAGNETOMETERS') == 1)
		disp('setFilterHeadingSource/getFilterHeadingSource test ok!');
	else
		error('setFilterHeadingSource/getFilterHeadingSource test failed.');
	end
	
	% Test magnetic declination
	instance.setMagneticDeclination(2/180*pi);
	if (abs(instance.getMagneticDeclination()-2/180*pi)<0.01)
		disp('setMagneticDeclination/getMagneticDeclination test ok!');
	else
		error('setMagneticDeclination/getMagneticDeclination test failed.');
	end
	
	% Test send filter heading
	instance.sendFilterHeading(45/180*pi, 1/180*pi);
	disp('sendFilterHeading test ok!');
	
	% Test the heave configuration
 	instance.setHeaveConf(false);
 	if (instance.getHeaveConf() == false)
 		disp('setHeaveConf/getHeaveConf test ok!');
 	else
 		error('setHeaveConf/getHeaveConf test failed.');
 	end
	
	%----------------------------------------------------------------------
	%- Orientation commands                                               -
	%----------------------------------------------------------------------
	
	% Test the auto orientation offset
	instance.setAutoOrientationOffset('SBG_OFFSET_PRE_ROT_Z_RESET');
	disp('setAutoOrientationOffset test ok!');
	
	% Create an identity matrix
	identityMatrix = [	1;0;0;
						0;1;0;
						0;0;1];
	
	% Test the manual orientation offset command	
	instance.setManualOrientationOffset('SBG_OFFSET_PRE_ROT', identityMatrix);
	if (instance.getOrientationOffset('SBG_OFFSET_PRE_ROT') == identityMatrix)
		disp('setManualOrientationOffset/getOrientationOffset test ok!');
	else
		error('setManualOrientationOffset/getOrientationOffset test failed.');
	end
	
	%----------------------------------------------------------------------
	%- Navigation commands                                                -
	%----------------------------------------------------------------------
	
	% Test reference pressure
	instance.setReferencePressure(101326);
	if (instance.getReferencePressure() == 101326)
		disp('setReferencePressure/getReferencePressure test ok!');
	else
		error('setReferencePressure/getReferencePressure test failed.');
	end
	
	% Test gps options
% 	instance.setGpsOptions('SBG_GPS_MODEL_SEA', 'SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS|SBG_GPS_ENABLE_SBAS_RANGING');
% 	[gpsModelStr, gpsOptionsStr] = instance.getGpsOptions();
% 	if ( (strcmp(gpsModelStr, 'SBG_GPS_MODEL_SEA') == 1) && (strcmp(gpsOptionsStr, 'SBG_GPS_ENABLE_SBAS_DIFF_CORRECTIONS|SBG_GPS_ENABLE_SBAS_RANGING') == 1) )
% 		disp('setGpsOptions/getGpsOptions test ok!');
% 	else
% 		error('setGpsOptions/getGpsOptions test failed.');
% 	end
	
	% Test nav velocity src
	instance.setNavVelocitySrc('SBG_VEL_SRC_USER');
	if (strcmp(instance.getNavVelocitySrc(), 'SBG_VEL_SRC_USER') == 1)
		disp('setNavVelocitySrc/getNavVelocitySrc test ok!');
	else
		error('setNavVelocitySrc/getNavVelocitySrc test failed.');
	end
	
	% Test nav position src
	instance.setNavPositionSrc('SBG_POS_SRC_USER');
	if (strcmp(instance.getNavPositionSrc(), 'SBG_POS_SRC_USER') == 1)
		disp('setNavPositionSrc/getNavPositionSrc test ok!');
	else
		error('setNavPositionSrc/getNavPositionSrc test failed.');
	end
	
	% Test gps lever arm
	instance.setGpsLeverArm([1.0, 2.0, 3.0]);
	[gpsLeverArm] = instance.getGpsLeverArm();
	if ( (abs(gpsLeverArm(1)-1.0)<0.01) && (abs(gpsLeverArm(2)-2.0)<0.01) && (abs(gpsLeverArm(3)-3.0)<0.01) )
		disp('setGpsLeverArm/getGpsLeverArm test ok!');
	else
		error('setGpsLeverArm/getGpsLeverArm test failed.');
	end
	
	% Test gravity magnitude
	instance.setGravityMagnitude(10.453);
	if (abs(instance.getGravityMagnitude()-10.453)<0.01)
		disp('setGravityMagnitude/getGravityMagnitude test ok!');
	else
		error('setGravityMagnitude/getGravityMagnitude test failed.');
	end
	
	% Test send nav velocity
	instance.sendNavVelocity([5, 1, 2], 0.1);
	disp('sendNavVelocity test ok!');
	
	% Test send nav position
	instance.sendNavPosition([48.51, 2.17, 10] , 0.5, 1);
	disp('sendNavPosition test ok!');
	
	%----------------------------------------------------------------------
	%- Odometer configurations                                            -
	%----------------------------------------------------------------------
	
	% Test the odometer configuration
	instance.setOdoConfig(0, 'SBG_ODO_X', 1.345, 20, true);
	[odoAxisStr, pulsesPerMeter, gainError, autoGpsGain] = instance.getOdoConfig(0);
	if ( (strcmp(odoAxisStr, 'SBG_ODO_X') == 1) && (abs(pulsesPerMeter-1.345)<0.01) && (gainError == 20) && (autoGpsGain == true) )
		disp('setOdoConfig/getOdoConfig test ok!');
	else
		error('setOdoConfig/getOdoConfig test failed.');
	end
	
	% Test the odometer configuration
	instance.setOdoDirection(0, 'SBG_ODO_DIR_NEGATIVE');
	if (strcmp(instance.getOdoDirection(0), 'SBG_ODO_DIR_NEGATIVE') == 1)
		disp('setOdoDirection/getOdoDirection test ok!');
	else
		error('setOdoDirection/getOdoDirection test failed.');
	end
	
	% Test odometer GPS lever arm
	instance.setOdoLeverArm(0, [1.0, 2.0, 3.0]);
	[odoLeverArm] = instance.getOdoLeverArm(0);
	if ( (abs(odoLeverArm(1)-1.0)<0.01) && (abs(odoLeverArm(2)-2.0)<0.01) && (abs(odoLeverArm(3)-3.0)<0.01) )
		disp('setOdoLeverArm/getOdoLeverArm test ok!');
	else
		error('setOdoLeverArm/getOdoLeverArm test failed.');
	end
	
	%----------------------------------------------------------------------
	%- Synchronization input and output operations                        -
	%----------------------------------------------------------------------
	
	% Test logic channel input configuration
	instance.setLogicInChannel(0, 'SBG_IN_TIME_PULSE', 'SBG_IN_LEVEL_CHANGE', 'SBG_IN_STD_LOCATION', 1256);
	[inputTypeStr, sensitivtyStr, locationStr, nsDelay] = instance.getLogicInChannel(0);
	if ( (strcmp(inputTypeStr, 'SBG_IN_TIME_PULSE') == 1) && (strcmp(sensitivtyStr, 'SBG_IN_LEVEL_CHANGE') == 1) && (strcmp(locationStr, 'SBG_IN_STD_LOCATION') == 1) && (nsDelay == 1256) )
		disp('setLogicInChannel/getLogicInChannel test ok!');
	else
		error('setLogicInChannel/getLogicInChannel test failed.');
	end
	
	% Test logic channel output configuration
	instance.setLogicOutChannel(0, 'SBG_OUT_MAIN_LOOP_DIVIDER', 'SBG_OUT_TOGGLE', 124);
	[outputTypeStr, polarityStr, duration] = instance.getLogicOutChannel(0);
	if ( (strcmp(outputTypeStr, 'SBG_OUT_MAIN_LOOP_DIVIDER') == 1) && (strcmp(polarityStr, 'SBG_OUT_TOGGLE') == 1) && (duration == 124) )
		disp('setLogicOutChannel/getLogicOutChannel test ok!');
	else
		error('setLogicOutChannel/getLogicOutChannel test failed.');
	end
		
	% Test virtual odometer configuration
	instance.setVirtualOdoConf(12.55);
	if (abs(instance.getVirtualOdoConf()-12.55)<0.01)
		disp('setVirtualOdoConf/getVirtualOdoConf test ok!');
	else
		error('setVirtualOdoConf/getVirtualOdoConf test failed.');
	end
	
	% Destroy our instance
	instance.close();
	clear instance;
end
