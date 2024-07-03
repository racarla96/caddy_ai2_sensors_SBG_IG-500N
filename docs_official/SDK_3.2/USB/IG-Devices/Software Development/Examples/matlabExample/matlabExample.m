function [triggerMask] = matlabExample(comPort,speed,numSamples)
%
% comPort : Set here a string with the name of the com port (ex: 'COM1')
% speed : communication speed in bps (ex: 155200)
% numSamples : Number of samples to acquiere (ex: 500)
%
% This small example shows how to use the CSbgMatLab class as an interface
% with IG devices.
%
% The example here connects to a device, then get some settings of it, and
% retrieve 1000 samples of euler angles, and finally display result on a plot.
%

% Create the CSbgMatlab object
igDevice = CSbgMatLab();

% Init the number of samples we would like


% Init our variable that will contain numSamples euler angles and utc infos
euler = zeros(3, numSamples);
utcTmp = zeros(7, numSamples);

% Init communications with the device.
igDevice.init(comPort,speed);

try

	% Display the product code of the device
	igDevice.getInfo()

	% Set output mask to euler angles and enable continuous mode
	igDevice.setDefaultOutputMask('SBG_OUTPUT_EULER|SBG_OUTPUT_UTC_TIME_REFERENCE');
	igDevice.setContinuousMode('SBG_CONTINUOUS_MODE_ENABLE', 1)

	% Initialize our sample counter
	n = 0;

	while n < numSamples

		% Get all last data received
		[num, triggerMask, eulerTmp, utcTmp] = igDevice.getData('SBG_OUTPUT_TRIGGER_MASK', 'SBG_OUTPUT_EULER', 'SBG_OUTPUT_UTC_TIME_REFERENCE');
		
		
		
		%copy data into our euler angle list    
		if (num > 0)
			triggerMask
			euler(:,n+1:n+num)= eulerTmp;
		end

		% update our sample counter (display the sample number on the console)
		n = n+num;

	end

catch
	% Close our device
	igDevice.close();

	% Delete our object in order to free memory and com port
	clear igDevice;
	
	% Rethrow the error
	rethrow(lasterror);
end

% Close our device
igDevice.close();

% Delete our object in order to free memory and com port
clear igDevice;
%
% Display results
%
plot(euler(1,:),'r');
hold on
plot(euler(2,:),'g');
plot(euler(3,:),'b');
