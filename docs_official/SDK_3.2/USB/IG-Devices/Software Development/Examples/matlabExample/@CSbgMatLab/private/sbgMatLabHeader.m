function [methodinfo,structs,enuminfo,ThunkLibName]=sbgMatLabHeader
%SBGMATLABHEADER Create structures to define interfaces found in 'sbgMatLab'.

% Support for sbgMatLab version 1.6
ival={cell(1,0)}; % change 0 to the actual number of functions to preallocate the data.
structs=[];enuminfo=[];fcnNum=1;
fcns=struct('name',ival,'calltype',ival,'LHS',ival,'RHS',ival,'alias',ival);
ThunkLibName=[];

%----------------------------------------------------------------------//
%-  Library general operations                                        -//
%----------------------------------------------------------------------//

% SbgErrorCode sbgMatLabInit(const char * deviceName, uint32 baudRate, SbgMatLabHandle **pHandle); 
fcns.name{fcnNum}='sbgMatLabInit'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'cstring', 'uint32', 'voidPtrPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabClose(SbgMatLabHandle *pHandle); 
fcns.name{fcnNum}='sbgMatLabClose'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr'};fcnNum=fcnNum+1;

% void sbgMatLabGetVersion(char sbgMatLabVersion[32], char sbgComVersion[32]);
fcns.name{fcnNum}='sbgMatLabGetVersion'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}=[]; fcns.RHS{fcnNum}={'stringPtr', 'stringPtr'};fcnNum=fcnNum+1;

% void sbgMatLabErrorToString(SbgErrorCode errorCode, char errorMsg[256]);
fcns.name{fcnNum}='sbgMatLabErrorToString'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}=[]; fcns.RHS{fcnNum}={'uint32', 'stringPtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%-  Data management operations                                        -//
%----------------------------------------------------------------------//

% SbgErrorCode sbgMatLabHandleData (SbgMatLabHandle *pHandle, uint32 * pNumData ); 
fcns.name{fcnNum}='sbgMatLabHandleData'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32Ptr'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetData(SbgMatLabHandle *pHandle, uint32 index, SbgMatLabData *pData, char triggerMask[512]);
fcns.name{fcnNum}='sbgMatLabGetData'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32', 's_SbgMatLabDataPtr', 'stringPtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Settings commands operations                                       -//
%----------------------------------------------------------------------//

%SbgErrorCode sbgMatLabGetInfo(SbgMatLabHandle *pHandle, char productCode[32], uint32 *pDeviceId, char firmwareVersion[32], char calibDataVersion[32], char mainBoardVersion[32], char gpsBoardVersion[32]);
fcns.name{fcnNum}='sbgMatLabGetInfo'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr', 'uint32Ptr', 'stringPtr', 'stringPtr', 'stringPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetUserId(SbgMatLabHandle * pHandle , uint32 userId ); 
fcns.name{fcnNum}='sbgMatLabSetUserId'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetUserId(SbgMatLabHandle * pHandle , uint32 * pUserId ); 
fcns.name{fcnNum}='sbgMatLabGetUserId'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetProtocolMode(SbgMatLabHandle *pHandle, uint32 baudRate, const char uartOptionsStr[256]);
fcns.name{fcnNum}='sbgMatLabSetProtocolMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetProtocolMode(SbgMatLabHandle *pHandle, uint32 *pBaudRate, char uartOptionsStr[256]); 
fcns.name{fcnNum}='sbgMatLabGetProtocolMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32Ptr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetOutputMode(SbgMatLabHandle *pHandle, const char outputModeStr[256]); 
fcns.name{fcnNum}='sbgMatLabSetOutputMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetOutputMode(SbgMatLabHandle *pHandle, char outputModeStr[256]);
fcns.name{fcnNum}='sbgMatLabGetOutputMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetLowPowerModes(SbgMatLabHandle *pHandle,  const char devicePowerModeStr[256], const char gpsPowerModeStr[256]);
fcns.name{fcnNum}='sbgMatLabSetLowPowerModes'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetLowPowerModes(SbgMatLabHandle *pHandle, char devicePowerModeStr[256], char gpsPowerModeStr[256]);
fcns.name{fcnNum}='sbgMatLabGetLowPowerModes'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabRestoreDefaultSettings(SbgMatLabHandle *pHandle); 
fcns.name{fcnNum}='sbgMatLabRestoreDefaultSettings'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSaveSettings(SbgMatLabHandle *pHandle); 
fcns.name{fcnNum}='sbgMatLabSaveSettings'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetAdvancedOptions(SbgMatLabHandle *pHandle, const char advancedOptionsStr[256]);
fcns.name{fcnNum}='sbgMatLabSetAdvancedOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetAdvancedOptions(SbgMatLabHandle *pHandle, char advancedOptionsStr[256]);
fcns.name{fcnNum}='sbgMatLabGetAdvancedOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Output configuration commands                                      -//
%----------------------------------------------------------------------//

% SbgErrorCode sbgMatLabSetContinuousMode(SbgMatLabHandle *pHandle, const char contModeStr[256], uint8 divider); 
fcns.name{fcnNum}='sbgMatLabSetContinuousMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring', 'uint8'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetContinuousMode(SbgMatLabHandle *pHandle, char contModeStr[256], uint8 *pDivider); 
fcns.name{fcnNum}='sbgMatLabGetContinuousMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr', 'uint8Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatSetDefaultOutputMask(SbgMatLabHandle *pHandle, const char outputMaskStr[256]);
fcns.name{fcnNum}='sbgMatSetDefaultOutputMask'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetDefaultOutputMask(SbgMatLabHandle *pHandle, char outputMaskStr[256]);
fcns.name{fcnNum}='sbgMatLabGetDefaultOutputMask'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetTriggeredMode(SbgMatLabHandle *pHandle, uint8 condId, const char triggerMaskStr[256], const char outputMaskStr[256]);
fcns.name{fcnNum}='sbgMatLabSetTriggeredMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'cstring', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetTriggeredMode(SbgMatLabHandle *pHandle, uint8 condId, char triggerMaskStr[256], char outputMaskStr[256]);
fcns.name{fcnNum}='sbgMatLabGetTriggeredMode'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'stringPtr', 'stringPtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Calibration commands                                               -//
%----------------------------------------------------------------------//

% SbgErrorCode SBG_API sbgMatLabCalibMagnetometers(SbgMatLabHandle *pHandle, const char argumentStr[256]);
fcns.name{fcnNum}='sbgMatLabCalibMagnetometers'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabCalibMagnetometersSetTransformations(SbgMatLabHandle *pHandle, const float offset[3], const float crossAxis[9]);
fcns.name{fcnNum}='sbgMatLabCalibMagnetometersSetTransformations'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr', 'singlePtr'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabCalibMagnetometersGetTransformations(SbgMatLabHandle *pHandle, float offset[3], float crossAxis[9]);
fcns.name{fcnNum}='sbgMatLabCalibMagnetometersGetTransformations'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabCalibGyroBias(SbgMatLabHandle *pHandle, const char argumentStr[256]);
fcns.name{fcnNum}='sbgMatLabCalibGyroBias'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%-  Kalman Filter commands                                            -//
%----------------------------------------------------------------------//

% SbgErrorCode SBG_API sbgMatLabSetFilterAttitudeOptions(SbgMatLabHandle *pHandle, const char filterOptionsStr[256]);
fcns.name{fcnNum}='sbgMatLabSetFilterAttitudeOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetFilterAttitudeOptions(SbgMatLabHandle *pHandle, char filterOptionsStr[256]);
fcns.name{fcnNum}='sbgMatLabGetFilterAttitudeOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetFilterFrequencies(SbgMatLabHandle *pHandle, float gyroAccelsSampling, float cutoffGyro, float cutoffAccel, float cutoffMagneto, float mainLoopFrequency);
fcns.name{fcnNum}='sbgMatLabSetFilterFrequencies'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'single', 'single', 'single', 'single', 'single'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetFilterFrequencies(SbgMatLabHandle *pHandle, float *pGyroAccelsSampling, float *pCutoffGyro, float *pCutoffAccel, float *pCutoffMagneto, float *pMainLoopFrequency);
fcns.name{fcnNum}='sbgMatLabGetFilterFrequencies'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr', 'singlePtr', 'singlePtr', 'singlePtr', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetFilterHeadingSource(SbgMatLabHandle *pHandle, const char sourceStr[256]);
fcns.name{fcnNum}='sbgMatLabSetFilterHeadingSource'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetFilterHeadingSource(SbgMatLabHandle *pHandle, char sourceStr[256]);
fcns.name{fcnNum}='sbgMatLabGetFilterHeadingSource'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetMagneticDeclination(SbgMatLabHandle *pHandle, float declination);
fcns.name{fcnNum}='sbgMatLabSetMagneticDeclination'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'single'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetMagneticDeclination(SbgMatLabHandle *pHandle, float *pDeclination);
fcns.name{fcnNum}='sbgMatLabGetMagneticDeclination'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSendFilterHeading(SbgMatLabHandle *pHandle, float heading, float accuracy);
fcns.name{fcnNum}='sbgMatLabSendFilterHeading'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'single', 'single'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetHeaveConf(SbgMatLabHandle *pHandle, bool enableHeave);
fcns.name{fcnNum}='sbgMatLabSetHeaveConf'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabGetHeaveConf(SbgMatLabHandle *pHandle, bool *pEnableHeave);
fcns.name{fcnNum}='sbgMatLabGetHeaveConf'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8Ptr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%-  Orientation commands                                              -//
%----------------------------------------------------------------------//

% SbgErrorCode SBG_API sbgMatLabSetManualOrientationOffset(SbgMatLabHandle *pHandle, const char offsetTypeStr[256], const float rotationMatrix[9]);
fcns.name{fcnNum}='sbgMatLabSetManualOrientationOffset'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring', 'singlePtr'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetOrientationOffset(SbgMatLabHandle *pHandle, const char offsetTypeStr[256], float rotationMatrix[9]);
fcns.name{fcnNum}='sbgMatLabGetOrientationOffset'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetAutoOrientationOffset(SbgMatLabHandle *pHandle, const char offsetTypeStr[256]);
fcns.name{fcnNum}='sbgMatLabSetAutoOrientationOffset'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%-  Navigation commands                                               -//
%----------------------------------------------------------------------//

% SbgErrorCode sbgMatLabSetReferencePressure(SbgMatLabHandle *pHandle, uint32 reference);
fcns.name{fcnNum}='sbgMatLabSetReferencePressure'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetReferencePressure(SbgMatLabHandle *pHandle, uint32 *pReference);
fcns.name{fcnNum}='sbgMatLabGetReferencePressure'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint32Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetGpsOptions(SbgMatLabHandle *pHandle, const char modelStr[256], const char optionsStr[256]);
fcns.name{fcnNum}='sbgMatLabSetGpsOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetGpsOptions(SbgMatLabHandle *pHandle, char modelStr[256], char optionsStr[256]);
fcns.name{fcnNum}='sbgMatLabGetGpsOptions'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetNavVelocitySrc(SbgMatLabHandle *pHandle, const char aidingSrc[256]);
fcns.name{fcnNum}='sbgMatLabSetNavVelocitySrc'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetNavVelocitySrc(SbgMatLabHandle *pHandle, char aidingSrcStr[256]);
fcns.name{fcnNum}='sbgMatLabGetNavVelocitySrc'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetNavPositionSrc(SbgMatLabHandle *pHandle, const char aidingSrc[256]);
fcns.name{fcnNum}='sbgMatLabSetNavPositionSrc'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetNavPositionSrc(SbgMatLabHandle *pHandle, char aidingSrcStr[256]);
fcns.name{fcnNum}='sbgMatLabGetNavPositionSrc'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetGpsLeverArm(SbgMatLabHandle *pHandle, const float gpsLeverArm[3]);
fcns.name{fcnNum}='sbgMatLabSetGpsLeverArm'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetGpsLeverArm(SbgMatLabHandle *pHandle, float gpsLeverArm[3]);
fcns.name{fcnNum}='sbgMatLabGetGpsLeverArm'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetGravityMagnitude(SbgMatLabHandle *pHandle, float gravityMagnitude);
fcns.name{fcnNum}='sbgMatLabSetGravityMagnitude'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'single'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetGravityMagnitude(SbgMatLabHandle *pHandle, float *pGravityMagnitude);
fcns.name{fcnNum}='sbgMatLabGetGravityMagnitude'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSendNavVelocity(SbgMatLabHandle *pHandle, const float velocity[3], float accuracy);
fcns.name{fcnNum}='sbgMatLabSendNavVelocity'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr', 'single'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabSendNavPosition(SbgMatLabHandle *pHandle, const double position[3], float hAccuracy, float vAccuracy);
fcns.name{fcnNum}='sbgMatLabSendNavPosition'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'doublePtr', 'single', 'single'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Odometer configurations                                            -//
%----------------------------------------------------------------------//

% SbgErrorCode SBG_API sbgMatLabSetOdoConfig(SbgMatLabHandle *pHandle, uint8 channel, const char axisStr[256], float pulsesPerMeter, uint8 gainError, bool gpsGainCorrection);
fcns.name{fcnNum}='sbgMatLabSetOdoConfig'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'cstring', 'single', 'uint8', 'uint8'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetOdoConfig(SbgMatLabHandle *pHandle, uint8 channel, char axisStr[256], float *pPulsesPerMeter, uint8 *pGainError, bool *pGpsGainCorrection);
fcns.name{fcnNum}='sbgMatLabGetOdoConfig'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'stringPtr', 'singlePtr', 'uint8Ptr', 'uint8Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetOdoDirection(SbgMatLabHandle *pHandle, uint8 channel, const char odoDirectionStr[256]);
fcns.name{fcnNum}='sbgMatLabSetOdoDirection'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'cstring'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetOdoDirection(SbgMatLabHandle *pHandle, uint8 channel, char odoDirectionStr[256]);
fcns.name{fcnNum}='sbgMatLabGetOdoDirection'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'stringPtr'};fcnNum=fcnNum+1;

% SbgErrorCode SBG_API sbgMatLabSetOdoLeverArm(SbgMatLabHandle *pHandle, uint8 channel, const float odoLeverArm[3]);
fcns.name{fcnNum}='sbgMatLabSetOdoLeverArm'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'singlePtr'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetOdoLeverArm(SbgMatLabHandle *pHandle, uint8 channel, float odoLeverArm[3]);
fcns.name{fcnNum}='sbgMatLabGetOdoLeverArm'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'singlePtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Synchronization input and output operations                        -//
%----------------------------------------------------------------------//

% SbgErrorCode sbgMatLabSetLogicInChannel(SbgMatLabHandle *pHandle, uint8 channel, const char inputTypeStr[256], const char sensitivityStr[256], const char locationStr[256], int32 nsDelay);
fcns.name{fcnNum}='sbgMatLabSetLogicInChannel'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'cstring', 'cstring', 'cstring', 'int32'};fcnNum=fcnNum+1;
% SbgErrorCode sbgMatLabGetLogicInChannel(SbgMatLabHandle *pHandle, uint8 channel, char inputTypeStr[256], char sensitivityStr[256], char locationStr[256], int32 *pNsDelay);
fcns.name{fcnNum}='sbgMatLabGetLogicInChannel'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'stringPtr', 'stringPtr', 'stringPtr', 'int32Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetLogicOutChannel(SbgMatLabHandle *pHandle, uint8 channel, const char outputTypeStr[256], const char polarityStr[256], uint8 duration);
fcns.name{fcnNum}='sbgMatLabSetLogicOutChannel'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'cstring', 'cstring', 'uint8'};fcnNum=fcnNum+1;
% SbgErrorCode SBG_API sbgMatLabGetLogicOutChannel(SbgMatLabHandle *pHandle, uint8 channel, char outputTypeStr[256], char polarityStr[256], uint8 *pDuration);
fcns.name{fcnNum}='sbgMatLabGetLogicOutChannel'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'uint8', 'stringPtr', 'stringPtr', 'uint8Ptr'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabSetVirtualOdoConf(SbgMatLabHandle *pHandle, float distance);
fcns.name{fcnNum}='sbgMatLabSetVirtualOdoConf'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'single'};fcnNum=fcnNum+1;

% SbgErrorCode sbgMatLabGetVirtualOdoConf(SbgMatLabHandle *pHandle, float *pDistance);
fcns.name{fcnNum}='sbgMatLabGetVirtualOdoConf'; fcns.calltype{fcnNum}='cdecl'; fcns.LHS{fcnNum}='uint32'; fcns.RHS{fcnNum}={'voidPtr', 'singlePtr'};fcnNum=fcnNum+1;

%----------------------------------------------------------------------//
%- Structures definitions                                             -//
%----------------------------------------------------------------------//

structs.s_SbgMatLabData.members=struct('q0', 'single', 'q1', 'single', 'q2', 'single', 'q3', 'single', 'roll', 'single', 'pitch', 'single', 'yaw', 'single', 'ma0', 'single', 'ma1', 'single', 'ma2', 'single', 'ma3', 'single', 'ma4', 'single', 'ma5', 'single', 'ma6', 'single', 'ma7', 'single', 'ma8', 'single', 'g0', 'single', 'g1', 'single', 'g2', 'single', 'a0', 'single', 'a1', 'single', 'a2', 'single', 'm0', 'single', 'm1', 'single', 'm2', 'single', 't0', 'single', 't1', 'single', 'gT0', 'single', 'gT1', 'single', 'gT2', 'single', 'gR0', 'uint16', 'gR1', 'uint16', 'gR2', 'uint16', 'aR0', 'uint16', 'aR1', 'uint16', 'aR2', 'uint16', 'mR0', 'uint16', 'mR1', 'uint16', 'mR2', 'uint16', 'tR0', 'uint16', 'tR1', 'uint16', 'gTR0', 'uint16', 'gTR1', 'uint16', 'gTR2', 'uint16', 'timeSinceReset', 'uint32', 'deviceStatus', 'single', 'gpsLatitude', 'double', 'gpsLongitude', 'double', 'gpsAltitude', 'int32', 'gpsV0', 'int32', 'gpsV1', 'int32', 'gpsV2', 'int32', 'gpsHeading', 'single', 'gpsHorAccuracy', 'uint32', 'gpsVertAccuracy', 'uint32', 'gpsSpeedAccuracy', 'uint32', 'gpsHeadingAccuracy', 'single', 'gpsTimeMs', 'uint32', 'gpsFlags', 'uint8', 'gpsNbSats', 'uint8', 'gpsTrueHeading', 'single', 'gpsTrueHeadingAccuracy', 'single', 'baroPressure', 'single', 'baroAltitude', 'single', 'p0', 'double', 'p1', 'double', 'p2', 'double', 'v0', 'single', 'v1', 'single', 'v2', 'single', 'attitudeAccuracy', 'single', 'positionAccuracy', 'single', 'velocityAccuracy', 'single', 'utcYear', 'uint8', 'utcMonth', 'uint8', 'utcDay', 'uint8', 'utcHour', 'uint8', 'utcMin', 'uint8', 'utcSec', 'uint8', 'utcNano', 'uint32', 'magCalibData0', 'uint32', 'magCalibData1', 'uint32', 'magCalibData2', 'uint32', 'odoRawVel0', 'single', 'odoRawVel1', 'single', 'deltaAngleX', 'single', 'deltaAngleY', 'single', 'deltaAngleZ', 'single', 'heave', 'single');
enuminfo.e_SbgErrorCode=struct('SBG_NO_ERROR',0,'SBG_ERROR',1,'SBG_NULL_POINTER',2,'SBG_INVALID_CRC',3,'SBG_INVALID_FRAME',4,'SBG_TIME_OUT',5,'SBG_WRITE_ERROR',6,'SBG_READ_ERROR',7,'SBG_BUFFER_OVERFLOW',8,'SBG_INVALID_PARAMETER',9,'SBG_NOT_READY',10,'SBG_MALLOC_FAILED',11,'SGB_CALIB_MAG_NOT_ENOUGH_POINTS',12,'SBG_CALIB_MAG_INVALID_TAKE',13,'SBG_CALIB_MAG_SATURATION',14,'SBG_CALIB_MAG_POINTS_NOT_IN_A_PLANE',15,'SBG_DEVICE_NOT_FOUND',16,'SBG_OPERATION_CANCELLED',17,'SBG_NOT_CONTINUOUS_FRAME',18,'SBG_INCOMPATIBLE_HARDWARE',19);
methodinfo=fcns;