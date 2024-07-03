#!/bin/sh
# This script is used to build the ig500IronCalibExample on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500IronCalibExample.c  -I../../../sbgIronCalibration/src/ -I../../../sbgCom/src/ ../../../sbgIronCalibration/libSbgIronCalibration.a ../../../sbgCom/libSbgComSerial.a -o ../../ig500IronCalibExample
