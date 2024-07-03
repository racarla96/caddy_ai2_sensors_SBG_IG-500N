#!/bin/sh
# This script is used to build the ironCalibExample on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/main.c  -I../../../sbgIronCalibration/src/ ../../../sbgIronCalibration/libSbgIronCalibration.a -o ../../ironCalibExample

