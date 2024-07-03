#!/bin/sh
# This script is used to build the ig500MotionProfile on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500MotionProfile.c  -I../../../../sbgCom/src/ ../../../../sbgCom/libSbgComSerial.a -o ../../ig500MotionProfile

