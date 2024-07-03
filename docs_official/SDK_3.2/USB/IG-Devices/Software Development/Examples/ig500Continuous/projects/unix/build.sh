#!/bin/sh
# This script is used to build the ig500Continuous on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500Continuous.c  -I../../../../sbgCom/src/ ../../../../sbgCom/libSbgComSerial.a -o ../../ig500Continuous

