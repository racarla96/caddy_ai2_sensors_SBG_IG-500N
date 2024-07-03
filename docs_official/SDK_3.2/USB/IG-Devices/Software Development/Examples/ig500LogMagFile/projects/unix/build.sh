#!/bin/sh
# This script is used to build the ig500LogMagFile on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500LogMagFile.c  -I../../../../sbgCom/src/ ../../../../sbgCom/libSbgComSerial.a -o ../../ig500LogMagFile

