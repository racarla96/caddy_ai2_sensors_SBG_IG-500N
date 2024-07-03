#!/bin/sh
# This script is used to build the ig500Trigger on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500Trigger.c  -I../../../../sbgCom/src/ ../../../../sbgCom/libSbgComSerial.a -o ../../ig500Trigger

