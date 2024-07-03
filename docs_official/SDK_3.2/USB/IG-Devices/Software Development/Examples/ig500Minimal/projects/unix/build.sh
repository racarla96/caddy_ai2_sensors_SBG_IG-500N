#!/bin/sh
# This script is used to build the ig500Minimal on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ig500Minimal.c  -I../../../../sbgCom/src/ ../../../../sbgCom/libSbgComSerial.a -o ../../ig500Minimal

