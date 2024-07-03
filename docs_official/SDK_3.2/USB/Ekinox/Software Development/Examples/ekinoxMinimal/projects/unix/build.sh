#!/bin/sh
# This script is used to build the ekinoxMinimal example on unix systems.
# Example: ./build.sh

gcc -Wall ../../src/ekinoxMinimal.c  -I../../../../sbgECom/src/ ../../../../sbgECom/libSbgECom.a -o ../../ekinoxMinimal

