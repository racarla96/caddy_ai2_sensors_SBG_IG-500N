#!/bin/sh
# This script is used to build the sbgIronCalibration library on unix systems.
# Example: ./build.sh

# Define default GCC options
gccOptions="-c -Wall"

# Create the intermediate directory
mkdir obj

# Create all objects
gcc $gccOptions ../../src/ironCalibMath.c -o obj/ironCalibMath.o
gcc $gccOptions ../../src/ironComputation.c -o obj/ironComputation.o
gcc $gccOptions ../../src/point.c -o obj/point.o
gcc $gccOptions ../../src/pointList.c -o obj/pointList.o
gcc $gccOptions ../../src/sbgIronCalibration.c -o obj/sbgIronCalibration.o

# Create the library
ar cr ../../libSbgIronCalibration.a obj/ironCalibMath.o obj/ironComputation.o obj/point.o obj/pointList.o obj/sbgIronCalibration.o

