#!/bin/sh
# This script is used to build the sbgCom library on unix systems.
# To compile the library, you have to specify the byte ordering.
# Example: ./build.sh SBG_PLATFORM_LITTLE_ENDIAN

# Test that we have the endianness argument
if [ $# -ne 1 ]; then
    echo "You have to specify the platform endianness using either SBG_PLATFORM_BIG_ENDIAN or SBG_PLATFORM_LITTLE_ENDIAN"
    exit 1
fi

# Test the first argument and define the GCC options according to the selected endianness
if [ "$1" = "SBG_PLATFORM_BIG_ENDIAN" ]; then
    # The platform is in big endian
    gccOptions="-c -Wall -D SBG_PLATFORM_BIG_ENDIAN"
elif [ "$1" = "SBG_PLATFORM_LITTLE_ENDIAN" ]; then
    # The platform is in little endian
    gccOptions="-c -Wall -D SBG_PLATFORM_LITTLE_ENDIAN"
else
    echo "You have entered an invalid argument"
    exit 1
fi

# Create the intermediate directory		
mkdir obj

# Create all objects
gcc $gccOptions ../../src/comWrapper/comSerialUnix.c -o obj/comSerialUnix.o
gcc $gccOptions ../../src/protocol/extDevices/extIg.c -o obj/extIg.o
gcc $gccOptions ../../src/protocol/extDevices/extNmea.c -o obj/extNmea.o
gcc $gccOptions ../../src/protocol/commands.c -o obj/commands.o
gcc $gccOptions ../../src/protocol/commandsCalib.c -o obj/commandsCalib.o
gcc $gccOptions ../../src/protocol/commandsExt.c -o obj/commandsExt.o
gcc $gccOptions ../../src/protocol/commandsFilter.c -o obj/commandsFilter.o
gcc $gccOptions ../../src/protocol/commandsIg30.c -o obj/commandsIg30.o
gcc $gccOptions ../../src/protocol/commandsNav.c -o obj/commandsNav.o
gcc $gccOptions ../../src/protocol/commandsOdo.c -o obj/commandsOdo.o
gcc $gccOptions ../../src/protocol/commandsOrientation.c -o obj/commandsOrientation.o
gcc $gccOptions ../../src/protocol/commandsOutput.c -o obj/commandsOutput.o
gcc $gccOptions ../../src/protocol/commandsSync.c -o obj/commandsSync.o
gcc $gccOptions ../../src/protocol/protocol.c -o obj/protocol.o
gcc $gccOptions ../../src/protocol/protocolOutput.c -o obj/protocolOutput.o
gcc $gccOptions ../../src/protocol/protocolOutputMode.c -o obj/protocolOutputMode.o
gcc $gccOptions ../../src/time/sbgTime.c -o obj/sbgTime.o
gcc $gccOptions ../../src/sbgCom.c -o obj/sbgCom.o

# Create the library
ar cr ../../libSbgComSerial.a obj/comSerialUnix.o obj/extIg.o obj/extNmea.o obj/commands.o obj/commandsCalib.o obj/commandsExt.o obj/commandsFilter.o obj/commandsIg30.o obj/commandsNav.o obj/commandsOdo.o obj/commandsOrientation.o obj/commandsOutput.o  obj/commandsSync.o obj/protocol.o obj/protocolOutput.o obj/protocolOutputMode.o obj/sbgTime.o  obj/sbgCom.o