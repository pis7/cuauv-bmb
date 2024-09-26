#!/usr/bin/env bash

# Change this to the model number of the chip you are building for
MCU=atxmega32a4u

####### You shouldn't need to change anything below this line ########

# Traverse upwards in the directory tree to find the AVRCode folder
cd "${0%/*}"
initial_directory=${PWD}
while [[ "$PWD" != "/" ]] ; do
    if [[ -d "AVRCode" ]] ; then
        break
    fi
    cd ..
done

# If we didn't find one, print an error
if [[ ! -d "AVRCode" ]] ; then
    echo "Couldn't find an AVRCode directory?!?!?!"
    exit -1
fi

# Store the path to the AVRCode directory
cd AVRCode
AVRCODE_PATH=${PWD}

# Make a build directory, if it doesn't already exist
cd ${initial_directory}
mkdir -p build
cd build

# Initialize the build system
# Explanation of arguments:
# -DCMAKE_MODULE_PATH: Tells cmake that build system files can be found in the "cmake" directory
# -DAVR_MCU: Tells the build system what model of chip to build for
# -DCMAKE_TOOLCHAIN_FILE: Tells cmake what file describes the AVR toolchain. Will configure the compiler and linker properly, and emit a reasonable linker script
# -DCMAKE_BUILD_TYPE: The default build type (Release)
# ..: Use the CMakeLists.txt in this directory to drive the rest of the build
cmake -DCMAKE_MODULE_PATH=${AVRCODE_PATH}/cmake -DAVR_MCU=$MCU -DCMAKE_TOOLCHAIN_FILE=${AVRCODE_PATH}/cmake/gcc_avr.cmake -DCMAKE_BUILD_TYPE=Release ..
