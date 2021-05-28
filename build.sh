#!/bin/bash

# Should add the Keil UV4 path to PATH environment variable
export PATH="$PATH:/c/Keil_v5/UV4"
SDK_PATH=$PWD
FW_PRJ_PATH=Projects/STM32F401RE-Nucleo/QST_Projects/blue_air_speed
FW_PRJ_NAME=MDK-ARM/Project.uvprojx


echo "=============================================================================="
echo "                 Shanghai QST Corporation"
echo "                  Firmware build script"
echo "                    Author: AE team"
echo "                     Version 0.0.1"
echo "=============================================================================="
echo "INFO: Starting to build application: $FW_PRJ_PATH/$FW_PRJ_NAME"
echo "INFO: Prject location: $FW_PRJ_PATH"
#rm $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log
echo "INFO: Building will take a while..."
UV4 -b -j0 "$SDK_PATH/$FW_PRJ_PATH/$FW_PRJ_NAME" -o $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log
#UV4 -cr -jo "$SDK_PATH/$FW_PRJ_PATH/$FW_PRJ_NAME" -o $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log
cat $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log