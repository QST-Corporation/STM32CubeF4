#!/bin/bash

# Should add the Keil UV4 path to PATH environment variable
export PATH="$PATH:/c/Keil_v5/UV4"
SDK_PATH=$PWD
FW_PRJ_PATH=Projects/STM32F429ZI-Nucleo/QST_Projects/QMI8658
FW_PRJ_NAME=MDK-ARM/Project.uvprojx


echo "=============================================================================="
echo "                   QST Corporation Ltd."
echo "                  Firmware build script"
echo "                    Author: QST-0256"
echo "                     Version 0.0.1"
echo "=============================================================================="
echo "INFO: Starting to build application: $FW_PRJ_PATH/$FW_PRJ_NAME"
echo "INFO: Prject location: $FW_PRJ_PATH"
echo "INFO: Building will take a while..."
UV4 -b -j0 "$SDK_PATH/$FW_PRJ_PATH/$FW_PRJ_NAME" -o $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log
#UV4 -cr -jo "$SDK_PATH/$FW_PRJ_PATH/$FW_PRJ_NAME" -o $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log
cat $SDK_PATH/$FW_PRJ_PATH/MDK-ARM/build.log