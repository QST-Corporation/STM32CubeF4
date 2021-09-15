#!/bin/bash

# Should add the ST-LINK_CLI.exe path to PATH environment variable
TOOL_PATH="/c/Program Files (x86)/STMicroelectronics/STM32 ST-LINK Utility/ST-LINK Utility"
SDK_PATH=$PWD
FW_PATH=Projects/STM32F429ZI-Nucleo/QST_Projects/QMI8658
FW_NAME=MDK-ARM/STM32F429ZI_NUCLEO/STM32F429ZI_NUCLEO.hex

echo "=============================================================================="
echo "                    QST Corporation Ltd."
echo "                  Firmware download script"
echo "                    Author: QST-0256"
echo "                      Version 0.0.1"
echo "=============================================================================="
echo "Starting to download firmware:$FW_PATH/$FW_NAME"

#ERASE:
#ST-LINK_CLI -ME

#FLASH:
ST-LINK_CLI -ME -P $SDK_PATH/$FW_PATH/$FW_NAME -V "after_programming" -Rst

echo "Firmware download is completed"
#cd -
exit 0
