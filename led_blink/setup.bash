#!/bin/bash
# sudo apt install -y build-essential make cmake g++ gcc-arm-none-eabi binutils-arm-none-eabi doxygen libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib python3 ninja-build git

echo "export PICO_TOOLCHAIN_PATH=/usr" >> ~/.bashrc
echo "export PICO_SDK_PATH=~/pico-sdk" >> ~/.bashrc
echo "export FREERTOS_KERNEL_PATH=~/FreeRTOS-Kernel" >> ~/.bashrc

source ~/.bashrc
