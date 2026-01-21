FROM ubuntu:24.04 AS base-stage


# Avoid interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Install core dev tools
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    gdb \
    clang-format

# Install deps
RUN apt update && apt install -y \
    cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib


#! rpi-stage
FROM base-stage AS rpi-stage

WORKDIR /home/ubuntu

RUN echo "export PICO_TOOLCHAIN_PATH=/path" >> ~/.bashrc

# Cloning Pico SDK
RUN git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git
RUN echo "export PICO_SDK_PATH=/home/ubuntu/pico-sdk" >> ~/.bashrc

# Cloning the custom FreeRTOS SDK for RPi Pico2
RUN git clone --recurse-submodules https://github.com/raspberrypi/FreeRTOS-Kernel.git
RUN echo "export FREERTOS_KERNEL_PATH=/home/ubuntu/FreeRTOS-Kernel" >> ~/.bashrc



RUN rm -rf /var/lib/apt/lists/*


RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/' /home/ubuntu/.bashrc && \
    echo 'if [ -x /usr/bin/dircolors ]; then' >> /home/ubuntu/.bashrc && \
    echo '    eval "$(dircolors -b)"' >> /home/ubuntu/.bashrc && \
    echo '    alias ls="ls --color=auto"' >> /home/ubuntu/.bashrc && \
    echo '    alias grep="grep --color=auto"' >> /home/ubuntu/.bashrc && \
    echo '    alias fgrep="fgrep --color=auto"' >> /home/ubuntu/.bashrc && \
    echo '    alias egrep="egrep --color=auto"' >> /home/ubuntu/.bashrc && \
    echo 'fi' >> /home/ubuntu/.bashrc


# Switch to non-root user
USER ubuntu

# Set working directory
WORKDIR /home/ubuntu

# Default command: open a bash shell
CMD ["/bin/bash"]