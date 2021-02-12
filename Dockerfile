# syntax = docker/dockerfile:experimental
FROM ubuntu:18.04

WORKDIR /app

ARG DEBIAN_FRONTEND=noninteractive

# APT packages
RUN apt update && apt install -y --no-install-recommends git ninja-build gperf \
    ccache dfu-util device-tree-compiler wget \
    python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
    make gcc gcc-multilib g++-multilib libsdl2-dev curl

# Install recent CMake
RUN mkdir -p /opt/cmake && \
    cd /opt/cmake && \
    wget https://github.com/Kitware/CMake/releases/download/v3.17.2/cmake-3.17.2-Linux-x86_64.sh && \
    sh cmake-3.17.2-Linux-x86_64.sh --prefix=/opt/cmake --skip-license && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake

# GCC ARM
RUN cd .. && \
    wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    tar xjf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 && \
    echo "PATH=$PATH:/gcc-arm-none-eabi-9-2019-q4-major/bin" >> ~/.bashrc && \
    cd /app

ENV PATH="/gcc-arm-none-eabi-9-2019-q4-major/bin:${PATH}"
ENV ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
ENV GNUARMEMB_TOOLCHAIN_PATH="/gcc-arm-none-eabi-9-2019-q4-major"

# Newest version of pip
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3 get-pip.py

# Install west and the nRF Connect SDK
RUN pip3 install west
RUN mkdir /ncs
RUN cd /ncs && west init -m https://github.com/nrfconnect/sdk-nrf --mr 38b7dd13ce1a8e9b8b84808d9ad56360dd35e0a0
RUN cd /ncs && west update
RUN cd /ncs && west zephyr-export
RUN pip3 install -r /ncs/zephyr/scripts/requirements.txt

ENV ZEPHYR_BASE="/ncs/zephyr"
