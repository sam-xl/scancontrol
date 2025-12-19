ARG SCANCONTROL_SDK_VERSION=1.0.0

ARG UBUNTU_VERSION=24.04
ARG ROS_DISTRO=jazzy

FROM ubuntu:${UBUNTU_VERSION} AS build_stage

ARG SCANCONTROL_SDK_VERSION
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        checkinstall \
        gcc \
        g++ \
        libaravis-dev \
        libusb-1.0-0-dev \
        meson \
        unzip \
        wget \
    && rm -rf /var/lib/apt/lists/* && \
    # Ensure CA certificates are up to date (wget fails otherwise)
    update-ca-certificates

SHELL ["/bin/bash", "-c"]
RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -O scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    unzip scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -d scanCONTROL-Linux-SDK/ && \
    rm scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip

# Fix for libllt build errors
WORKDIR /scanCONTROL-Linux-SDK/libllt/
RUN sed -i '27,29 s/^/\/\/ /; 31,33 s/^/\/\/ /;' include/llt.h

WORKDIR /scanCONTROL-Linux-SDK/libmescan/
RUN meson setup build && \
    cd build && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion ${SCANCONTROL_SDK_VERSION} --default --fstrans=no ninja install

WORKDIR /scanCONTROL-Linux-SDK/libllt/
RUN meson setup build && \
    cd build && \
    ninja && \
    checkinstall --pkgname llt --pkgversion ${SCANCONTROL_SDK_VERSION} --default --fstrans=no ninja install

RUN mkdir /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libmescan/build/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libllt/build/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs

FROM ros:${ROS_DISTRO}

ARG SCANCONTROL_SDK_VERSION

RUN apt-get update && apt-get install -y --no-install-recommends\
      intltool \
      libaravis-dev \
      libusb-1.0-0 \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=bind,from=build_stage,source=/library_pkgs,target=/library_pkgs \
    apt-get update && \
    apt-get install -y --no-install-recommends /library_pkgs/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb && \
    apt-get install -y --no-install-recommends /library_pkgs/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb \
    && rm -rf /var/lib/apt/lists/*
