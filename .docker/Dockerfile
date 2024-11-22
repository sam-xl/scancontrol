ARG SCANCONTROL_SDK_VERSION=1.0.0

################################################################################
# Build .deb packages for Aravis and scanCONTROL SDK
################################################################################
FROM ubuntu:22.04 AS build

ARG SCANCONTROL_SDK_VERSION
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      checkinstall \
      cmake \
      debhelper \
      g++ \
      gcc \
      gettext \
      git \
      gobject-introspection \
      gtk-doc-tools \
      intltool \
      libgirepository1.0-dev \
      libglib2.0-dev \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      libgstreamer-plugins-good1.0-dev \
      libgtk-3-dev \
      libusb-1.0-0-dev \
      libxml2-dev \
      python3-pip \
      ninja-build \
      pkg-config \
      unzip \
      wget \
      xsltproc \
      meson \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/AravisProject/aravis/releases/download/0.8.30/aravis-0.8.30.tar.xz -O aravis-0.8.30.tar.xz &&\
    tar xfJ aravis-0.8.30.tar.xz && \
    rm aravis-0.8.30.tar.xz && \
    cd aravis-0.8.30 && \
    meson setup build && \
    cd build && \
    ninja && \
    checkinstall --pkgname aravis --pkgversion 0.8.30 --requires="libglib2.0-dev, libxml2-dev, libusb-1.0-0-dev" ninja install

SHELL ["/bin/bash", "-c"]
RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -O scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    unzip scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -d scanCONTROL-Linux-SDK/ && \
    rm scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    cd scanCONTROL-Linux-SDK/libmescan/ && \ 
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="aravis \(\>= 0.8.0\)" ninja install && \
    ldconfig && \
    cd /scanCONTROL-Linux-SDK/libllt/include && \
    # scanCONTROL SDK fix for libglib 
    sed -i '27,29 s/^/\/\/ /; 31,33 s/^/\/\/ /;' llt.h && \
    cd /scanCONTROL-Linux-SDK/libllt && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname llt --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="mescan \(\>= ${SCANCONTROL_SDK_VERSION}\),aravis \(\>= 0.8.0\)" ninja install

RUN mkdir /library_pkgs && \
    mv /aravis-0.8.30/build/aravis_0.8.30-1_amd64.deb /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libmescan/builddir/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libllt/builddir/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs

################################################################################
# ros-core and Aravis + scanCONTROL SDK
#   for use with CI
################################################################################
FROM ros:humble-ros-core AS scancontrol-core
ARG SCANCONTROL_SDK_VERSION

RUN apt-get update && apt-get install -y --no-install-recommends\
      build-essential \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build ["/library_pkgs", "/library_pkgs"]

RUN apt-get update && \
    apt install -y /library_pkgs/aravis_0.8.30-1_amd64.deb && \
    apt install /library_pkgs/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb && \
    apt install /library_pkgs/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb \
    && rm -rf /var/lib/apt/lists/*

################################################################################
# ros-base + Aravis + scanCONTROL SDK + RViz2
#   for use as a base image in development or deployment
################################################################################
FROM scancontrol-core AS scancontrol-base

ARG DEBIAN_FRONTEND=noninteractive

# Install the base release + rviz2
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-rviz2 \
      ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*
