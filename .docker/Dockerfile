ARG SCANCONTROL_SDK_VERSION=1.0.0

FROM ubuntu:focal AS build

ARG SCANCONTROL_SDK_VERSION
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      checkinstall \
      debhelper \
      g++ \
      gcc \
      git \
      intltool \
      python3-pip \
      ninja-build \
      pkg-config \
      unzip \
      wget \
      libxml2-dev libglib2.0-dev cmake libusb-1.0-0-dev gobject-introspection \
                 libgtk-3-dev gtk-doc-tools  xsltproc libgstreamer1.0-dev \
                 libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
                 libgirepository1.0-dev gettext \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install meson

RUN wget https://github.com/AravisProject/aravis/releases/download/0.8.30/aravis-0.8.30.tar.xz -O aravis-0.8.30.tar.xz &&\
    tar xfJ aravis-0.8.30.tar.xz && \
    rm aravis-0.8.30.tar.xz && \
    cd aravis-0.8.30 && \
    meson setup build && \
    cd build && \
    ninja && \
    checkinstall --pkgname aravis --pkgversion 0.8.30 ninja install

SHELL ["/bin/bash", "-c"]
RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -O scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    unzip scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip -d scanCONTROL-Linux-SDK/ && \
    rm scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    cd scanCONTROL-Linux-SDK/libmescan/ && \ 
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="aravis \(\>= 0.6.0\)" ninja install && \
    ldconfig && \
    cd ../../libllt && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname llt --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="mescan \(\>= ${SCANCONTROL_SDK_VERSION}\),aravis \(\>= 0.6.0\)" ninja install

RUN mkdir /library_pkgs && \
    mv /aravis-0.8.30/build/aravis_0.8.30-1_amd64.deb /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libmescan/builddir/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL-Linux-SDK/libllt/builddir/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs

FROM ros:noetic-ros-core
ARG SCANCONTROL_SDK_VERSION

RUN apt-get update && \
    apt-get install -y \
      intltool \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build ["/library_pkgs", "/library_pkgs"]

RUN apt-get update && \
    apt install -y /library_pkgs/aravis_0.8.30-1_amd64.deb && \
    apt install /library_pkgs/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb && \
    apt install /library_pkgs/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb
