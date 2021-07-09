ARG SCANCONTROL_SDK_VERSION=0.2.5

FROM ubuntu:bionic AS build

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
      libglib2.0-dev \
      libxml2-dev \
      meson \
      pkg-config \
      unzip \
      wget \
    && rm -rf /var/lib/apt/lists/*

RUN wget http://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    tar xfJ aravis-0.6.4.tar.xz && \
    rm aravis-0.6.4.tar.xz && \
    cd aravis-0.6.4 && \
    ./configure && \
    make && \
    checkinstall --pkgname aravis --requires="libglib2.0-dev, libxml2-dev" && \
    ldconfig

SHELL ["/bin/bash", "-c"]
RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    unzip scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    rm scanCONTROL-Linux-SDK-${SCANCONTROL_SDK_VERSION//./-}.zip && \
    cd /scanCONTROL\ Linux\ SDK\ ${SCANCONTROL_SDK_VERSION}/libmescan/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="aravis \(\>= 0.6.0\)" ninja install && \
    ldconfig && \
    cd "/scanCONTROL Linux SDK ${SCANCONTROL_SDK_VERSION}/libllt/" && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname llt --pkgversion ${SCANCONTROL_SDK_VERSION} --requires="mescan \(\>= ${SCANCONTROL_SDK_VERSION}\),aravis \(\>= 0.6.0\)" ninja install

RUN mkdir /library_pkgs && \
    mv /aravis-0.6.4/aravis_0.6.4-1_amd64.deb /library_pkgs && \
    mv "/scanCONTROL Linux SDK ${SCANCONTROL_SDK_VERSION}/libmescan/builddir/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL Linux SDK ${SCANCONTROL_SDK_VERSION}/libllt/builddir/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb" /library_pkgs

FROM ros:melodic-ros-core
ARG SCANCONTROL_SDK_VERSION

RUN apt-get update && \
    apt-get install -y \
      intltool \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build ["/library_pkgs", "/library_pkgs"]

RUN apt-get update && \
    apt install -y /library_pkgs/aravis_0.6.4-1_amd64.deb && \
    apt install /library_pkgs/mescan_${SCANCONTROL_SDK_VERSION}-1_amd64.deb && \
    apt install /library_pkgs/llt_${SCANCONTROL_SDK_VERSION}-1_amd64.deb
