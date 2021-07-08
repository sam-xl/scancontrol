FROM ubuntu:bionic AS build

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

RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-0-2-5.zip && \
    unzip scanCONTROL-Linux-SDK-0-2-5.zip && \
    rm scanCONTROL-Linux-SDK-0-2-5.zip && \
    cd /scanCONTROL\ Linux\ SDK\ 0.2.5/libmescan/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion 0.2.5 --requires="aravis \(\>= 0.6.0\)" ninja install && \
    ldconfig && \
    cd "/scanCONTROL Linux SDK 0.2.5/libllt/" && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname llt --pkgversion 0.2.5 --requires="mescan \(\>= 0.2.5\),aravis \(\>= 0.6.0\)" ninja install

RUN mkdir /library_pkgs && \
    mv /aravis-0.6.4/aravis_0.6.4-1_amd64.deb /library_pkgs && \
    mv "/scanCONTROL Linux SDK 0.2.5/libmescan/builddir/mescan_0.2.5-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL Linux SDK 0.2.5/libllt/builddir/llt_0.2.5-1_amd64.deb" /library_pkgs

FROM ros:melodic-ros-core

RUN apt-get update && \
    apt-get install -y \
      intltool \
      pkg-config \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build ["/library_pkgs", "/library_pkgs"]

RUN apt-get update && \
    apt install -y /library_pkgs/aravis_0.6.4-1_amd64.deb && \
    apt install /library_pkgs/mescan_0.2.5-1_amd64.deb && \
    apt install /library_pkgs/llt_0.2.5-1_amd64.deb