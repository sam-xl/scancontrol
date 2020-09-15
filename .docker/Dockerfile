FROM ubuntu:bionic AS build

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends checkinstall debhelper g++ gcc git glib2.0 intltool libxml2-dev meson pkg-config unzip wget && \
    rm -rf /var/lib/apt/lists/*

RUN wget http://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    tar xfJ aravis-0.6.4.tar.xz && \
    rm aravis-0.6.4.tar.xz && \
    cd aravis-0.6.4 && \
    ./configure && \
    make && \
    checkinstall --pkgname aravis  && \
    ldconfig

RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-0-2-4.zip && \
    unzip scanCONTROL-Linux-SDK-0-2-4.zip && \
    rm scanCONTROL-Linux-SDK-0-2-4.zip && \
    cd /scanCONTROL\ Linux\ SDK\ 0.2.4/libmescan/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname mescan --pkgversion 0.2.4 ninja install && \
    ldconfig && \
    cd "/scanCONTROL Linux SDK 0.2.4/libllt/" && \
    meson builddir && \
    cd builddir && \
    ninja && \
    checkinstall --pkgname llt --pkgversion 0.2.4 ninja install

RUN mkdir /library_pkgs && \
    mv /aravis-0.6.4/aravis_0.6.4-1_amd64.deb /library_pkgs && \
    mv "/scanCONTROL Linux SDK 0.2.4/libmescan/builddir/mescan_0.2.4-1_amd64.deb" /library_pkgs && \
    mv "/scanCONTROL Linux SDK 0.2.4/libllt/builddir/llt_0.2.4-1_amd64.deb" /library_pkgs

FROM ros:melodic-ros-core

RUN apt-get update && \
    apt-get install -y glib2.0 intltool libxml2-dev pkg-config && \
    rm -rf /var/lib/apt/lists/*

COPY --from=build ["/library_pkgs", "/library_pkgs"]

RUN apt install /library_pkgs/aravis_0.6.4-1_amd64.deb && \
    apt install /library_pkgs/mescan_0.2.4-1_amd64.deb && \
    apt install /library_pkgs/llt_0.2.4-1_amd64.deb

COPY ./docker_entrypoint.sh /
ENTRYPOINT [ "/docker_entrypoint.sh" ]
CMD [ "bash" ]