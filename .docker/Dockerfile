FROM ubuntu:bionic AS build

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends wget meson git gcc g++ intltool pkg-config glib2.0 libxml2-dev unzip && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

RUN wget http://ftp.acc.umu.se/pub/GNOME/sources/aravis/0.6/aravis-0.6.4.tar.xz && \
    tar xfJ aravis-0.6.4.tar.xz && \
    rm aravis-0.6.4.tar.xz && \
    cd aravis-0.6.4 && \
    ./configure && \
    make && \
    make install && \
    cd .. && \
    rm -rf aravis-0.6.4/  && \
    ldconfig

RUN wget https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-0-2-4.zip && \
    unzip scanCONTROL-Linux-SDK-0-2-4.zip && \
    rm scanCONTROL-Linux-SDK-0-2-4.zip && \
    cd /scanCONTROL\ Linux\ SDK\ 0.2.4/libmescan/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    ninja install && \
    ldconfig && \
    cd /scanCONTROL\ Linux\ SDK\ 0.2.4/libllt/ && \
    meson builddir && \
    cd builddir && \
    ninja && \
    ninja install && \
    ldconfig

FROM ros:melodic-ros-core
ENV ROS_DISTRO melodic

RUN apt-get update && \
    apt-get upgrade -y && apt-get install ninja-build && \
    rm -rf /var/lib/apt/lists/*

COPY --from=build ["/scanCONTROL Linux SDK 0.2.4/", "/scanCONTROL Linux SDK 0.2.4/"]

RUN cd /scanCONTROL\ Linux\ SDK\ 0.2.4/libmescan/builddir && \
    ninja install && \
    cd /scanCONTROL\ Linux\ SDK\ 0.2.4/libllt/builddir && \
    ninja install && \
    cd && \
    rm -rf /scanCONTROL\ Linux\ SDK\ 0.2.4/ && \
    ldconfig 

COPY ./docker_entrypoint.sh /
ENTRYPOINT [ "/docker_entrypoint.sh" ]
CMD [ "bash" ]