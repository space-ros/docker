FROM ubuntu:20.04
WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get install -y \
        apt-utils \
        bison \
        build-essential \
        bzip2 \
        cmake \
        curl \
        flex \
        git \
        libexpat1-dev \
        libglib2.0-dev \
        libpixman-1-dev \
        libtinfo-dev \
        ninja-build \
        pax \
        pkg-config \
        python \
        python-dev \
        python3-dev \
        texinfo \
        tree \
        u-boot-tools \
        unzip \
        vim \
        wget \
        xz-utils \
        zlib1g-dev

RUN mkdir -p /root/rtems/6
ENV PREFIX /root/rtems/6
ENV RTEMS_VERSION 6

RUN mkdir -p /root/rtems
WORKDIR /root/rtems

RUN git clone git://git.rtems.org/rtems-source-builder.git rsb
RUN git clone git://git.rtems.org/rtems.git --depth 1

RUN git clone git://git.rtems.org/rtems-libbsd.git -b 6-freebsd-12
WORKDIR /root/rtems/rtems-libbsd
RUN git submodule init
RUN git submodule update rtems_waf

WORKDIR /root/qemu
RUN wget -q https://download.qemu.org/qemu-7.1.0.tar.xz
RUN tar xJf qemu-7.1.0.tar.xz
WORKDIR /root/qemu/qemu-7.1.0/build
RUN ../configure --target-list=aarch64-softmmu
RUN make -j8

# build the cross-compile toolchain
WORKDIR /root/rtems/rsb/rtems
RUN ../source-builder/sb-set-builder --prefix=$PREFIX 6/rtems-aarch64

# build the RTEMS kernel
WORKDIR /root/rtems/rtems
RUN echo -e "[aarch64/xilinx_zynqmp_lp64_qemu]" > config.ini
RUN ./waf configure --prefix=$PREFIX
RUN ./waf
RUN ./waf install

# build the BSD kernel library for networking
WORKDIR /root/rtems/rtems-libbsd
RUN ./waf configure --prefix=$PREFIX --rtems-archs=aarch64 --rtems-bsps=aarch64/xilinx_zynqmp_lp64_qemu --buildset=buildset/default.ini
RUN ./waf
RUN ./waf install

# build our application
WORKDIR /root
RUN git clone https://github.com/eclipse-zenoh/zenoh-pico
