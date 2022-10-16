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
        libtinfo-dev \
        pax \
        pkg-config \
        python \
        python-dev \
        texinfo \
        tree \
        u-boot-tools \
        unzip \
        vim \
        xz-utils \
        zlib1g-dev

RUN mkdir -p development/rtems/6
ENV PREFIX /root/development/rtems/6
ENV RTEMS_VERSION 6

RUN mkdir -p development/src
WORKDIR /root/development/src

RUN git clone git://git.rtems.org/rtems-source-builder.git rsb
RUN git clone git://git.rtems.org/rtems.git

WORKDIR /root/development/src/rsb/rtems
RUN ../source-builder/sb-set-builder --prefix=$PREFIX 6/rtems-riscv
RUN ../source-builder/sb-set-builder --prefix=$PREFIX --target=riscv-rtems6 --with-rtems-bsp=riscv/rv64imafdc_medany --with-rtems-tests=samples 6/rtems-kernel --with-rtems-bspopts="RISCV_MAXIMUM_EXTERNAL_INTERRUPTS=128"

WORKDIR /root
COPY hello hello/
COPY hello_posix hello_posix/
