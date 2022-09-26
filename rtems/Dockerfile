FROM ubuntu:20.04
WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
    && apt-get install -y \
        apt-utils build-essential \
        vim u-boot-tools git cmake \
        bison flex texinfo bzip2 \
        xz-utils unzip python \
        libexpat1-dev curl \
        python-dev \
        zlib1g-dev \
        libtinfo-dev \
        pax \
        tree


RUN mkdir -p development/rtems/5
ENV PREFIX /root/development/rtems/5
ENV RTEMS_VERSION 5

RUN mkdir -p development/src
WORKDIR /root/development/src

RUN curl https://ftp.rtems.org/pub/rtems/releases/5/5.1/sources/rtems-source-builder-5.1.tar.xz | tar xJf -
RUN mv rtems-source-builder-5.1/ rsb

WORKDIR /root/development/src/rsb/rtems

RUN ../source-builder/sb-set-builder --prefix=$PREFIX 5/rtems-sparc
RUN ../source-builder/sb-set-builder --prefix=$PREFIX --target=sparc-rtems5 --with-rtems-bsp=leon3 --with-rtems-tests=yes 5/rtems-kernel

WORKDIR /root

RUN git clone https://github.com/robamu-org/rtems-cmake.git

COPY example example/
COPY hello hello/
COPY tinyxml2 tinyxml2/

WORKDIR /root/example/build
RUN cmake ../src/
# RUN cmake --build .

WORKDIR /root

RUN echo 'alias sparc-rtems5-gcc="development/rtems/5/bin/sparc-rtems5-gcc"' >> ~/.bashrc

#sparc-rtems5-gcc -Wall -lc --entry main hello.c -o hello
