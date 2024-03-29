FROM ubuntu:20.04
WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update \
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
RUN git clone https://github.com/leethomason/tinyxml2.git

COPY example example/
COPY hello hello/
COPY hello_posix hello_posix/
COPY tinyxml2 tinyxml2/

WORKDIR /root/example/build
RUN cmake ../src/
RUN cmake --build .

WORKDIR /root/tinyxml2

RUN echo 'alias sparc-rtems5-gcc="~/development/rtems/5/bin/sparc-rtems5-gcc"' >> ~/.bashrc
RUN echo 'alias sparc-rtems5-objdump="~/development/rtems/5/bin/sparc-rtems5-objdump"' >> ~/.bashrc

#sparc-rtems5-gcc -Wall -lc --entry main hello.c -o hello
