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
RUN git submodule init && git submodule update rtems_waf

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
RUN echo "[aarch64/xilinx_zynqmp_lp64_qemu]" > config.ini
RUN ./waf configure --prefix=$PREFIX
RUN ./waf
RUN ./waf install

# build the BSD kernel library for networking
WORKDIR /root/rtems/rtems-libbsd
RUN ./waf configure --prefix=$PREFIX --rtems-archs=aarch64 --rtems-bsps=aarch64/xilinx_zynqmp_lp64_qemu --buildset=buildset/default.ini
RUN ./waf
RUN ./waf install

# build Zenoh Pico, using a CMake toolchain file to cross-compile
WORKDIR /root
RUN git clone https://github.com/eclipse-zenoh/zenoh-pico
RUN mkdir /root/zenoh-pico/build
RUN wget -q https://raw.githubusercontent.com/space-ros/docker/zynq_rtems_zenoh_pico/zynq_rtems/toolchain.cmake -O /root/toolchain.cmake
WORKDIR /root/zenoh-pico/build
RUN cmake -DCMAKE_TOOLCHAIN_FILE=/root/toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTING=OFF ..
RUN make

# install Rust and Cargo, and add to the Docker build path
WORKDIR /root
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# build "normal" (non-Pico) Zenoh
WORKDIR /root
RUN git clone https://github.com/eclipse-zenoh/zenoh
WORKDIR /root/zenoh
RUN cargo build
