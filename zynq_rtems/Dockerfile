FROM ubuntu:22.04
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
        python2 \
        python2-dev \
	python-is-python3 \
	python3 \
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

RUN locale  # check for UTF-8
RUN apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get install curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt install -y \
	python3-colcon-common-extensions \
	python3-lark \
	libyaml-cpp-dev
#RUN apt install -y libgtest-dev

# build CycloneDDS for its IDL generator
# WORKDIR /root
# RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds
# #RUN mkdir build
# WORKDIR /root/cyclonedds/build
# RUN cmake -DBUILD_EXAMPLES=NO -DBUILD_TESTING=NO -DBUILD_IDLC=YES -DENABLE_IPV6=NO -DBUILD_SHARED_LIBS=NO -DENABLE_SECURITY=NO ..
# #cmake -DCMAKE_TOOLCHAIN_FILE=/root/toolchain.cmake -DBUILD_EXAMPLES=NO -DBUILD_TESTING=NO -DBUILD_IDLC=YES -DENABLE_IPV6=NO -DBUILD_SHARED_LIBS=NO -DENABLE_SSL=NO -DENABLE_SECURITY=NO ..
# ENV VERBOSE 1
# RUN make -j4 && make install
#RUN apt install libasio-dev
