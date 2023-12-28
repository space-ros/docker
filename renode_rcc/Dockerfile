FROM ubuntu:20.04
WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

# Using Docker BuildKit cache mounts for /var/cache/apt and /var/lib/apt ensures that
# the cache won't make it into the built image but will be maintained between steps.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update -y && \
    apt-get install -y automake \
    autoconf \
    ca-certificates \
    git \
    make \
    netcat \
    tree \
    vim \
    wget \
    xterm \
    xz-utils

ARG RENODE_VERSION=1.13.0

USER root
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    wget https://github.com/renode/renode/releases/download/v${RENODE_VERSION}/renode_${RENODE_VERSION}_amd64.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends ./renode_${RENODE_VERSION}_amd64.deb python3-dev && \
    rm ./renode_${RENODE_VERSION}_amd64.deb
RUN pip3 install -r /opt/renode/tests/requirements.txt --no-cache-dir
RUN git clone https://github.com/antmicro/renode-rtems-leon3.git
COPY renode-rtems-leon3/ /root/renode-rtems-leon3/

WORKDIR /root/renode-rtems-leon3
RUN ./build-rtems.sh
RUN ./build-prom.sh
RUN mv rcc-1.3.0-gcc/ /opt/renode/
RUN mv grlib-gpl-2021.2-b4267 /opt/renode/

ENV PATH $PATH:/opt/renode/rcc-1.3.0-gcc/bin

WORKDIR /root
COPY config .config/renode/

#renode
#s @renode-rtems-leon3/leon3_rtems.resc
