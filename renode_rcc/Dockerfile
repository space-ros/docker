FROM ubuntu:20.04
WORKDIR /root
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update -y && \
    apt-get install -y git \
    xz-utils \
    make \
    vim \
    ca-certificates \
    wget \
    automake \
    autoconf \
    netcat \
    tree

ARG RENODE_VERSION=1.13.0

USER root
RUN wget https://github.com/renode/renode/releases/download/v${RENODE_VERSION}/renode_${RENODE_VERSION}_amd64.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends ./renode_${RENODE_VERSION}_amd64.deb python3-dev && \
    rm ./renode_${RENODE_VERSION}_amd64.deb
RUN pip3 install -r /opt/renode/tests/requirements.txt --no-cache-dir
RUN git clone https://github.com/tonylitianyu/renode-rtems-leon3.git


WORKDIR /root/renode-rtems-leon3
RUN ./build-rtems.sh
RUN ./build-prom.sh
RUN mv rcc-1.3.0-gcc/ /opt/renode/
RUN mv grlib-gpl-2021.2-b4267 /opt/renode/

ENV PATH $PATH:/opt/renode/rcc-1.3.0-gcc/bin

WORKDIR /root

#renode
#s @renode-rtems-leon3/leon3_rtems.resc
