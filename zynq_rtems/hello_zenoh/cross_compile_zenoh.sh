#!/bin/bash
set -o verbose
set -o errexit
mkdir -p zenoh_build
cd zenoh_build
cmake -DCMAKE_TOOLCHAIN_FILE=`pwd`/../toolchain.cmake -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTING=OFF ../../zenoh-pico
make
