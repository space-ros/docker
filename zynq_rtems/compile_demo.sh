#!/bin/bash
set -o errexit
set -o verbose
docker run --rm --net=host -e DISPLAY=$DISPLAY \
  --device=/dev/dri:/dev/dri \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/hello_network:/root/hello_network" \
  -w /root/hello_network \
  openrobotics/zynq_rtems:latest \
  /root/hello_network/compile.sh
