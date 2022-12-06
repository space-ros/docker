#!/bin/bash
set -o errexit
set -o verbose
docker run --rm --net=host -e DISPLAY=$DISPLAY \
  --device=/dev/dri:/dev/dri \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/hello_zenoh:/root/hello_zenoh" \
  -w /root/hello_zenoh \
  openrobotics/zynq_rtems:latest \
  /root/hello_zenoh/compile.sh
