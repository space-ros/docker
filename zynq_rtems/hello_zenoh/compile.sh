#!/bin/bash
APP_NAME=hello_zenoh
if [ $(basename $PWD) != $APP_NAME ]; then
  echo "This script must be run in the '$APP_NAME' directory."
  exit
fi
if [ $(id -u) -ne 0 ]; then
  echo "Non-root user detected. Re-running myself inside docker..."
  set -o verbose
  docker run --rm --net=host -e DISPLAY=$DISPLAY \
    --device=/dev/dri:/dev/dri \
    --volume=$PWD:/root/$APP_NAME \
    -w /root/$APP_NAME \
    openrobotics/zynq_rtems:latest \
    /root/$APP_NAME/compile.sh
  exit
fi
set -o errexit
set -o verbose
./waf configure --rtems=/root/rtems/6 --rtems-archs=aarch64 --rtems-version=6 --show-commands
./waf
