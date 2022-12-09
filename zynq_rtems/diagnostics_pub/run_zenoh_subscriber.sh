#!/bin/bash
APP_NAME=diagnostics_pub
echo $PWD
if [ $(basename $PWD) != $APP_NAME ]; then
  echo "This script must be run in the '$APP_NAME' directory."
  exit
fi
set -o errexit
set -o verbose
docker run --rm -e DISPLAY=$DISPLAY \
  --init \
  --device=/dev/dri:/dev/dri \
  --device=/dev/net/tun --cap-add=NET_ADMIN \
  --env RUST_LOG=debug \
  --network=host \
  -w /root/zenoh \
  openrobotics/zynq_rtems:latest \
  cargo run --example z_sub -- --key rt/diagnostics
