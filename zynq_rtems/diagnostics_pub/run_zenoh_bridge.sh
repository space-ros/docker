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
  --volume="$PWD:/root/$APP_NAME" \
  --env RUST_LOG=info \
  --network=host \
  -w /root \
  openrobotics/zynq_rtems:latest \
  /root/zenoh/target/release/zenohd -c /root/$APP_NAME/zenoh_config.json5
