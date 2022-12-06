#!/bin/sh
docker run --rm -e DISPLAY=$DISPLAY \
  --device=/dev/dri:/dev/dri \
  --device=/dev/net/tun --cap-add=NET_ADMIN \
  --volume="$PWD/hello_network:/root/hello_network" \
  --volume="$PWD/hello_zenoh:/root/hello_zenoh" \
  --network=host \
  -w /root \
  -it \
  openrobotics/zynq_rtems:latest
