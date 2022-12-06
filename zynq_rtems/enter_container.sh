#!/bin/sh
docker run --rm -e DISPLAY=$DISPLAY \
  --device=/dev/dri:/dev/dri \
  --device=/dev/net/tun --cap-add=NET_ADMIN \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD/hello_network:/root/hello_network" \
  --network=host \
  -w /root \
  -it \
  openrobotics/zynq_rtems:latest
