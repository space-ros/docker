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
  --device=/dev/dri:/dev/dri \
  --device=/dev/net/tun --cap-add=NET_ADMIN \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$PWD:/root/$APP_NAME" \
  --network=host \
  -w /root/$APP_NAME \
  openrobotics/zynq_rtems:latest \
  /root/qemu/qemu-7.1.0/build/qemu-system-aarch64 \
    -M xlnx-zcu102 \
    -m 2G \
    -no-reboot \
    -icount shift=1,align=off,sleep=on -rtc clock=vm \
    -nographic \
    -serial mon:stdio \
    -bios none \
    -net nic,model=cadence_gem -net nic,model=cadence_gem -net nic,model=cadence_gem \
    -net nic,model=cadence_gem,netdev=mynet0 \
    -netdev tap,id=mynet0,ifname=tap0,script=no,downscript=no \
    -kernel /root/$APP_NAME/build/aarch64-rtems6-xilinx_zynqmp_lp64_qemu/$APP_NAME.exe
