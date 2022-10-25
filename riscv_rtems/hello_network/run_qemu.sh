#!/bin/bash
set -o verbose
set -o errexit
QEMU_PATH=$HOME/qemu/qemu-7.1.0/build/qemu-system-riscv64
sudo $QEMU_PATH  -M virt -kernel build/riscv-rtems6-rv64imafdc_medany/hello_network.exe -no-reboot -m 128M -bios none -nographic \
  -netdev tap,id=mynet0,ifname=tap0,script=no,downscript=no \
  -device virtio-net-device,netdev=mynet0,mac=00:11:22:33:44:55

  #-device e1000,netdev=mynet0,mac=00:11:22:33:44:55
  #-device virtio-net-device,netdev=mynet0,mac=00:11:22:33:44:55
  #-device e1000,netdev=mynet0
  #virtio-net-device,netdev=mynet0,mac=00:11:22:33:44:55
  #-net nic,model=e1000k \
  #-net tap,ifname=tap0,script=no,downscript=no
  #-device e1000,netdev=mynet0,mac=00:11:22:33:44:55
  #-net nic -net tap,ifname=tap0,script=no,downscript=no
#-netdev tap,id=tap0 -device tap,netdev=tap0
