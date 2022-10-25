#!/bin/bash
set -o verbose
set -o errexit
~/qemu/qemu-7.1.0/build/qemu-system-riscv64  -M virt -kernel build/riscv-rtems6-rv64imafdc_medany/hello_network.exe -no-reboot -net none -m 128M -bios none -nographic
