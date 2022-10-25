#!/bin/bash
set -o verbose
set -o errexit
# make sure you have tunctl:
#   sudo apt install uml-utilities
#sudo tunctl -u `id -u`
#sudo ifconfig tap0 10.0.42.1 netmask 255.255.255.0 up
sudo ip tuntap add tap0 mode tap user $(whoami)
sudo ip link set dev tap0 up
sudo ip addr add 10.0.42.1/24 dev tap0
