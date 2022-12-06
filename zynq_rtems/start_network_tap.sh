#!/bin/bash
set -o verbose
set -o errexit
sudo ip tuntap add tap0 mode tap user $(whoami)
sudo ip link set dev tap0 up
sudo ip addr add 10.0.42.1/24 dev tap0
