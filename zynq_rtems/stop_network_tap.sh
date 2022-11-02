#!/bin/bash
set -o verbose
set -o errexit
# make sure you have tunctl:
#   sudo apt install uml-utilities
sudo tunctl -d tap0
