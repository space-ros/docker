#!/bin/bash
set -e

# Setup the Space ROS environment
source "install/setup.bash"
exec "$@"
