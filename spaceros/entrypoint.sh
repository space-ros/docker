#!/bin/bash
set -e

# Setup the Space ROS environment
source "/spaceros_ws/install/setup.bash"
exec "$@"
