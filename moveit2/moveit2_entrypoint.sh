#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/root/src/moveit2_ws/install/setup.bash"
exec "$@"
