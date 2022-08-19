#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/root/src/moveit2/install/setup.bash"
exec "$@"
