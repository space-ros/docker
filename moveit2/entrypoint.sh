#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/home/spaceros-user/src/moveit2/install/setup.bash"
exec "$@"
