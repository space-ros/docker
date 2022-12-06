#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/home/spaceros-user/moveit2/install/setup.bash"
exec "$@"
