#!/bin/bash
set -e

# Setup the MoveIt2 Tutorials environment
source "/home/spaceros-user/src/moveit2_tutorials/install/setup.bash"
exec "$@"
