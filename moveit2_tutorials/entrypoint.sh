#!/bin/bash
set -e

# Setup the MoveIt2 Tutorials environment
source "/home/spaceros-user/moveit2_tutorials/install/setup.bash"
exec "$@"
