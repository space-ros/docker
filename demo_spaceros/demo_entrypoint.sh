#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/root/src/spaceros_demo_ws/install/setup.bash"
exec "$@"