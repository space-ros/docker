#!/bin/bash
set -e

# Setup the MoveIt2 environment
source "/home/spaceros-user/src/spaceros/install/setup.bash"
exec "$@"
